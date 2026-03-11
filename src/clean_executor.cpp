#include <algorithm>
#include <cmath>
#include <mutex>
#include <string>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

namespace {

double clampValue(double value, double min_value, double max_value) {
  return std::max(min_value, std::min(max_value, value));
}

double distance2D(const geometry_msgs::Point& a, const geometry_msgs::Point& b) {
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  return std::hypot(dx, dy);
}

double shortestAngle(double from, double to) {
  return std::atan2(std::sin(to - from), std::cos(to - from));
}

}  // namespace

class CleanExecutor {
public:
  CleanExecutor()
      : private_nh_("~"),
        state_(IDLE),
        resume_state_(IDLE),
        track_mode_(TRACK_NONE),
        current_index_(0),
        last_status_progress_(-1) {
    private_nh_.param("control_frequency", control_frequency_, 15.0);

    wall_profile_.lookahead = 0.20;
    wall_profile_.linear_speed = 0.12;
    wall_profile_.min_linear_speed = 0.05;
    wall_profile_.angular_gain = 2.2;
    wall_profile_.yaw_gain = 0.6;
    wall_profile_.max_angular_speed = 0.8;
    wall_profile_.point_tolerance = 0.10;
    wall_profile_.goal_tolerance = 0.08;
    wall_profile_.goal_yaw_tolerance = 0.35;
    wall_profile_.heading_stop = 0.85;

    cover_profile_.lookahead = 0.45;
    cover_profile_.linear_speed = 0.20;
    cover_profile_.min_linear_speed = 0.08;
    cover_profile_.angular_gain = 1.5;
    cover_profile_.yaw_gain = 0.35;
    cover_profile_.max_angular_speed = 0.9;
    cover_profile_.point_tolerance = 0.16;
    cover_profile_.goal_tolerance = 0.10;
    cover_profile_.goal_yaw_tolerance = 0.45;
    cover_profile_.heading_stop = 1.10;

    control_sub_ = nh_.subscribe("/robot_api/clean_executor/control", 10,
                                 &CleanExecutor::controlCB, this);
    wall_sub_ = nh_.subscribe("wall_trajectory", 1, &CleanExecutor::wallPathCB, this);
    cover_sub_ = nh_.subscribe("clean_trajectory", 1, &CleanExecutor::coverPathCB, this);

    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    status_pub_ = nh_.advertise<std_msgs::String>("/robot_api/clean_executor/status", 1, true);
    clean_room_path_pub_ = nh_.advertise<nav_msgs::Path>("/clean_room/path", 1, true);

    control_timer_ =
        nh_.createTimer(ros::Duration(1.0 / control_frequency_), &CleanExecutor::controlLoop, this);

    publishStatus("idle", 0);
  }

private:
  enum State {
    IDLE = 0,
    WAIT_WALL,
    EXEC_WALL,
    WAIT_COVER,
    EXEC_COVER,
    PAUSED,
    BLOCKED
  };

  enum TrackMode {
    TRACK_NONE = 0,
    TRACK_WALL,
    TRACK_COVER
  };

  struct TrackingProfile {
    double lookahead;
    double linear_speed;
    double min_linear_speed;
    double angular_gain;
    double yaw_gain;
    double max_angular_speed;
    double point_tolerance;
    double goal_tolerance;
    double goal_yaw_tolerance;
    double heading_stop;
  };

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber control_sub_;
  ros::Subscriber wall_sub_;
  ros::Subscriber cover_sub_;
  ros::Publisher vel_pub_;
  ros::Publisher status_pub_;
  ros::Publisher clean_room_path_pub_;
  ros::Timer control_timer_;
  tf::TransformListener tf_listener_;
  std::mutex mutex_;

  State state_;
  State resume_state_;
  TrackMode track_mode_;
  nav_msgs::Path wall_path_;
  nav_msgs::Path cover_path_;
  nav_msgs::Path active_path_;
  size_t current_index_;
  double control_frequency_;
  TrackingProfile wall_profile_;
  TrackingProfile cover_profile_;
  std::string last_status_name_;
  int last_status_progress_;

  bool lookupRobotPose(geometry_msgs::PoseStamped* pose_msg) {
    tf::StampedTransform transform;
    try {
      tf_listener_.lookupTransform("map", "base_footprint", ros::Time(0.0), transform);
    } catch (tf::TransformException& e) {
      ROS_WARN_THROTTLE(1.0, "CleanExecutor: cannot get robot pose: %s", e.what());
      return false;
    }

    pose_msg->header.frame_id = "map";
    pose_msg->header.stamp = ros::Time::now();
    pose_msg->pose.position.x = transform.getOrigin().x();
    pose_msg->pose.position.y = transform.getOrigin().y();
    pose_msg->pose.position.z = transform.getOrigin().z();
    pose_msg->pose.orientation.x = transform.getRotation().x();
    pose_msg->pose.orientation.y = transform.getRotation().y();
    pose_msg->pose.orientation.z = transform.getRotation().z();
    pose_msg->pose.orientation.w = transform.getRotation().w();
    return true;
  }

  void stopRobot() {
    vel_pub_.publish(geometry_msgs::Twist());
  }

  void publishStatus(const std::string& state_name, int progress) {
    progress = std::max(0, std::min(100, progress));
    if (state_name == last_status_name_ && progress == last_status_progress_) {
      return;
    }
    last_status_name_ = state_name;
    last_status_progress_ = progress;

    std_msgs::String msg;
    msg.data = state_name + " " + std::to_string(progress);
    status_pub_.publish(msg);
  }

  void publishCleanRoomPath(const nav_msgs::Path& path) {
    clean_room_path_pub_.publish(path);
  }

  void clearExecution() {
    state_ = IDLE;
    resume_state_ = IDLE;
    track_mode_ = TRACK_NONE;
    active_path_.poses.clear();
    active_path_.header.frame_id = "map";
    current_index_ = 0;
    wall_path_.poses.clear();
    cover_path_.poses.clear();

    nav_msgs::Path empty_path;
    empty_path.header.frame_id = "map";
    empty_path.header.stamp = ros::Time::now();
    publishCleanRoomPath(empty_path);
  }

  size_t findNearestIndex(const nav_msgs::Path& path, const geometry_msgs::PoseStamped& robot_pose,
                          size_t start_index) const {
    if (path.poses.empty()) {
      return 0;
    }
    const size_t search_end = std::min(path.poses.size() - 1, start_index + 40);
    size_t nearest = std::min(start_index, path.poses.size() - 1);
    double best_distance =
        distance2D(path.poses[nearest].pose.position, robot_pose.pose.position);
    for (size_t i = nearest; i <= search_end; ++i) {
      const double distance =
          distance2D(path.poses[i].pose.position, robot_pose.pose.position);
      if (distance < best_distance) {
        best_distance = distance;
        nearest = i;
      }
    }
    return nearest;
  }

  size_t lookaheadIndex(const nav_msgs::Path& path, size_t start_index,
                        double lookahead) const {
    if (path.poses.empty()) {
      return 0;
    }
    size_t index = std::min(start_index, path.poses.size() - 1);
    double accumulated = 0.0;
    while (index + 1 < path.poses.size() && accumulated < lookahead) {
      accumulated += distance2D(path.poses[index].pose.position,
                                path.poses[index + 1].pose.position);
      ++index;
    }
    return index;
  }

  int progressPercent(const nav_msgs::Path& path, size_t current_index) const {
    if (path.poses.size() <= 1) {
      return path.poses.empty() ? 0 : 100;
    }
    const double progress =
        (100.0 * std::min(current_index, path.poses.size() - 1)) / (path.poses.size() - 1);
    return static_cast<int>(progress);
  }

  std::string stateName(State state) const {
    switch (state) {
      case WAIT_WALL:
        return "wait_wall";
      case EXEC_WALL:
        return "wall";
      case WAIT_COVER:
        return "wait_cover";
      case EXEC_COVER:
        return "cover";
      case PAUSED:
        return "paused";
      case BLOCKED:
        return "blocked";
      case IDLE:
      default:
        return "idle";
    }
  }

  void startTracking(const nav_msgs::Path& path, State state,
                     const geometry_msgs::PoseStamped* robot_pose = nullptr) {
    if (path.poses.empty()) {
      return;
    }
    active_path_ = path;
    if (active_path_.header.frame_id.empty()) {
      active_path_.header.frame_id = "map";
    }
    state_ = state;
    resume_state_ = state;
    track_mode_ = (state == EXEC_WALL) ? TRACK_WALL : TRACK_COVER;
    current_index_ = 0;
    if (robot_pose != nullptr) {
      current_index_ = findNearestIndex(active_path_, *robot_pose, 0);
    }

    if (state == EXEC_WALL) {
      publishCleanRoomPath(wall_path_);
    } else if (state == EXEC_COVER) {
      publishCleanRoomPath(cover_path_);
    }

    publishStatus(stateName(state_), progressPercent(active_path_, current_index_));
  }

  void finishCurrentSegment(const geometry_msgs::PoseStamped& robot_pose) {
    if (state_ == EXEC_WALL) {
      if (!cover_path_.poses.empty()) {
        startTracking(cover_path_, EXEC_COVER, &robot_pose);
      } else {
        stopRobot();
        state_ = WAIT_COVER;
        resume_state_ = WAIT_COVER;
        track_mode_ = TRACK_NONE;
        active_path_.poses.clear();
        current_index_ = 0;
        publishStatus("wait_cover", 0);
      }
      return;
    }

    stopRobot();
    clearExecution();
    publishStatus("completed", 100);
  }

  void controlCB(const std_msgs::String::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (msg->data == "start") {
      stopRobot();
      clearExecution();
      state_ = WAIT_WALL;
      resume_state_ = WAIT_WALL;
      publishStatus("wait_wall", 0);
      return;
    }
    if (msg->data == "pause") {
      if (state_ != IDLE && state_ != PAUSED) {
        stopRobot();
        resume_state_ = (state_ == BLOCKED) ? resume_state_ : state_;
        state_ = PAUSED;
        const int progress = active_path_.poses.empty() ? 0 : progressPercent(active_path_, current_index_);
        publishStatus("paused", progress);
      }
      return;
    }
    if (msg->data == "resume") {
      if (state_ == PAUSED) {
        if (resume_state_ == WAIT_WALL) {
          geometry_msgs::PoseStamped robot_pose;
          if (!wall_path_.poses.empty()) {
            const bool has_pose = lookupRobotPose(&robot_pose);
            startTracking(wall_path_, EXEC_WALL, has_pose ? &robot_pose : nullptr);
          } else {
            state_ = WAIT_WALL;
            publishStatus("wait_wall", 0);
          }
        } else if (resume_state_ == WAIT_COVER) {
          geometry_msgs::PoseStamped robot_pose;
          if (!cover_path_.poses.empty()) {
            const bool has_pose = lookupRobotPose(&robot_pose);
            startTracking(cover_path_, EXEC_COVER, has_pose ? &robot_pose : nullptr);
          } else {
            state_ = WAIT_COVER;
            publishStatus("wait_cover", 0);
          }
        } else {
          state_ = resume_state_;
          publishStatus(stateName(state_), progressPercent(active_path_, current_index_));
        }
      }
      return;
    }
    if (msg->data == "cancel") {
      stopRobot();
      clearExecution();
      publishStatus("cancelled", 0);
    }
  }

  void wallPathCB(const nav_msgs::Path::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    wall_path_ = *msg;
    if (state_ == WAIT_WALL && !wall_path_.poses.empty()) {
      geometry_msgs::PoseStamped robot_pose;
      if (lookupRobotPose(&robot_pose)) {
        startTracking(wall_path_, EXEC_WALL, &robot_pose);
      } else {
        startTracking(wall_path_, EXEC_WALL, nullptr);
      }
    }
  }

  void coverPathCB(const nav_msgs::Path::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    cover_path_ = *msg;
    if (cover_path_.poses.empty()) {
      return;
    }
    geometry_msgs::PoseStamped robot_pose;
    const bool has_pose = lookupRobotPose(&robot_pose);
    if (state_ == WAIT_COVER) {
      startTracking(cover_path_, EXEC_COVER, has_pose ? &robot_pose : nullptr);
    } else if (state_ == EXEC_COVER || (state_ == BLOCKED && resume_state_ == EXEC_COVER)) {
      startTracking(cover_path_, EXEC_COVER, has_pose ? &robot_pose : nullptr);
    }
  }

  void controlLoop(const ros::TimerEvent&) {
    geometry_msgs::PoseStamped robot_pose;
    if (!lookupRobotPose(&robot_pose)) {
      return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ == IDLE || state_ == WAIT_WALL || state_ == WAIT_COVER || state_ == PAUSED) {
      stopRobot();
      return;
    }

    const TrackingProfile& profile =
        (track_mode_ == TRACK_WALL) ? wall_profile_ : cover_profile_;

    if (state_ == BLOCKED) {
      state_ = resume_state_;
      publishStatus(stateName(state_), progressPercent(active_path_, current_index_));
    }

    if (active_path_.poses.empty()) {
      stopRobot();
      return;
    }

    current_index_ = findNearestIndex(active_path_, robot_pose, current_index_);
    while (current_index_ + 1 < active_path_.poses.size() &&
           distance2D(active_path_.poses[current_index_].pose.position, robot_pose.pose.position) <=
               profile.point_tolerance) {
      ++current_index_;
    }

    const geometry_msgs::PoseStamped& goal_pose = active_path_.poses.back();
    const double goal_distance = distance2D(goal_pose.pose.position, robot_pose.pose.position);
    const double robot_yaw = tf::getYaw(robot_pose.pose.orientation);
    const double goal_yaw = tf::getYaw(goal_pose.pose.orientation);
    const double goal_yaw_error = shortestAngle(robot_yaw, goal_yaw);

    if (goal_distance <= profile.goal_tolerance &&
        std::fabs(goal_yaw_error) <= profile.goal_yaw_tolerance) {
      finishCurrentSegment(robot_pose);
      return;
    }

    const size_t target_index = lookaheadIndex(active_path_, current_index_, profile.lookahead);
    const geometry_msgs::PoseStamped& target_pose = active_path_.poses[target_index];
    const double dx = target_pose.pose.position.x - robot_pose.pose.position.x;
    const double dy = target_pose.pose.position.y - robot_pose.pose.position.y;
    const double local_x = std::cos(robot_yaw) * dx + std::sin(robot_yaw) * dy;
    const double local_y = -std::sin(robot_yaw) * dx + std::cos(robot_yaw) * dy;
    const double heading_error = std::atan2(local_y, std::max(local_x, 1e-3));
    const double target_yaw = tf::getYaw(target_pose.pose.orientation);
    const double target_yaw_error = shortestAngle(robot_yaw, target_yaw);

    geometry_msgs::Twist cmd;
    const bool rotate_in_place = local_x < 0.02 || std::fabs(heading_error) > profile.heading_stop;
    if (!rotate_in_place) {
      const double speed_scale = std::max(0.2, 1.0 - std::min(std::fabs(heading_error), 1.4) / 1.4);
      cmd.linear.x =
          std::max(profile.min_linear_speed, profile.linear_speed * speed_scale);
    }
    cmd.angular.z = profile.angular_gain * heading_error + profile.yaw_gain * target_yaw_error;
    cmd.angular.z = clampValue(cmd.angular.z, -profile.max_angular_speed, profile.max_angular_speed);

    if (goal_distance <= profile.goal_tolerance * 2.0) {
      cmd.linear.x = std::min(cmd.linear.x, profile.min_linear_speed);
    }

    vel_pub_.publish(cmd);
    publishStatus(stateName(state_), progressPercent(active_path_, current_index_));
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "clean_executor");
  CleanExecutor node;
  ros::spin();
  return 0;
}

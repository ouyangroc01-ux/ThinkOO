cmake_minimum_required(VERSION 3.5)
project(sensor_handle)

# Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Append local cmake module path so FindGTSAM.cmake is found
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

## Find catkin and required ROS components
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf
  tf2
  tf2_ros
)

## Find PCL
find_package(PCL REQUIRED COMPONENTS
  common
  kdtree
  octree
  search
  filters
  io
  features
  segmentation
  visualization
  surface
  registration
  keypoints
  tracking
  recognition
)

## Find OpenCV
find_package(OpenCV REQUIRED)

## Find Eigen3
find_package(Eigen3 REQUIRED)

## Find Ceres Solver
find_package(Ceres REQUIRED)

## Find GTSAM (optional — build without GTSAM features if not installed)
## To provide a custom GTSAM install path set GTSAM_DIR or extend
## CMAKE_PREFIX_PATH before invoking cmake, for example:
##   cmake .. -DGTSAM_DIR=/usr/local/lib/cmake/GTSAM
find_package(GTSAM QUIET)
if(GTSAM_FOUND)
  message(STATUS "Found GTSAM: ${GTSAM_INCLUDE_DIRS}")
else()
  message(WARNING
    "GTSAM not found — sensor_handle will be built WITHOUT GTSAM support.\n"
    "To enable GTSAM features install GTSAM (https://gtsam.org) and re-run cmake.\n"
    "You can set GTSAM_DIR to the directory containing GTSAMConfig.cmake."
  )
endif()

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp
    rospy
    std_msgs
    sensor_msgs
    geometry_msgs
    nav_msgs
    tf
    tf2
    tf2_ros
  DEPENDS
    PCL
    OpenCV
    EIGEN3
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${CERES_INCLUDE_DIRS}
)

if(GTSAM_FOUND)
  include_directories(${GTSAM_INCLUDE_DIRS})
endif()

## Collect source files
file(GLOB_RECURSE SENSOR_HANDLE_SOURCES
  src/*.cpp
  src/*.cc
)

if(SENSOR_HANDLE_SOURCES)
  add_library(${PROJECT_NAME}
    ${SENSOR_HANDLE_SOURCES}
  )

  target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${CERES_LIBRARIES}
  )

  if(GTSAM_FOUND)
    target_compile_definitions(${PROJECT_NAME} PRIVATE WITH_GTSAM)
    target_link_libraries(${PROJECT_NAME}
      ${GTSAM_LIBRARIES}
    )
  endif()

  ## Install targets
  install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )
endif()

## Install headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

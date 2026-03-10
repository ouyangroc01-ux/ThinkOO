# FindGTSAM.cmake
# Finds the GTSAM library and its dependencies.
#
# This module defines:
#   GTSAM_FOUND        - True if GTSAM was found
#   GTSAM_INCLUDE_DIRS - Include directories for GTSAM headers
#   GTSAM_LIBRARIES    - Libraries to link against
#   GTSAM_VERSION      - Version string (if available)
#
# To hint the location of GTSAM, set GTSAM_DIR or add the GTSAM install
# prefix to CMAKE_PREFIX_PATH before calling find_package(GTSAM).

# Search for the GTSAM config file in common install locations
find_package(GTSAM CONFIG QUIET
  HINTS
    ${GTSAM_DIR}
    /usr/local
    /usr
    /opt/gtsam
    /opt/ros/$ENV{ROS_DISTRO}
  PATH_SUFFIXES
    lib/cmake/GTSAM
    lib/cmake/gtsam
    share/GTSAM/cmake
    share/gtsam/cmake
)

if(GTSAM_FOUND)
  if(NOT GTSAM_FIND_QUIETLY)
    message(STATUS "Found GTSAM via config (version ${GTSAM_VERSION})")
  endif()
  return()
endif()

# Fall back to manual search for header and library
find_path(GTSAM_INCLUDE_DIR
  NAMES gtsam/base/Vector.h
  HINTS
    ${GTSAM_DIR}/include
    /usr/local/include
    /usr/include
    /opt/gtsam/include
)

find_library(GTSAM_LIBRARY
  NAMES gtsam
  HINTS
    ${GTSAM_DIR}/lib
    /usr/local/lib
    /usr/lib
    /usr/lib/x86_64-linux-gnu
    /opt/gtsam/lib
)

find_library(GTSAM_UNSTABLE_LIBRARY
  NAMES gtsam_unstable
  HINTS
    ${GTSAM_DIR}/lib
    /usr/local/lib
    /usr/lib
    /usr/lib/x86_64-linux-gnu
    /opt/gtsam/lib
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(GTSAM
  REQUIRED_VARS GTSAM_INCLUDE_DIR GTSAM_LIBRARY
)

if(GTSAM_FOUND)
  set(GTSAM_INCLUDE_DIRS ${GTSAM_INCLUDE_DIR})
  set(GTSAM_LIBRARIES ${GTSAM_LIBRARY})
  if(GTSAM_UNSTABLE_LIBRARY)
    list(APPEND GTSAM_LIBRARIES ${GTSAM_UNSTABLE_LIBRARY})
  endif()

  if(NOT TARGET GTSAM::GTSAM)
    add_library(GTSAM::GTSAM UNKNOWN IMPORTED)
    set_target_properties(GTSAM::GTSAM PROPERTIES
      IMPORTED_LOCATION "${GTSAM_LIBRARY}"
      INTERFACE_INCLUDE_DIRECTORIES "${GTSAM_INCLUDE_DIRS}"
    )
  endif()
endif()

mark_as_advanced(GTSAM_INCLUDE_DIR GTSAM_LIBRARY GTSAM_UNSTABLE_LIBRARY)

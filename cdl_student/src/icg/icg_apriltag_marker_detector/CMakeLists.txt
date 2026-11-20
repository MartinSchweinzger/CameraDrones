cmake_minimum_required(VERSION 3.0.2)
project(icg_apriltag_marker_detector)

set(CMAKE_BUILD_TYPE Release)
#set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_VERBOSE_MAKEFILE false)

#------------- PARAMETERS ------------------------------------
#set(OpenCV_DIR $ENV{OpenCV_DIR})
set(APRILTAG_DIR $ENV{APRILTAG_DIR})
set(YAML_CPP_INSTALL_DIR $ENV{YAML_CPP_INSTALL_DIR})

#-------------CATKIN build system ---------------------------
find_package(catkin REQUIRED COMPONENTS
  image_transport
  std_msgs
  cv_bridge
  sensor_msgs
  mav_msgs
  tf2
  tf2_ros
  roscpp
  cdl_common 
)

IF(catkin_FOUND)
  catkin_package(
    INCLUDE_DIRS include
    LIBRARIES icg_apriltag_marker_detector_ros icg_apriltag_marker_detector_ros
    CATKIN_DEPENDS image_transport std_msgs cv_bridge sensor_msgs mav_msgs tf2 tf2_ros roscpp cdl_common
  )
ENDIF(catkin_FOUND)

SET( EXTRA_LIBS
    ${EXTRA_LIBS}
    ${catkin_LIBRARIES}
)

find_package(cmake_modules REQUIRED)

find_package(Eigen) 
find_package(Eigen3) 

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
  ${EIGEN_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIR}
)
#-------------CATKIN end-------------------------------------

#ADD_DEFINITIONS("-std=c++0x")

#-----------------------------------------------------------------------------

# OpenCV for image handling
# don't use opencv as found by any prior running of find_package

find_package(OpenCV REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )
SET( EXTRA_LIBS 
  ${EXTRA_LIBS}
  ${OpenCV_LIBS}
)


# Apriltag
SET( EXTRA_LIBS
  ${EXTRA_LIBS}
  apriltags
)
INCLUDE_DIRECTORIES( ${APRILTAG_DIR}/src )
LINK_DIRECTORIES( ${APRILTAG_DIR}/build )

# yaml-cpp
SET( EXTRA_LIBS
  ${EXTRA_LIBS}
  yaml-cpp
)
INCLUDE_DIRECTORIES( ${YAML_CPP_INSTALL_DIR}/include )
LINK_DIRECTORIES( ${YAML_CPP_INSTALL_DIR}/lib )

LINK_DIRECTORIES(/usr/local/lib)


# --- Icg Apriltag Marker Detector Library ---
SET( SRC
  src/${PROJECT_NAME}/icg_apriltag_marker_detector_ros.cpp
)

SET( HDR
  include/${PROJECT_NAME}/icg_apriltag_marker_detector_ros.h
)

add_library(icg_apriltag_marker_detector_ros SHARED ${SRC} ${HDR})
TARGET_LINK_LIBRARIES(icg_apriltag_marker_detector_ros
    ${EXTRA_LIBS}
    ${OpenCV_LIBS}
)
# -----------------------------

# --- Icg Apriltag Marker Detector Node / Executable ---
set (EXTRA_LIBS
    ${EXTRA_LIBS}
    icg_apriltag_marker_detector_ros
)

##
add_executable(icg_apriltag_marker_detector_ros_node src/main_ros_node.cpp )
target_link_libraries(icg_apriltag_marker_detector_ros_node
    ${EXTRA_LIBS}
    ${OpenCV_LIBS}
)
# -----------------------------

## Mark executables and/or libraries for installation
install(TARGETS icg_apriltag_marker_detector_ros_node icg_apriltag_marker_detector_ros
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY ./
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".git" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES launch/icg_pariltag_marker_detector_guidance.launch launch/icg_pariltag_marker_detector.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

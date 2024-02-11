cmake_minimum_required(VERSION 3.5.1)
project(pcl_labeling)

set(CMAKE_CXX_STANDARD 14)
add_compile_options(-Wall -Wextra -Wpedantic)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CATKIN_PACKAGE_DEPENDENCIES
    cv_bridge
    pcl_conversions
    pcl_ros
    sensor_msgs
    std_msgs
    roscpp
)

# TODO
set(SRC_FILES
  src/PointLabeler.cpp
  src/PointLabelerRos.cpp
  src/LidarCameraProjector.cpp
)

find_package(catkin REQUIRED
  COMPONENTS
    ${CATKIN_PACKAGE_DEPENDENCIES}
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_DEPENDENCIES}
  DEPENDS
    PCL
)

###########
## Build ##
###########
SET(EIGEN3_INCLUDE_DIRS "../eigen") # Adapt if necessary

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  SYSTEM
    ${PCL_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)

link_directories( #only needed for pcl
  ${PCL_LIBRARY_DIRS}
)

add_definitions( #only needed for pcl
  ${PCL_DEFINITIONS}
)

add_library(${PROJECT_NAME}
  ${SRC_FILES}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PCL_COMMON_LIBRARIES}
  ${PCL_IO_LIBRARIES}
)

# TODO
add_executable(pcl_labeling_node
  src/pcl_labeling_node.cpp
)

# TODO
target_link_libraries(pcl_labeling_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PCL_COMMON_LIBRARIES}
  ${PCL_IO_LIBRARIES}
)

#############
## Install ##
#############
#install(DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#  PATTERN ".svn" EXCLUDE
#)
#install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node point_cloud_stitcher_node
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)
#
#install(DIRECTORY
#  launch
#  param
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)

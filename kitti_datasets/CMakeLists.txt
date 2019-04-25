cmake_minimum_required(VERSION 2.8.3)
project(kitti_datasets)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
    nav_msgs
    roscpp
    rosconsole
    sensor_msgs
    std_msgs
    rospy
    pcl_conversions
    pcl_ros
    tf
    cv_bridge
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES kitti_datasets
#  CATKIN_DEPENDS roscpp rospy
#  DEPENDS system_lib
)



## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(sync_dados src/sync_dados.cpp)
target_link_libraries(sync_dados
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)


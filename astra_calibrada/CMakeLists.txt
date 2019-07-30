cmake_minimum_required(VERSION 2.8.3)
project(astra_calibrada)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
    tf2_ros
    nav_msgs
    roscpp
    rosconsole
    sensor_msgs
    image_transport
    dynamic_reconfigure
    urdf
    std_msgs
    rospy
    pcl_conversions
    pcl_ros
    tf
    image_geometry
    cv_bridge)

find_package(OpenCV REQUIRED)
find_package(PCL 1.2 REQUIRED)
INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS} "/usr/local/include")
LINK_DIRECTORIES(${PCL_LIBRARY_DIRS} "/usr/local/lib")
ADD_DEFINITIONS(${PCL_DEFINITIONS})
INCLUDE_DIRECTORIES(${catkin_INCLUDE_DIRS})
find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
  message(STATUS "OPENMP FOUND")
  set(OpenMP_FLAGS ${OpenMP_CXX_FLAGS})  # or if you use C: ${OpenMP_C_FLAGS}
  set(OpenMP_LIBS gomp)
endif()

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${CUDA_INCLUDE_DIRS})

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

# Parametros ajustaveis zed-astra calibracao
generate_dynamic_reconfigure_options(cfg/params.cfg)
#include_directories(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES astra_calibrada
#  CATKIN_DEPENDS other_catkin_pkg
  DEPENDS OpenMP
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
 ${catkin_INCLUDE_DIRS}
 ${OpenCV_INCLUDE_DIRS}
 ${OpenMP_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/astra_calibrada.cpp
# )

add_executable(astra_calibrada src/astra_calibrada.cpp)
#target_compile_options(astra_calibrada PRIVATE ${OpenMP_FLAGS})
add_dependencies(astra_calibrada ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})
target_link_libraries(astra_calibrada
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  ${roslib_LIBRARIES}
  ${OpenMP_LIBS}
)

add_executable(astra_calibrada_felipe src/astra_calibrada_felipe.cpp)
target_link_libraries(astra_calibrada_felipe
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  ${roslib_LIBRARIES}
)

add_executable(calibrar_astra_online src/calibrar_astra_online.cpp)
target_link_libraries(calibrar_astra_online
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  ${roslib_LIBRARIES}
)

add_executable(calibrar_astra_zed src/calibrar_astra_zed.cpp)
target_link_libraries(calibrar_astra_zed
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  ${roslib_LIBRARIES}
)

#add_executable(viewer viewer.cpp)
#target_link_libraries(viewer ${PCL_LIBRARIES} ${catkin_LIBRARIES} ${roslib_LIBRARIES})

#add_executable(sincroniza_odom sincroniza_odom.cpp)
#target_link_libraries(sincroniza_odom ${PCL_LIBRARIES} ${catkin_LIBRARIES} ${roslib_LIBRARIES})

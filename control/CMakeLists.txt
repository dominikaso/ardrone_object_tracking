cmake_minimum_required(VERSION 2.8.3)
project(control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  darknet_ros_msgs
  sensor_msgs
  geometry_msgs
  cv_bridge
  image_transport
  message_generation
  ardrone_autonomy
  keyboard
)

find_package(OpenCV)

add_message_files(
  FILES
  ColorMsg.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

include_directories(
include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES compute_cdist
  CATKIN_DEPENDS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  darknet_ros_msgs
  ardrone_autonomy
  cv_bridge
  image_transport
  message_runtime
  keyboard
#  DEPENDS assimp Eigen mlpack
)

add_executable(control src/uavControlMain.cpp src/uavControl.cpp)

target_link_libraries(control
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
 )

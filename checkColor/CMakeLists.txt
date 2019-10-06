cmake_minimum_required(VERSION 2.8.3)
project(objectColor)

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
  cv_bridge
  image_transport
  message_generation
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

find_package(OpenCV)

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
  darknet_ros_msgs
  cv_bridge
  image_transport
  message_runtime
#  DEPENDS assimp Eigen mlpack
)



add_executable(objectColor src/checkColorMain.cpp src/checkColor.cpp)

target_link_libraries(objectColor
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
 )


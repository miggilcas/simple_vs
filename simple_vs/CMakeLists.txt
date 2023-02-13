cmake_minimum_required(VERSION 3.0.2)
project(simple_vs)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
)
find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})

catkin_package(
  CATKIN_DEPENDS cv_bridge image_transport roscpp rospy sensor_msgs std_msgs
)

###########
## Build ##
###########

include_directories(  ${catkin_INCLUDE_DIRS} )

add_executable(video_streamer src/video_streamer.cpp)
target_link_libraries(video_streamer ${catkin_LIBRARIES})
target_link_libraries(video_streamer ${OpenCV_LIBRARIES})


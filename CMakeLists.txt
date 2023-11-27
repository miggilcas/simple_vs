cmake_minimum_required(VERSION 3.0.2)
project(simple_vs)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

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
  std_srvs
  geometry_msgs
  theora_image_transport
)


find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})


## Load FFMPEG
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBAV REQUIRED IMPORTED_TARGET
  libavformat
  libavutil
  libavcodec
	libswscale
)

catkin_package(
  CATKIN_DEPENDS cv_bridge image_transport roscpp rospy sensor_msgs std_msgs std_srvs
)
catkin_install_python(PROGRAMS  scripts/readMAV-ROS.py scripts/srv_catec.py scripts/theora_video_streamer.py scripts/streamTopic.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

###########
## Build ##
###########

include_directories(  ${catkin_INCLUDE_DIRS} )

add_executable(video_streamer src/video_streamer.cpp )
add_executable(video_streamer_dji src/video_streamer_dji.cpp )
add_executable(video_streamer_dji_h264 src/video_streamer_dji_H264.cpp )
add_executable(video_stream_compress src/video_streamer_compress.cpp)
add_executable(video_resize src/video_resize.cpp )

target_link_libraries(video_streamer ${catkin_LIBRARIES}
                                       ${OpenCV_LIBRARIES})

target_link_libraries(video_stream_compress ${catkin_LIBRARIES}
                                            ${OpenCV_LIBRARIES})

target_link_libraries(video_resize ${catkin_LIBRARIES}
                                    ${OpenCV_LIBRARIES})

target_link_libraries(video_streamer_dji ${catkin_LIBRARIES}  
                                          ${OpenCV_LIBRARIES})

target_link_libraries(video_streamer_dji_h264 ${catkin_LIBRARIES}
                                              ${OpenCV_LIBRARIES}
                                              avformat avcodec swscale avutil)


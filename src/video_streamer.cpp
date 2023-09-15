// @Description: This node publishes a frame in color (bgr8), from a usb connected camera, in the topic specified in launch
// Source: http://wiki.ros.org/image_transport/Tutorials/PublishingImages
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <string>
#include <iostream>

using namespace cv;

int main(int argc, char** argv)
{
  int fps=30;
  int height;
  int width;
  int cam = 0;
  std::string path_video;

  
  ros::init(argc, argv, "video_stream");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  std::string ns = ros::this_node::getNamespace();
  std::string path_topic = "/" + ns + "/video_stream";
  
  image_transport::Publisher pub = it.advertise(path_topic, 1);

  nh.getParam("video_streamer/frame_height",height);
  nh.getParam("video_streamer/frame_width",width);
  nh.getParam("video_streamer/fps",fps);
  nh.getParam("video_streamer/path_video",path_video);

  ROS_INFO("fps: %d ",fps);
  ROS_INFO("Resolution: %d x %d ",width,height);
  ROS_INFO_STREAM(path_video);
  VideoCapture cap(cam); 

  if(!cap.isOpened()) return 1;
  Mat frame,send;
  sensor_msgs::ImagePtr msg;

  ros::Rate loop_rate(fps);

  while (nh.ok()) {
    
    cap >> frame; // Read a new frame
    if(!frame.empty()){
        //resize(frame, send, Size(width, height), 0, 0, INTER_LINEAR);

        msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8",frame).toImageMsg();

        pub.publish(msg); 

    }

    ros::spinOnce();
    loop_rate.sleep();
  }
}

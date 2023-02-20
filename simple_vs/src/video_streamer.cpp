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

  
  ros::init(argc, argv, "video_stream");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  std::string ns = ros::this_node::getNamespace();
  std::string path_topic = "/" + ns + "/video_stream";
  
  image_transport::Publisher pub = it.advertise(path_topic, 1);

    ros::param::get(path_topic+"/frame_height",height);
    ros::param::get(path_topic+"/frame_width",width);
  
  ROS_INFO("fps: %d ",fps);
  ROS_INFO("Resolution: %d x %d ",width,height);
  VideoCapture cap(cam); // '0' to use the laptop webcam
  if(!cap.isOpened()) return 1;
  Mat frame,send;
  sensor_msgs::ImagePtr msg;
  //std::cout << "Namespace  : " << ns << std::endl ;
  ros::Rate loop_rate(fps);
  while (nh.ok()) {
    
    cap >> frame; // Read a new frame
    if(!frame.empty()){
        resize(frame, send, Size(width, height), 0, 0, INTER_LINEAR);

        msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8",send).toImageMsg();

        pub.publish(msg); 

    }
    //  ROS_INFO("Resolution: %d x %d ",width,height);

    
    ros::spinOnce();
    loop_rate.sleep();
  }
 
}
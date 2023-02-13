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
  int height=720;
  int width=1080;
  std::string ns ;
  
  ros::param::param<std::string>("ns", ns, "uav_3");
  ros::param::param<int>("fps", fps, 30);
  ros::param::param<int>("height", height, 720);
  ros::param::param<int>("width", width, 1080);

  //Get param values:
  ros::param::get("~ns",ns);
  ros::param::get("~fps",fps);
  ros::param::get("~height",height);
  ros::param::get("~width",width);
  
  ros::param::get("~ns",ns);
  std::string pub_topic_name = "/" + ns + "/video_stream";
  ros::init(argc, argv, "video_streamer");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise(pub_topic_name, 1);


  

  
  VideoCapture cap(0); // '0' to use the laptop webcam
  if(!cap.isOpened()) return 1;
  Mat frame,send;
  sensor_msgs::ImagePtr msg;
  std::cout << "Namespace  : " << ns << std::endl ;
  ros::Rate loop_rate(fps);
  while (nh.ok()) {
    cap >> frame; // Read a new frame
    if(!frame.empty()){
        resize(frame, send, Size(width, height), 0, 0, INTER_LINEAR);

        msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8",send).toImageMsg();

        pub.publish(msg); 

    }
    
    ros::spinOnce();
    loop_rate.sleep();
  }
 
}
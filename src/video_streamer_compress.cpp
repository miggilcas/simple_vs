// @Description: This node publishes a frame in color (bgr8), from a usb connected camera, in the topic specified in launch
// Source: http://wiki.ros.org/image_transport/Tutorials/PublishingImages


#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgcodecs.hpp>

#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <string>
#include <iostream>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CompressedImage.h>

using namespace cv;
using namespace std;

int main(int argc, char** argv)
{
  int fps=30;
  int height;
  int width;
  int cam = 0;
  int source = 0;
  std::string path_video;

  
  ros::init(argc, argv, "video_stream_compress");
  ros::NodeHandle nh;
  std::string ns = ros::this_node::getNamespace();
  //image_transport::ImageTransport it(nh);
  
  std::string path_topic = "/" + ns + "/video_stream_compress";

  // Publicador del topic "image_topic"
  ros::Publisher pub = nh.advertise<sensor_msgs::CompressedImage>(path_topic, 1);  // Publicador del topic "image_topic"

  nh.getParam("video_stream_compress/frame_height",height);
  nh.getParam("video_stream_compress/frame_width",width);
  nh.getParam("video_stream_compress/fps",fps);
  nh.getParam("video_stream_compress/path_video",path_video);
  nh.getParam("video_stream_compress/source",source);

  ROS_INFO("fps: %d ",fps);
  ROS_INFO("source: %d ",source);
  ROS_INFO("Resolution: %d x %d ",width,height);
  ROS_INFO_STREAM(path_video);


  VideoCapture cap;
  // if(source == 0){
  //  cap.open(cam); // '0' to use the laptop webcam
  // }else{
  //   ROS_INFO("Video ");
  //   cap.open(path_video); 
  // }
  cap.open(source); 
  if(!cap.isOpened()) return 1;
  Mat frame,send;
  sensor_msgs::CompressedImage msg;

  ros::Rate loop_rate(fps);

  while (nh.ok()) {
    
    cap >> frame; // Read a new frame
    if(frame.empty()){
        ROS_ERROR("No se pudo capturar la imagen");
        break;
    }

    vector<uchar> buffer;
    // Escalar y Comprimir la imagen en formato JPG y empaquetarla en un mensaje de ROS
    resize(frame, send, Size(width, height), 0, 0, INTER_LINEAR);
    imencode(".jpg", send, buffer);

    msg.header.stamp = ros::Time::now();
    msg.format = "jpeg";
    msg.data = buffer;

    pub.publish(msg); 

    ros::spinOnce();
    loop_rate.sleep();
  }
 
}

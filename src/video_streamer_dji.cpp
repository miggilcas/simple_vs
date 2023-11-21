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


class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  ros::Publisher image_compress_pub_;
  sensor_msgs::CompressedImage image_compress_msg;
  int fps=30;
  int height;
  int width;
  std::string source;

public:
  ImageConverter()
    : it_(nh_)
  {
    nh_.getParam("video_streamer_dji/frame_height",height);
    nh_.getParam("video_streamer_dji/frame_width",width);
    nh_.getParam("video_streamer_dji/fps",fps);
    nh_.getParam("video_streamer_dji/source",source);
    std::string ns = ros::this_node::getNamespace();
    std::string path_subscriber = "/" + ns + "/video_stream_compress";
    std::string path_publisher1 = "/" + ns + "/video_stream_resize";
    std::string path_publisher = "/" + ns + "/video_stream_compress";

    ROS_INFO("fps: %d ",fps);
    ROS_INFO("Resolution: %d x %d ",width,height);
    ROS_INFO_STREAM(source);

    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe(source, 1,&ImageConverter::imageCb, this);
    image_compress_pub_ = nh_.advertise<sensor_msgs::CompressedImage>(path_publisher, 1);  // Publicador del topic "image_topic"
    image_pub_ = it_.advertise(path_publisher1, 1);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    ROS_INFO("publish image ");
    cv::Mat Image1;
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    ROS_INFO("publish image1 ");
    cv::resize(cv_ptr->image,Image1,cv::Size(608,448));

    // image resend
    sensor_msgs::ImagePtr mymsg;
    mymsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8",Image1).toImageMsg();
    image_pub_.publish(mymsg);
    ROS_INFO("publish image2 ");
        
    //compress image 

    vector<uchar> buffer;
    imencode(".jpg", Image1, buffer);
    image_compress_msg.header.stamp = ros::Time::now();
    image_compress_msg.format = "jpeg";
    image_compress_msg.data = buffer;

    image_compress_pub_.publish(image_compress_msg); 
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_resize");
  ImageConverter ic;
  ros::spin();
  return 0;
}


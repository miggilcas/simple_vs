//http://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <string>
#include <iostream>

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  
  int fps = 30;
  int height = 448;
  int width = 608;
  std::string source;

public:
  ImageConverter()
    : it_(nh_)
  {
    nh_.getParam("video_resize/frame_height", height);
    nh_.getParam("video_resize/frame_width", width);
    nh_.getParam("video_resize/fps", fps);
    nh_.getParam("video_resize/source", source);
    std::string ns = ros::this_node::getNamespace();
    std::string path_subscriber =  ns + source;
    std::string path_publisher =  ns  + source + "_resize";

    ROS_INFO("fps: %d ", fps);
    ROS_INFO("Resolution: %d x %d ", width, height);
    ROS_INFO_STREAM(path_subscriber);

    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe(path_subscriber, 1, &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise(path_publisher, 1);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    ROS_INFO("Callback of suscriber %d", 1);
    sensor_msgs::ImagePtr mymsg;
    cv::Mat Image1,Image2;

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

    cv::resize(cv_ptr->image,Image1,cv::Size(width,height));

    mymsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8",Image1).toImageMsg();

    // Output modified video stream
    image_pub_.publish(mymsg);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_resize");
  ImageConverter ic;
  ros::spin();
  return 0;
}
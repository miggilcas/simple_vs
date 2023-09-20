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

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/uav_2/dji_osdk_ros/main_camera_images", 1,
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/uav_2/dji_osdk_ros/main_camera_images_resize", 1);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
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

    cv::resize(cv_ptr->image,Image1,Size(608,448))

    mymsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8",Image1).toImageMsg();

    // Output modified video stream
    //image_pub_.publish(cv_ptr->toImageMsg());
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
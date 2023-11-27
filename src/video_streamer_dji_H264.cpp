// @Description: This node publishes a frame in color (bgr8), from a usb connected camera, in the topic specified in launch
// Source: http://wiki.ros.org/image_transport/Tutorials/PublishingImages

#include <ros/ros.h>

#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <cv_bridge/cv_bridge.h>
#include <string>
#include <iostream>

#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>

// INCLUDE
#include <ros/ros.h>
#include <iostream>
#include <dji_camera_image.hpp>
// #include <dji_osdk_ros/SetupCameraStream.h>
// #include <dji_osdk_ros/SetupCameraH264.h>
#include <sensor_msgs/Image.h>

extern "C"{
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libswscale/swscale.h>
}

//using namespace cv;
using namespace std;

class VideoStream
{
  private:

    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    ros::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
     sensor_msgs::ImagePtr img_msg; // >> message to be sent

    int fps = 30;
    int height;
    int width;
    std::string source;
    cv::Mat imgDJI_;

    // Decode Img
    AVCodecContext*       pCodecCtx;
    AVCodec*              pCodec;
    AVCodecParserContext* pCodecParserCtx;
    SwsContext*           pSwsCtx;
    AVFrame* pFrameYUV;
    AVFrame* pFrameRGB;
    uint8_t* rgbBuf;
    size_t   bufSize;

public:
  VideoStream(): it_(nh_)
  {
    nh_.getParam("video_streamer_dji_h264/frame_height", height);
    nh_.getParam("video_streamer_dji_h264/frame_width", width);
    nh_.getParam("video_streamer_dji_h264/fps", fps);
    nh_.getParam("video_streamer_dji_h264/source", source);
    std::string ns = ros::this_node::getNamespace();
    std::string path_subscriber =  ns + "/dji_osdk_ros/camera_h264_stream";
    std::string path_publisher =  ns + "/dji_osdk_ros/camera_h264_stream_uncompress";

    ROS_INFO("fps: %d ", fps);
    ROS_INFO("Resolution: %d x %d ", width, height);
    ROS_INFO_STREAM(path_subscriber);

    ffmpeg_init();
    // Subscrive to input video feed and publish output video feed

    image_sub_ = nh_.subscribe(path_subscriber, 10, &VideoStream::cameraH264CallBack, this);
    image_pub_ = it_.advertise(path_publisher, 1);
  }

void cameraH264CallBack(const sensor_msgs::Image& msg)
  {
    ROS_INFO("Image Decode");

  decodeToDisplay((uint8_t *)&msg.data[0], msg.data.size());
  }

  //---------------------------------------------------------------------------------------------------------------------
  // Decode from https://github.com/dji-sdk/Onboard-SDK-ROS/blob/4.1/src/dji_osdk_ros/samples/camera_stream_node.cpp#L96
void decodeToDisplay(uint8_t *buf, int bufLen)
{
    uint8_t* pData   = buf;
    int remainingLen = bufLen;
    int processedLen = 0;

    AVPacket pkt;
    av_init_packet(&pkt);
    while (remainingLen > 0)
    {
        processedLen = av_parser_parse2(pCodecParserCtx, pCodecCtx,
                                        &pkt.data, &pkt.size,
                                        pData, remainingLen,
                                        AV_NOPTS_VALUE, AV_NOPTS_VALUE, AV_NOPTS_VALUE);
        remainingLen -= processedLen;
        pData        += processedLen;

        if (pkt.size > 0)
        {
            int gotPicture = 0;
            avcodec_decode_video2(pCodecCtx, pFrameYUV, &gotPicture, &pkt);

            if (!gotPicture)
            {
                ROS_INFO("Got Frame, but no picture\n");
                continue;
            }
            else
            {
                int w = pFrameYUV->width;
                int h = pFrameYUV->height;
                ROS_INFO("Got picture! size=%dx%d\n", w, h);

                if(NULL == pSwsCtx)
                {
                    pSwsCtx = sws_getContext(w, h, pCodecCtx->pix_fmt,
                                             w, h, AV_PIX_FMT_RGB24,
                                             4, NULL, NULL, NULL);
                }

                if(NULL == rgbBuf)
                {
                    bufSize = avpicture_get_size(AV_PIX_FMT_RGB24, w, h);
                    rgbBuf = (uint8_t*) av_malloc(bufSize);
                    avpicture_fill((AVPicture*)pFrameRGB, rgbBuf, AV_PIX_FMT_RGB24, w, h);
                }

                if(NULL != pSwsCtx && NULL != rgbBuf)
                {
                    sws_scale(pSwsCtx,
                              (uint8_t const *const *) pFrameYUV->data, pFrameYUV->linesize, 0, pFrameYUV->height,
                              pFrameRGB->data, pFrameRGB->linesize);

                    pFrameRGB->height = h;
                    pFrameRGB->width = w;

                    cv::Mat mat(pFrameRGB->height, pFrameRGB->width, CV_8UC3, pFrameRGB->data[0], pFrameRGB->width * 3);
                    cv::cvtColor(mat, mat, cv::COLOR_RGB2BGR);
                    //cv::imshow("camera_stream_node", mat);
                    //cv::waitKey(1)
                    img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", mat).toImageMsg();
                    image_pub_.publish(img_msg);
                }
            }
        }
    }
    av_free_packet(&pkt);
}  

bool ffmpeg_init()
{
    avcodec_register_all();
    pCodecCtx = avcodec_alloc_context3(NULL);
    if (!pCodecCtx)
    {
        return false;
    }

    pCodecCtx->thread_count = 4;
    pCodec = avcodec_find_decoder(AV_CODEC_ID_H264);
    if (!pCodec || avcodec_open2(pCodecCtx, pCodec, NULL) < 0)
    {
        return false;
    }

    pCodecParserCtx = av_parser_init(AV_CODEC_ID_H264);
    if (!pCodecParserCtx)
    {
        return false;
    }

    pFrameYUV = av_frame_alloc();
    if (!pFrameYUV)
    {
        return false;
    }

    pFrameRGB = av_frame_alloc();
    if (!pFrameRGB)
    {
        return false;
    }

    pSwsCtx = NULL;

    pCodecCtx->flags2 |= AV_CODEC_FLAG2_SHOW_ALL;
}

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_resize");

  VideoStream node = VideoStream();
  ros::spin();
  return 0;
}

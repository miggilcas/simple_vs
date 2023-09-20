#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

topic = "/uav_2/dji_osdk_ros/main_camera_images"
width = 608#800
height = 448#720#600


bridge = CvBridge()

pub = rospy.Publisher('/uav_2/dji_osdk_ros/main_camera_images_resize', Image, queue_size=1)

def callback(msg): 
    global pub
    frame = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    frame1 = cv2.resize(frame,(width,height),interpolation = cv2.INTER_LINEAR)
    mymsg = bridge.cv2_to_imgmsg(frame1, encoding="passthrough")
    pub.publish(mymsg)


if __name__ == '__main__':
    rospy.init_node("imagetimer111", anonymous=True)
    rospy.Subscriber(topic, Image, callback)
    rospy.spin()
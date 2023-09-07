#!/usr/bin/env python3
from datetime import datetime
from time import sleep, time
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import os

import cv2
import numpy as np

fps = 30
width = 1080#800
height = 720#600
colors = [
    (0, 0, 255),
    (255, 0, 0),
    (0, 255, 0),
]

out = cv2.VideoWriter('appsrc ! videoconvert' + \
    ' ! video/x-raw,format=I420' + \
    ' ! x264enc bframes=0 tune=zerolatency speed-preset=ultrafast bitrate=600 key-int-max=' + str(fps * 2) + \
    ' ! video/x-h264,profile=baseline' + \
    ' ! rtspclientsink location=rtsp://10.222.6.2:8554/uav2_fpv',
    cv2.CAP_GSTREAMER, 0, fps, (width, height), True)
if not out.isOpened():
    raise Exception("can't open video writer")

curcolor = 0
start = time()
bridge = CvBridge()
now = time()
diff = (1 / fps) - now - start

def callback(msg):
    global now,start,diff
    frame = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    #frame = np.zeros((height, width, 3), np.uint8)

    # create a rectangle
    #color = colors[curcolor]
    #curcolor += 1
    #curcolor %= len(colors)
    #for y in range(0, int(frame.shape[0] / 2)):
    #    for x in range(0, int(frame.shape[1] / 2)):
    #        frame[y][x] = color

    out.write(frame)
    print("%s frame written to the server" % datetime.now())

    now = time()
    diff = (1 / fps) - now - start
    #if diff > 0:
    #    print("tiempo en espera")
    #    sleep(diff)
    start = now

if __name__ == '__main__':
    rospy.init_node("imagetimer111", anonymous=True)
    rospy.Subscriber("/uav_2/dji_osdk_ros/fvp_camera_images", Image, callback)
    rospy.spin()
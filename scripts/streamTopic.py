#!/usr/bin/env python3
#https://harinduravin.github.io/2021/08/18/@harinduravin-sending-opencv-frames-as-a-video-stream-using-ros.html
"""
Created on Aug 13 2021
"""
import os
import numpy as np
import cv2
import time
import rospy
import threading

from sensor_msgs.msg import Image as SensorImage

import subprocess
rtmp_url = "rtsp://127.0.0.1:8554/stream"
width = 800
height = 450
fps = 30

# command and params for ffmpeg
command = ['ffmpeg',
           '-y',
           '-f', 'rawvideo',
           '-vcodec', 'rawvideo',
           '-pix_fmt', 'bgr24',
           '-s', "{}x{}".format(width, height),
           '-r', str(fps),
           '-i', '-',
           '-c:v', 'libx264',
           '-pix_fmt', 'yuv420p',
           '-preset', 'ultrafast',
           '-f', 'rtsp','-rtsp_transport','tcp',
           rtmp_url]

# using subprocess and pipe to fetch frame data
p = subprocess.Popen(command, stdin=subprocess.PIPE)

frame_count = 0

frame = None

def output_callback(data):

    global frame, frame_count, _frame, width, height, p

    if frame is not None:

        _frame = cv2.cvtColor(cv2.resize(frame, (800,450)), cv2.COLOR_RGB2BGR)
        p.stdin.write(_frame.tobytes())
        
    frame = np.frombuffer(data.data, dtype=np.uint8).reshape(data.height, data.width, -1)
    frame_count += 1

def streamer():
    rospy.loginfo("Streamer initiated...")
    rospy.init_node('streamer',anonymous=True)
    rospy.Subscriber('/uav_2/video_stream', SensorImage, output_callback)
    rospy.spin()

if __name__ == '__main__':

    try:
        streamer()
    except rospy.ROSInterruptException:
        pass
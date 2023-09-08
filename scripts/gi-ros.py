#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from datetime import datetime
import gi
# https://gist.github.com/robobe/525c1155f47e4422127c715332d29bb5
# Add GStreamer imports
gi.require_version('Gst', '1.0')
from gi.repository import Gst

# Initialize GStreamer
Gst.init(None)

# Set the RTSP server address and port
rtsp_server_address = "127.0.0.1"
rtsp_server_port = "8554"
rtsp_stream_name = "my_stream"  # Choose a name for your stream
fps= 16


# Create a GStreamer pipeline to stream the image
pipeline = ('appsrc name=source  ! videoconvert' + \
    ' ! video/x-raw,format=I420' + \
    ' ! x264enc bframes=0 tune=zerolatency speed-preset=ultrafast bitrate=600 key-int-max=' + str(fps * 2) + \
    ' ! video/x-h264,profile=baseline' + \
    ' ! rtspclientsink location=rtsp://localhost:8554/uav2_fpv'
        )

# Create a GStreamer pipeline and set it to playing state
gst_pipeline = Gst.parse_launch(pipeline)
gst_pipeline.set_state(Gst.State.PLAYING)
# Initialize the CvBridge
bridge = CvBridge()

# Define a callback function to handle incoming images
def image_callback(msg):
    global gst_pipeline,bridge,rtsp_server_address,rtsp_server_port,rtsp_stream_name
    try:
        # Convert the ROS Image message to a OpenCV image
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        #gst_pipeline.set_state(Gst.State.PLAYING)
        print("%s frame written to the server" % datetime.now())
        # Get the appsrc element and feed it with the OpenCV image data
        appsrc = gst_pipeline.get_by_name("source")
        appsrc.emit("push-buffer", Gst.Buffer.new_wrapped(cv_image.tobytes()))

        # Publish the RTSP stream using the specified server and port
        rtsp_url = f"rtsp://{rtsp_server_address}:{rtsp_server_port}/{rtsp_stream_name}"
        rospy.loginfo(f"Streaming image to {rtsp_url}")

        # Wait for a Ctrl+C to stop the stream
        #rospy.spin()

        # Stop the GStreamer pipeline
        #gst_pipeline.set_state(Gst.State.NULL)

    except Exception as e:
        rospy.logerr(f"Error: {str(e)}")

# Subscribe to the ROS topic that provides the images

if __name__ == '__main__':
    # Initialize the ROS node
    rospy.init_node("image_to_rtsp_stream")
    image_topic = "/uav_2/video_stream"
    rospy.Subscriber(image_topic, Image, image_callback)

    rospy.spin()
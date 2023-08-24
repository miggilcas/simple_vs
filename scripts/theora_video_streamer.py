import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from theora_image_transport.msg import Packet

def image_callback(msg):
    # Convert ROS image message to OpenCV image
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    # Encode the image using Theora codec
    retval, buffer = cv2.imencode('.ogg', cv_image)

    if retval:
        # Create a Packet message and publish it
        packet_msg = Packet()
        packet_msg.header = msg.header
        packet_msg.data = buffer.tostring()
        theora_pub.publish(packet_msg)

if __name__ == '__main__':
    rospy.init_node('theora_video_streamer')

    bridge = CvBridge()

    # Subscribe to the image topic
    image_sub = rospy.Subscriber('/camera/image', Image, image_callback)

    # Create a Theora image transport publisher
    theora_pub = rospy.Publisher('/theora/stream', Packet, queue_size=10)

    rospy.spin()
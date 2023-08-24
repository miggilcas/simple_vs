#!/usr/bin/env python
from __future__ import print_function
   
import rospy
from std_srvs.srv import Trigger, TriggerResponse
   
def trigger_response(req):
    print("se recibio triger")
    return TriggerResponse(
    	success=True,
    	message= "ok"
    )
def add_two_ints_server():
    rospy.init_node('catec')
    s = rospy.Service('uav_1/threat_confirmation', Trigger, trigger_response)
    print("Ready service")
    rospy.spin()

if __name__ == "__main__":
    add_two_ints_server()

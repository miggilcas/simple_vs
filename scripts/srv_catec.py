#!/usr/bin/env python
from __future__ import print_function
#rostopic  pub -r 10 /uav_1/threat std_msgs/Bool "data: true"
import rospy
from std_srvs.srv import Trigger, TriggerResponse, SetBool,SetBoolResponse
   
def trigger_response(req):
    print("se recibio triger")
    return TriggerResponse(
    	success=True,
    	message= "ok"
    )
def bool_response(req):
    print("se recibio bool")
    return SetBoolResponse(
    	success=req.data,
    	message= "ok"
    )

def add_two_ints_server():
    rospy.init_node('catec')
    s = rospy.Service('uav_100/threat_defuse', Trigger, trigger_response)
    s1 = rospy.Service('uav_100/threat_confirmation', Trigger, trigger_response)
    print("Ready service")
    rospy.spin()




if __name__ == "__main__":
    add_two_ints_server()

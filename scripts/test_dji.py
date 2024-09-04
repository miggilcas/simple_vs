#!/usr/bin/env python3
from time import sleep, time
import rospy
from sensor_msgs.msg import Image
import os
from beginner_tutorials.srv import AddTwoInts, AddTwoIntsResponse
from aerialcore_common.srv import ConfigMission, ConfigMissionResponse,finishGetFiles,finishGetFilesResponse,finishMission,finishMissionResponse
from std_srvs.srv import SetBool, SetBoolResponse
from dji_osdk_ros.srv import DownloadMedia, DownloadMediaResponse

topic = "/uav_2/dji_osdk_ros/fpv_camera_images"
rospy.loginfo("I will suscribe to the topic %s", topic)


def SrvCommandMission(req):
    print("called service SrvCommandMission")
    response = SetBoolResponse()
    return response


def SrvLoadMission(req):
    print("called service SrvloadMission")
    response = ConfigMissionResponse()
    
    return response


def SrvDownloadFiles(req):
    print("called service srvDownloadFiles")
    response = DownloadMediaResponse()
    return response


def SrvCallFinishMissionGCS(req):
    print("called service SrvCallFinishMissionGCS")
    CallSrvCallFinishMissionGCS()
    response = SetBoolResponse()
    
    return response

def SrvCallDownloadFilesGCS(req):    
    print("called service SrvCallDownloadFilesGCS")
    response = SetBoolResponse()
    CallSrvFinishDownload()
    
    return response


def CallSrvFinishDownload():
    print("called service callSrvFinishDownload")
    rospy.wait_for_service('/GCS/FinishDownload')
    try:
        myServiceCall = rospy.ServiceProxy('/GCS/FinishDownload', finishGetFiles)
        resp1 = myServiceCall()
        return resp1
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


def CallSrvCallFinishMissionGCS():
    print("called service callSrvCallFinishMissionGCS")
    rospy.wait_for_service('/GCS/FinishMission')
    try:
        myServiceCall = rospy.ServiceProxy('/GCS/FinishMission', finishMission)
        resp1 = myServiceCall()
        return resp1
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


if __name__ == '__main__':
    rospy.init_node("Test DJI services ", anonymous=True)
    s1 = rospy.Service('/dji_control/configure_mission', SrvLoadMission, ConfigMission)
    s2 = rospy.Service('/dji_control/start_mission', SrvCommandMission, SetBool)
    s3 = rospy.Service('/camera_download_files', SrvDownloadFiles, DownloadMedia)
    s4 = rospy.Service('/finish_mission', SrvCallFinishMissionGCS, SetBool)
    s4 = rospy.Service('/finish_download', SrvCallDownloadFilesGCS, SetBool)
    rospy.spin()

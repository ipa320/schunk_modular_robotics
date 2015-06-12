#!/usr/bin/python

import roslib
roslib.load_manifest('cob_script_server')
import rospy
import os

import dynamic_reconfigure.client
from std_srvs.srv import Trigger, TriggerResponse

from simple_script_server import *
sss = simple_script_server()

def right_cb(req):
    rospy.loginfo("recovering gripper_right")
    os.system("rosnode kill /gripper_right/gripper_right_node")
    return TriggerResponse()

def left_cb(req):
    rospy.loginfo("recovering gripper_left")
    os.system("rosnode kill /gripper_left/gripper_left_node")
    return TriggerResponse()

if __name__ == "__main__":
    rospy.init_node("sdhx_recover")
    rospy.Service('/gripper_right/driver/recover', Trigger, right_cb)
    rospy.Service('/gripper_left/driver/recover', Trigger, left_cb)
    rospy.spin()

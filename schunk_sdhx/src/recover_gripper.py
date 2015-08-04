#!/usr/bin/python

import os

import rospy
from std_srvs.srv import Trigger, TriggerResponse

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

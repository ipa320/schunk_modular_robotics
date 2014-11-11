#!/usr/bin/env python
#################################################################
##\file
#
# \note
# Copyright (c) 2014 \n
# Fraunhofer Institute for Manufacturing Engineering
# and Automation (IPA) \n\n
#
#################################################################
#
# \note
# Project name: schunk_modular_robotics
# \note
# ROS stack name: schunk_finger
# \note
# ROS package name: schunk_finger
#
# \author
# Thiago de Freitas, email:tdf@ipa.fhg.de
#
# \date Date of creation: November 2014
#
# \brief
# Simple driver for the Schunk finger
#
#################################################################
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# - Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer. \n
# - Redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution. \n
# - Neither the name of the Fraunhofer Institute for Manufacturing
# Engineering and Automation (IPA) nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission. \n
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License LGPL as
# published by the Free Software Foundation, either version 3 of the
# License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU Lesser General Public License LGPL for more details.
#
# You should have received a copy of the GNU Lesser General Public
# License LGPL along with this program.
# If not, see <http://www.gnu.org/licenses/>.
#
#################################################################

import roslib; roslib.load_manifest('schunk_finger')

from control_msgs.msg import *
from sensor_msgs.msg import *

FJT_ACTION_NAME="finger_controller/follow_joint_trajectory"

import serial
import time
import rospy
import actionlib

Terminator="\r\n"
commands = {"position": "p", "stop": "s", "move" : "m"}

class Finger():

  def __init__(self, port):
  
    self._ser = serial.Serial(port, timeout=1.2)
    
    self._as = actionlib.SimpleActionServer(FJT_ACTION_NAME, FollowJointTrajectoryAction, self.execute_cb, False)
    self._as.start()
    self._pub_joint_states = rospy.Publisher('joint_states', JointState,queue_size=5)
    self._pub_controller_state = rospy.Publisher('finger_controller/state', JointTrajectoryControllerState,queue_size=5)
    # joint state
    self._joint_states = JointState()
    self._joint_states.name = ["finger_distal", "finger_proximal"]
    self._joint_states.velocity = [0.0,0.0]
    self._joint_states.effort = [0.0,0.0]
    # controller state
    self._controller_state = JointTrajectoryControllerState()

  def get_pos(self):  
    status,pos = self.execute_command("position")
    
    return status, pos
  
  def move(self,pos_0, pos_1):
    complement = " "+(str)(pos_0)+","+(str)(pos_1)+Terminator
    
    status, reply = self.execute_command("move", complement)

    return status, reply


    
  def close_port(self):  
    self._ser.close()
  
  def execute_cb(self, goal):
    
    position_distal = goal.trajectory.points[-1].positions[0]
    position_proximal = goal.trajectory.points[-1].positions[1]
    
    status, reply = self.move(position_distal, position_proximal)

    if(status==True):
      self._as.set_succeeded()
    else:
      self._as.set_aborted()
    
  def publish_joint_states(self):
    status, pos = self.get_pos()
    if status == True:
      try:
        actual_pos_distal = pos[0]
        actual_pos_proximal = pos[1]
        self._joint_states.position = [(int)(actual_pos_distal), (int)(actual_pos_proximal)]
        self._joint_states.header.stamp = rospy.Time.now()
        self._pub_joint_states.publish(self._joint_states)
      except ValueError:
        pass
        
  def publish_controller_state(self):
    self._controller_state.header.stamp = rospy.Time.now()
    self._controller_state.joint_names = self._joint_states.name
    self._controller_state.actual.positions = self._joint_states.position
    self._controller_state.actual.velocities = self._joint_states.velocity 
    self._pub_controller_state.publish(self._controller_state)

  def execute_command(self, command, complement=Terminator, timeout=2):
  
    t_out = time.time() + timeout
    
    try:
    
      self._ser.write(commands[command]+complement)
      line_to_parse = self._ser.readline()
      if(command == "position"):
      
        while True:
          if (time.time() > t_out):
            return False,None
          elif("P=" not in line_to_parse[0:2]):
            self._ser.write(commands[command]+complement)
            line_to_parse = self._ser.readline()
          else:
            break
        
        line_to_parse = line_to_parse.replace('P=','')
        line_to_parse = line_to_parse.strip()
        line_to_parse = line_to_parse.split(',')
        result = line_to_parse
        
        if(len(result) !=2):
          status = False
        else:
          status=True
              
      elif(command == "move"):
      
        while True:
          if (time.time() > t_out):
            return False,None
          elif("@Mov" not in line_to_parse[0:5]):
            self._ser.write(commands[command]+complement)
            line_to_parse = self._ser.readline()
          else:
            break
       
        result = line_to_parse
      
        status = True
      
      self._ser.flush()
      
      return status, result
      
    except serial.serialutil.SerialException:
      pass
    
    
    
    
if __name__ == '__main__':
  rospy.init_node('schunk_finger')
  fing = Finger("/dev/ttyACM0")
  r = rospy.Rate(30) # 10hz
  while not rospy.is_shutdown():
    fing.publish_joint_states()
    fing.publish_controller_state()
    r.sleep()
  fing.close_port()


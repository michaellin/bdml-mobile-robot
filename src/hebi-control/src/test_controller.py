#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import math
import hebi
from time import sleep
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import Joy



controllerCmds = np.zeros(4)

def get_joy_cmd(data):
  global controllerCmds
  maxVel = 2.1 # rad/s
  minJoyPosition = .12 # unit vector
  controllerCmds = np.array(data.axes[0:4])
  i = 2
  #for i in range(4):
  # use // to divide instead of all these repetive lines of code, (MATH)
  if controllerCmds[i] > minJoyPosition:
    if controllerCmds[i] < .2:
      controllerCmds[i] = controllerCmds[i] * maxVel*.2
    elif controllerCmds[i] < .4:
      controllerCmds[i] = controllerCmds[i] * maxVel*.4
    elif controllerCmds[i] < .6:
      controllerCmds[i] = controllerCmds[i] * maxVel* .6
    elif controllerCmds[i] < .8:
      controllerCmds[i] = controllerCmds[i] * maxVel *.8
    else:
      controllerCmds[i] = controllerCmds[i] * maxVel


  return


lookup = hebi.Lookup()
sleep(2) # delay allows for discovery
print('Modules found on network:')
print(lookup.entrylist._size)
for entry in lookup.entrylist:
  print(f'{entry.family} | {entry.name}')

sleep(2.0)
family_name = "base"
module_name = "front_left leg"
group = lookup.get_group_from_names([family_name], ["front_left leg"])

group_command = hebi.GroupCommand(group.size)
group_feedback = hebi.GroupFeedback(group.size)



rospy.init_node('controller_listener', disable_signals=True)

# ROS publishers and subscribers
joy_sub = rospy.Subscriber('joy', Joy, get_joy_cmd)

# set ROS loop rate to 100. Same as default feedback rate from HEBI.
rate = rospy.Rate(100)

while True:

  group.get_next_feedback(reuse_fbk=group_feedback)
  group_command.velocity = controllerCmds[2]   # 
  group.send_command(group_command)
  print(controllerCmds[2])
  rate.sleep()


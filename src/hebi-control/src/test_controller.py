#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import math
#import hebi
from time import sleep
from std_msgs.msg import Float64
from sensor_msgs.msg import Joy
import rospy


controllerCmds = np.zeros(4)

def get_joy_cmd(data):
  # storing the left and right joystick axes values
  print("hi")
  controllerCmds = np.array(data.axes[0:3])
  return


rospy.init_node('controller_listener', disable_signals=True)

# ROS publishers and subscribers
joy_sub = rospy.Subscriber('joy', Joy, get_joy_cmd)

# set ROS loop rate to 100. Same as default feedback rate from HEBI.
rate = rospy.Rate(100)

while True:
  print(controllerCmds)
  rate.sleep()


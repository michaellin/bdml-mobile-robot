#!/usr/bin/env python3
# -*- coding: utf-8 -*-
""" hebi_control.py
    ROS node for interfacing HEBI actuators. Example code.
    author: Michael Andres Lin (michaelv03@gmail.com)
    date: 12/08/2020
"""

import numpy as np
import math
import hebi
from time import sleep
from std_msgs.msg import Float64
from sensor_msgs.msg import Joy
import rospy

class HebiRobot(object):
  """class HebiRobot: implements simple ROS node interface for HEBI actuators.

  If the class has public attributes, they may be documented here
  in an ``Attributes`` section and follow the same formatting as a
  function's ``Args`` section. Alternatively, attributes may be documented
  inline with the attribute's declaration (see __init__ method below).

  Properties created with the ``@property`` decorator should be documented
  in the property's getter method.

  Attributes:
      family (str): name of the family of HEBI actuators
      names (str): list of names of individual actuators in a chain

  """

  def __init__(self, family='X8-9', names=['X-80749'], noHw = False):
    self.joint_position = 0.0   # variable for sensor position reading
    self.vel_des = 0.0    # variable for velocity control

    # store hebi actuator params
    self.family = family
    self.actuator_names = names

    self.noHw = noHw

    if (self.noHw):
      # automatic lookup of actuators connected to the network
      lookup = hebi.Lookup()
      # Give the Lookup process 2 seconds to discover modules
      sleep(2)
      print('Modules found on network:')
      for entry in lookup.entrylist:
        print("{0} | {1}".format(entry.family, entry.name))

      # get control group by family name and module name
      self.group = lookup.get_group_from_names([self.family], self.actuator_names)

      # create a group command object used to send commands to actuators
      self.group_command = hebi.GroupCommand(self.group.size)

      # set up the feedback callback function
      self.group.add_feedback_handler(self.get_actuator_feedback)

    # start ROS node
    rospy.init_node('hebi_control', disable_signals=True)

    # ROS publishers and subscribers
    self.vel_des_sub = rospy.Subscriber('vel_des', Float64, self.get_vel_des)
    self.joy_sub = rospy.Subscriber('joy', Joy, self.get_joy_cmd)
    self.feedback_pub = rospy.Publisher('motor_pos', Float64, queue_size=3)

    # set ROS loop rate to 100. Same as default feedback rate from HEBI.
    self.rate = rospy.Rate(100)


  def run_loop(self):

    while True:
      # send feedback values
      self.feedback_pub.publish(self.joint_position)

      # can do something with self.controllerCmds

      if (self.noHw):
        # command desired velocities
        self.group_command.velocity = self.vel_des
        self.group.send_command(self.group_command)

      self.rate.sleep()

  def get_vel_des(self, data):
    """ Fetches motor command data as a callback

    Args:
      data: std_msgs.Float64 with desired motor velocity

    Returns:
      Nothing
    """
    self.vel_des = data.data
    return



  def get_actuator_feedback(self, feedback):
    """ Fetches HEBI actuator sensor data as a callback

    Args:
      feedback: hebi.GroupFeedback instance contains joint position and velocity

    Returns:
      Nothing
    """
    self.joint_position = feedback.position
    return

  def get_joy_cmd(self, data):
    """ Fetches data from xbox 360 controller

    Args:
      data: of type Joy (Joy.buttons and Joy.axes)

    Returns:
      Nothing
    """
    # storing the left and right joystick axes values
    print("hi")
    self.controllerCmds = data.axes[0:3]
    return
    


if __name__ == '__main__':
  hebi = HebiRobot(noHw=True)
  try:
    hebi.run_loop()
  except KeyboardInterrupt:
    print("Terminating node")

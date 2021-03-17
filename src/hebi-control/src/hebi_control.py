#!/usr/bin/env python3
# -*- coding: utf-8 -*-
""" hebi_control.py
    ROS node for interfacing HEBI actuators. Example code.
    author: Michael Lin (michaelv03@gmail.com)
            Emilio Reyes (ereyes35@stanford.edu)
            Gaby Uribe
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
  max_vel = 2.1 # rad/s
  min_joy_position = .12 # unit vector
  step_interval = .2 # discretization bins 

  def __init__(self, family=['base','base'], names=['front_left leg','front_right leg'], no_hw = False):
    self.joint_position = 0.0   # variable for sensor position reading
    self.vel_des = 0.0    # variable for velocity control

    # user inputs linear vel (vx, vy) and angular vel (w)
    self.user_cmd = np.zeros(3)
    self.joint_vel_cmd = np.zeros(len(names))

    # store hebi actuator params
    self.family = family
    self.actuator_names = names

    self.no_hw = no_hw

    # if we have motors connected then initialize them
    if (self.no_hw):
      ### HEBI initializations ###

      # automatic lookup of actuators connected to the network
      lookup = hebi.Lookup()
      # Give the Lookup process 2 seconds to discover modules
      sleep(2)
      print('Modules found on network:')
      for entry in lookup.entrylist:
        print("{0} | {1}".format(entry.family, entry.name))

      # get control group by family name and module name
      self.group = lookup.get_group_from_names(self.family, self.actuator_names)

      # create a group command object used to send commands to actuators
      self.group_command = hebi.GroupCommand(self.group.size)

      # set up the feedback callback function
      self.group.add_feedback_handler(self.get_actuator_feedback)

      ### end: HEBI initializations ###

    ### ROS initializations ###

    # start ROS node
    rospy.init_node('hebi_control', disable_signals=True)

    # ROS publishers and subscribers
    self.joy_sub = rospy.Subscriber('joy', Joy, self.get_joy_cmd)
    self.feedback_pub = rospy.Publisher('motor_pos', Float64, queue_size=3)

    # set ROS loop rate to 100. Same as default feedback rate from HEBI.
    self.rate = rospy.Rate(100)

    ### end: ROS initializations ###


  def run_loop(self):

    while True:
      # publish feedback values from hebi motors to other ROS nodes
      self.feedback_pub.publish(self.joint_position)

      if (self.no_hw):
        # first check that joint_vel_cmd is of correct size
        assert (len(self.joint_vel_cmd) != self.group.size), 
                f"joint_vel_cmd should be of size {self.group.size}"
        # command desired velocities
        self.group_command.velocity = self.joint_vel_cmd 
        self.group.send_command(self.group_command)

      self.rate.sleep()

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
    self.user_cmd = np.zeros(2)

    for i in range(0,2):      
      if (data.axes[i] > self.min_joy_position):
        self.user_cmd[i] = (data.axes[i] // self.step_interval) * self.max_vel * self.step_interval

    # TODO choose the joy stick direction mappings
    # TODO here add some controller input post processing then save to self.user_cmd
    # e.g. multiply with the inverse jacobian to get the joint velocities
    # TODO store these joint velocities in self.joint_vel_cmd
    # TODO saturate self.joint_vel_cmd
    self.joint_vel_cmd = self.user_cmd
    return
    


if __name__ == '__main__':
  hebi = HebiRobot(no_hw=True)

  try:
    hebi.run_loop()
  except KeyboardInterrupt:
    print("Terminating node")

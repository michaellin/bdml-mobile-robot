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
from std_msgs.msg import Float64, Float64MultiArray
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
  #max_joint_vel = 1.5 # rad/s
  #max_vel = np.array([.3, .3, .2, 0.1]) # [m/s, m/s, rad/s]
  max_joint_vel = 3.5 # rad/s
  max_vel = np.array([.25, .25, .25, 0.5]) # [m/s, m/s, rad/s]
  min_joy_position = .12 # unit vector
  step_interval = .2 # discretization bins 
  lx = .29068 # m
  ly = .2667 # m
  lxy = 1.0 # m
  r = .127 # m
  invR = 1.0/r
  rot_scaler = 1.
  back_scalar = 2.0
  #A = np.array([ [1, -1, -(lxy)*rot_scaler] , 
  #               [1,  1,  (lxy)*rot_scaler  ] ,
  #               [1, -1,  (lxy)*rot_scaler ] , 
  #               [1,  1, -(lxy)*rot_scaler ]  ] )
  A = np.array([ [1, -1, -(lxy), -1], 
                 [1,  1,  (lxy), -1],
                 [back_scalar, -back_scalar,  back_scalar*lxy, 1], 
                 [back_scalar,  back_scalar, -back_scalar*lxy, 1]])

  def __init__(self, family=['base','base', 'base', 'base'], names=['front_left_leg','front_right_leg', 'back_left_leg', 'back_right_leg'], hw = False):
    self.joint_position = 0.0   # variable for sensor position reading
    self.vel_des = 0.0    # variable for velocity control

    # user inputs linear vel (vx, vy) and angular vel (w)
    self.user_cmd = np.zeros(4)
    self.cmd_data_bias = np.zeros(4)
    self.joint_vel_cmd = np.zeros(len(names))

    # store hebi actuator params
    self.family = family
    self.actuator_names = names

    self.hw = hw

    # if we have motors connected then initialize them
    if (self.hw):
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
      assert hasattr(self.group, 'size'), "Group does not have attribute 'size' maybe you are not connected to actuators"
      self.group_command = hebi.GroupCommand(self.group.size)

      # set up the feedback callback function
      self.group.add_feedback_handler(self.get_actuator_feedback)

      ### end: HEBI initializations ###

    ### ROS initializations ###

    # start ROS node
    rospy.init_node('hebi_control', disable_signals=True)

    # ROS publishers and subscribers
    self.joy_sub = rospy.Subscriber('joy', Joy, self.get_joy_cmd)
    self.feedback_pub = rospy.Publisher('motor_vel', Float64MultiArray, queue_size=9)
    self.cmd_pub = rospy.Publisher('motor_vel_cmd', Float64MultiArray, queue_size=9)

    # set ROS loop rate to 100. Same as default feedback rate from HEBI.
    self.rate = rospy.Rate(100)

    ### end: ROS initializations ###


  def run_loop(self):

    while True:
      if (self.hw):
        # publish feedback values from hebi motors to other ROS nodes
        self.feedback_pub.publish(Float64MultiArray(data=self.joint_velocity))

        # first check that joint_vel_cmd is of correct size
        assert (len(self.joint_vel_cmd) == self.group.size), \
                f"joint_vel_cmd is of size {len(self.joint_vel_cmd)} but should be of size {self.group.size}"
        cmd_vel = self.joint_vel_cmd * np.array([1, -1, 1, -1])
        self.cmd_pub.publish(Float64MultiArray(data=cmd_vel))
        # command desired velocities
        #self.group_command.velocity = cmd_vel
        self.group_command.effort = cmd_vel
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
    self.joint_velocity = feedback.velocity
    return

  def get_joy_cmd(self, data):
    """ Fetches data from xbox 360 controller

    Args:
      data: of type Joy (Joy.buttons and Joy.axes)

    Returns:
      Nothing
    """
    # storing the left and right joystick axes values
    # user_cmd = [Vx,Vy,omega]
    self.user_cmd = np.zeros(4)
    #self.user_cmd[3] = 0.0 # internal force
    mapping = [3,2,0,4]
    cmd_data = np.array([data.axes[m] for m in mapping])
    cmd_data[3] = (1.0-(cmd_data[3]+1.0)/2.0)

    if (data.buttons[5]):
      # grab axes bias
      self.cmd_data_bias = cmd_data
      print(f"bias is {self.cmd_data_bias}")
    # TODO?: change max_vel to array catersian max vel 
    for i in range(len(mapping)):      
      cmd = cmd_data[i] - self.cmd_data_bias[i]
      if (abs(cmd) > self.min_joy_position) :
        self.user_cmd[i] = (cmd // self.step_interval) * \
                            self.max_vel[i] * self.step_interval 
        
    B = self.A * self.invR
    C = B.dot(self.user_cmd)
    #print(f"joint vel {self.user_cmd} output {C}")
    self.joint_vel_cmd = np.clip(C,-self.max_joint_vel,self.max_joint_vel)
    # C = [top_left, top_right, bottom_left, bottom_right]

    # TODO choose the joy stick direction mappings
    # TODO here add some controller input post processing then save to self.user_cmd
    # e.g. multiply with the inverse jacobian to get the joint velocities
    # TODO store these joint velocities in self.joint_vel_cmd
    # TODO saturate self.joint_vel_cmd
    
    return 
    


if __name__ == '__main__':
  hebi = HebiRobot(hw=True)

  try:
    hebi.run_loop()
  except KeyboardInterrupt:
    print("Terminating node")

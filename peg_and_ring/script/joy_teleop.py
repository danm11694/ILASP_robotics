#!/usr/bin/env python

#
# /blueooth_teleop/joy
# sensor_msgs/Joy
#
# std_msgs/Header header
#   uint32 seq
#   time stamp
#   string frame_id
# float32[] axes
# int32[] buttons
#
# axes: [-0.0, -0.0, -0.0, -0.0, -0.0, -0.0]
# buttons: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
#

import dvrk
import rospy
import sys
import time
import os

import dvrk
import math
import numpy
import PyKDL

from sensor_msgs.msg import Joy

class joy_control(object):

    def __init__(self):

        rospy.loginfo("[STARTING JOY TELEOP]")

        rate = rospy.Rate(5)

        rospy.Subscriber("/joy", Joy, self.joy_callback)

        self.__name = rospy.get_name()
        self.arm_name = rospy.get_param(self.__name + '/arm_name', 'arm_name')
        rospy.loginfo("[YOU ARE CONTROLLING]  " + self.arm_name )
        self.robot_ = dvrk.psm(self.arm_name)

        self.left_analog = [0,0]
        self.right_analog = [0,0]
        self.pad = [0,0]
                
        self.sq = self.tri = self.x = self.circ = self.L1 = self.R1 = self.L2 = self.R2 = self.L3 = self.R3 = self.select = self.start = 0

        self.__translation_displ = [float(0)]*3
        self.__displ = 0.005
        self.__rot_displ = 0.021

    def update(self):
        current_joint_position = numpy.copy(self.robot_.get_current_joint_position())
        
        cmd = 0.0
        indx = 0
        # if abs(self.right_analog[0]) > 0.0: # arrowleft neg jnt1
        if self.sq:
            cmd = -self.__displ 
            indx = 0

        # if abs(self.right_analog[1]) > 0.0: # arrowright pos jnt1
        if self.circ:
            cmd = self.__displ
            indx = 0

        elif self.x: # arrowdown neg jnt2
            cmd = -self.__displ
            indx = 1

        elif self.tri: # arrowup pos jnt2
            cmd = self.__displ
            indx = 1

        elif self.R1: # 'a' neg jnt3
            cmd = -self.__displ / 3.0
            indx = 2

        elif self.L1: # 'z' pos jnt3
            cmd = self.__displ / 3.0
            indx = 2

        elif self.R3: # 's' pos jnt4
            cmd = self.__rot_displ
            indx = 3

        elif self.L3: # 'x' neg jnt4
            cmd = -self.__rot_displ
            indx = 3

        if (('PSM1' in self.arm_name) or ('PSM2' in self.arm_name)):
            if self.L2:
                self.robot_.open_jaw()

            elif self.R2:
                self.robot_.close_jaw()
        

        self.robot_.dmove_joint_one(cmd, indx, interpolate=True, blocking=False)  # with interpolation


    # callback function maps button data observed from joystick topic         [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    def joy_callback(self, data):
        self.left_analog[0] = data.axes[0]
        self.left_analog[1] = data.axes[1]
        self.right_analog[0] = data.axes[2]
        self.right_analog[1] = data.axes[3]
        self.pad[0] = data.axes[4]
        self.pad[1] = data.axes[5]

        self.sq = data.buttons[0]
        self.tri = data.buttons[1]
        self.x = data.buttons[2]
        self.circ = data.buttons[3]
        self.L1 = data.buttons[4]
        self.R1 = data.buttons[5]
        self.L2 = data.buttons[6]
        self.R2 = data.buttons[7]
        self.L3 = data.buttons[8]
        self.R3 = data.buttons[9]
        self.select = data.buttons[10]
        self.start = data.buttons[11]        

if __name__ == "__main__":

    try:
        rospy.init_node("joy_teleop", anonymous=False)
        joy_teleop = joy_control()


        while not rospy.is_shutdown():
            joy_teleop.update()


    except rospy.ROSInterruptException:
        rospy.loginfo("joy_teleop node terminated.")

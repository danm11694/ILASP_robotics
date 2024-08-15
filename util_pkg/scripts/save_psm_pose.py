#!/usr/bin/env python  

from __future__ import print_function

import roslib
import rospy

import geometry_msgs.msg
import std_msgs.msg
import tf2_ros
import tf2_geometry_msgs
import cv2
import sys

class convertClicked:
    def __init__(self, *args):
        self.__name = rospy.get_name()
        self.psm_pose = '/dvrk/PSM1/position_cartesian_current'
        # self._converted_topic = '/camera_point'
        
        self.point_click_sub = rospy.Subscriber(self.psm_pose, geometry_msgs.msg.PoseStamped, self.convert_function) 
        # self.point_click_pub = rospy.Publisher(self._converted_topic, geometry_msgs.msg.PoseStamped, queue_size=1)


        self.point_click_sub = rospy.Subscriber('/save', std_msgs.msg.Bool, self.saveCB)
        self.point_xyz = geometry_msgs.msg.Point()
        self.save = False

    def convert_function(self, msg):
        self.point_xyz.x = msg.pose.position.x
        self.point_xyz.y = msg.pose.position.y
        self.point_xyz.z = msg.pose.position.z

        # print(self.point_xyz)

    def saveCB(self, msg):
        self.save = True


if __name__=='__main__':
    try:
        rospy.init_node('convertClicked_py')
        
        wc = convertClicked()
        f = open("psm1_pose_3_1.txt", "a")
        
        rate = 1 # hz  # 3   Hz
        ros_rate = rospy.Rate(rate)
        while not rospy.is_shutdown():
            if wc.save:
                print(wc.point_xyz)
                print('-----')
                f.write(str(wc.point_xyz.x)+ ' ' + str(wc.point_xyz.y)+ ' ' + str(wc.point_xyz.z) + '\n')
                wc.save = False            
            ros_rate.sleep()
        f.close()

    except rospy.ROSInterruptException:
        print ('[convertClicked] Interrupt.',file=sys.stderr)
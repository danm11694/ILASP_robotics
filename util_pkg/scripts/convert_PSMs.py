#!/usr/bin/env python

from __future__ import print_function

import roslib
import rospy

import geometry_msgs.msg

from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseStamped

import tf
import tf2_ros
import tf2_geometry_msgs
import tf.transformations as tr

import sys
import math
import numpy


class convertPSMs:
    def __init__(self, *args):
        self.__name = rospy.get_name()
        self.psm1_pose_pub = rospy.Publisher('/dvrk/PSM1/position_cartesian_current', geometry_msgs.msg.PoseStamped, queue_size=1)
        self.psm2_pose_pub = rospy.Publisher('/dvrk/PSM2/position_cartesian_current', geometry_msgs.msg.PoseStamped, queue_size=1)
        self.ecm_pose_pub = rospy.Publisher('/dvrk/ECM/position_cartesian_current', geometry_msgs.msg.PoseStamped, queue_size=1)

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

    def publish_pose(self):
        try:
            
            transform_psm1 = self.tfBuffer.lookup_transform('PSM1_base', #target_Frame
                                       'PSM1', #source frame
                                       rospy.Time(0), #get the tf at first available time
                                       rospy.Duration(1.0)) #wait for 1 second
            
            transform_psm2 = self.tfBuffer.lookup_transform('PSM2_base', #target_Frame
                                       'PSM2', #source frame
                                       rospy.Time(0), #get the tf at first available time
                                       rospy.Duration(1.0)) #wait for 1 second


            transform_ecm = self.tfBuffer.lookup_transform('ECM_base', #target_Frame
                                       'ECM', #source frame
                                       rospy.Time(0), #get the tf at first available time
                                       rospy.Duration(1.0)) #wait for 1 second

            pose_psm1 = geometry_msgs.msg.PoseStamped()
            pose_psm1.header.frame_id = 'PSM1_base_new'
            pose_psm1.header.stamp = rospy.get_rostime()
            pose_psm1.pose.position.x = transform_psm1.transform.translation.x
            pose_psm1.pose.position.y = transform_psm1.transform.translation.y
            pose_psm1.pose.position.z = transform_psm1.transform.translation.z
            pose_psm1.pose.orientation.x = transform_psm1.transform.rotation.x
            pose_psm1.pose.orientation.y = transform_psm1.transform.rotation.y
            pose_psm1.pose.orientation.z = transform_psm1.transform.rotation.z
            pose_psm1.pose.orientation.w = transform_psm1.transform.rotation.w


            pose_psm2 = geometry_msgs.msg.PoseStamped()
            pose_psm2.header.frame_id = 'PSM2_base_new'
            pose_psm2.header.stamp = rospy.get_rostime()
            pose_psm2.pose.position.x = transform_psm2.transform.translation.x
            pose_psm2.pose.position.y = transform_psm2.transform.translation.y
            pose_psm2.pose.position.z = transform_psm2.transform.translation.z
            pose_psm2.pose.orientation.x = transform_psm2.transform.rotation.x
            pose_psm2.pose.orientation.y = transform_psm2.transform.rotation.y
            pose_psm2.pose.orientation.z = transform_psm2.transform.rotation.z
            pose_psm2.pose.orientation.w = transform_psm2.transform.rotation.w
            

            pose_ecm = geometry_msgs.msg.PoseStamped()
            pose_ecm.header.frame_id = 'ECM_base_new'
            pose_ecm.header.stamp = rospy.get_rostime()
            pose_ecm.pose.position.x = transform_ecm.transform.translation.x
            pose_ecm.pose.position.y = transform_ecm.transform.translation.y
            pose_ecm.pose.position.z = transform_ecm.transform.translation.z
            pose_ecm.pose.orientation.x = transform_ecm.transform.rotation.x
            pose_ecm.pose.orientation.y = transform_ecm.transform.rotation.y
            pose_ecm.pose.orientation.z = transform_ecm.transform.rotation.z
            pose_ecm.pose.orientation.w = transform_ecm.transform.rotation.w
            

            self.psm1_pose_pub.publish(pose_psm1)
            self.psm2_pose_pub.publish(pose_psm2)
            self.ecm_pose_pub.publish(pose_ecm)
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print('error')


if __name__=='__main__':
    try:
        rospy.init_node('convertPSMs_py')

        wc = convertPSMs()

        rate = 200 # Hz
        ros_rate = rospy.Rate(rate)
			  
        while not rospy.is_shutdown():
            wc.publish_pose()
            ros_rate.sleep()

    except rospy.ROSInterruptException:
        print ('[arucoGlobamFrame] Interrupt.',file=sys.stderr)

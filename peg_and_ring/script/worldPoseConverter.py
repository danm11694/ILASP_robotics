#!/usr/bin/env python  

from __future__ import print_function

import roslib
import rospy

import geometry_msgs.msg
import tf2_ros
import tf2_geometry_msgs

import sys


# SEND HERE THE POSE STAMPED POINTS IN CAMERA_LINK COORDINATE TO CONVERT THEM TO PANDA_LINK0 FRAME (BASE_LINK)

class worldPoseConverter:
    def __init__(self, *args):
        self.__name = rospy.get_name()
        self._grasping_topic = rospy.get_param(self.__name + '/grasping_topic', 'grasping_topic')
        self._converted_topic = rospy.get_param(self.__name + '/converted_topic', 'converted_topic')
        
        self.peg_pose_sub = rospy.Subscriber(self._grasping_topic, geometry_msgs.msg.PoseStamped, self.convert_function) # CHANGE NAME HERE
        self.peg_pose_pub = rospy.Publisher(self._converted_topic, geometry_msgs.msg.PoseStamped, queue_size=100)

        # self.peg_pose_sub = rospy.Subscriber('/franka_pub/grasping_point', geometry_msgs.msg.PoseStamped, self.convert_function) # CHANGE NAME HERE
        # self.peg_pose_pub = rospy.Publisher('/peg_converted/pose', geometry_msgs.msg.PoseStamped, queue_size=100)

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

    def convert_function(self, msg):

        try:
            transform = self.tfBuffer.lookup_transform('panda_link0',
                                       'camera_link', #source frame
                                       rospy.Time(0), #get the tf at first available time
                                       rospy.Duration(1.0)) #wait for 1 second
            # print(transform)
            # pose_stamped = geometry_msgs.msg.PoseStamped()
            # pose_stamped.header.frame_id = 'panda_link0'
            # pose_stamped.header.stamp = rospy.get_rostime()                     
            pose_transformed = tf2_geometry_msgs.do_transform_pose(msg, transform)
           
            self.peg_pose_pub.publish(pose_transformed)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print('error')


if __name__=='__main__':
    try:
        rospy.init_node('worldPoseConverter_py')
        
        wc = worldPoseConverter()
        
        while not rospy.is_shutdown():
            rospy.spin()

    except rospy.ROSInterruptException:
        print ('[worldPoseConverter] Interrupt.',file=sys.stderr)
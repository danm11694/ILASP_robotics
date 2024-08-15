#!/usr/bin/env python  

from __future__ import print_function

import roslib
import rospy

import geometry_msgs.msg

from geometry_msgs.msg import TransformStamped

import tf
import tf2_ros
import tf2_geometry_msgs
import tf.transformations as tr

import sys
import math
import numpy


class arucoGlobamFrame:
    def __init__(self, *args):
        self.__name = rospy.get_name()
        self._arucoPose = rospy.get_param('/aruco_single/pose', '/aruco_single/pose')
        self._pointPose = rospy.get_param(self.__name + '/pointPose', 'pointPose')
        
        self.aruco_pose_sub = rospy.Subscriber(self._arucoPose, geometry_msgs.msg.PoseStamped, self.convert_point)
        self.aruco_pose_pub = rospy.Publisher(self._pointPose, geometry_msgs.msg.PoseStamped, queue_size=1)

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        self.radius = 0.047             #   radius of the outer circle

    def convert_point(self,msg):
        
        try:
            transform = self.tfBuffer.lookup_transform('aruco_marker_frame',
                                       'camera_color_optical_frame', #source frame
                                       rospy.Time(0), #get the tf at first available time
                                       rospy.Duration(1.0)) #wait for 1 second
            
            pose_transformed = tf2_geometry_msgs.do_transform_pose(msg, transform)
            rot = tr.quaternion_matrix([transform.transform.rotation.x,transform.transform.rotation.y,transform.transform.rotation.z,transform.transform.rotation.w])
            trasl = tr.translation_matrix( [transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z ] )
            T_matrix = numpy.dot(trasl, rot)
            inverse = numpy.linalg.inv(T_matrix)

            tf_inv = TransformStamped()
            tf_inv.transform.translation.x = inverse[0][3]
            tf_inv.transform.translation.y = inverse[1][3]
            tf_inv.transform.translation.z = inverse[2][3]
            
            q = tf.transformations.quaternion_from_matrix(inverse)
            tf_inv.transform.rotation.x = q[0]
            tf_inv.transform.rotation.y = q[1]
            tf_inv.transform.rotation.z = q[2]
            tf_inv.transform.rotation.w = q[3]

            pose_transformed.pose.position.z = pose_transformed.pose.position.z  + (self.radius * math.cos(147 * math.pi/180))
            pose_transformed.pose.position.x = pose_transformed.pose.position.x  + (self.radius * math.sin(147 * math.pi/180))
            pose_transformed.pose.position.y = pose_transformed.pose.position.y  +   0.005      #       + 0.04                                # OFFSET FOR ARUCO MARKER RECOGNITION
            # print (pose_transformed)

            pose_ = tf2_geometry_msgs.do_transform_pose(pose_transformed, tf_inv)
            pose_.header.frame_id = "camera_color_optical_frame"
            # print (pose_)
            self.aruco_pose_pub.publish(pose_)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print('error')


if __name__=='__main__':
    try:
        rospy.init_node('arucoGlobamFrame_py')
        
        wc = arucoGlobamFrame()
        
        while not rospy.is_shutdown():
            rospy.spin()

    except rospy.ROSInterruptException:
        print ('[arucoGlobamFrame] Interrupt.',file=sys.stderr)
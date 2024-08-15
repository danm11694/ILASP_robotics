#!/usr/bin/env python

from __future__ import print_function

import roslib
import rospy

import geometry_msgs.msg

from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseArray

import tf
import tf2_ros
import tf2_geometry_msgs
import tf.transformations as tr

import sys
import math
import numpy


class arucoGlobalFrame:
    def __init__(self, *args):
        self.__name = rospy.get_name()
        self._arucoPose = rospy.get_param('/aruco_single/pose', '/aruco_single/pose')
        # self._pointPose = rospy.get_param(self.__name + '/pointPose', 'pointPose')
        self._pointPoseArray = rospy.get_param(self.__name + '/pointPoseArray', 'pointPoseArray')

        self.aruco_pose_sub = rospy.Subscriber(self._arucoPose, geometry_msgs.msg.PoseStamped, self.convert_point)
        # self.aruco_pose_pub = rospy.Publisher(self._pointPose, geometry_msgs.msg.PoseStamped, queue_size=1) #nodo pubblicato sul topic self._pointPose di tipo geometry_msgs
        self.aruco_pose_array_pub = rospy.Publisher(self._pointPoseArray, geometry_msgs.msg.PoseArray, queue_size=1)

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        self.radius = 0.048             #   radius of the outer circle

    def convert_point(self,msg): # subscriber

        try:
            # trasforma sistema riferimento dalla camera al marker
            transform = self.tfBuffer.lookup_transform('aruco_marker_frame',
                                       'camera_color_optical_frame', #source frame
                                       rospy.Time(0), #get the tf at first available time
                                       rospy.Duration(1.0)) #wait for 1 second

            # applica la trasformazione al msg del marker
            pose_transformed = tf2_geometry_msgs.do_transform_pose(msg, transform) # coordinate nel sistema di riferimento del marker

            # Questa parte serve per fare la matrice inversa della transform, camera - marker, uguale per tutti i punti
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
        

            pose_array = PoseArray()
            pose_array.header.frame_id = "camera_color_optical_frame"
            pose_array.header.stamp =  rospy.Time.now()

            # punto 1
            first_point = geometry_msgs.msg.PoseStamped()      # geometry_msgs PoseStamped
            first_point.pose.position.z = pose_transformed.pose.position.z + ((self.radius - 0.001) * math.cos(270 * math.pi/180)) # parametrizzazione circonferenza
            first_point.pose.position.x = pose_transformed.pose.position.x  + ((self.radius - 0.001) * math.sin(270 * math.pi/180))
            first_point.pose.position.y = pose_transformed.pose.position.y  +  0.000    #  + 0.4 OFFSET FOR ARUCO MARKER RECOGNITION
            # print (first_point)
            first_point.pose.orientation = geometry_msgs.msg.Quaternion(0,0,0,1)
            pose_first = tf2_geometry_msgs.do_transform_pose(first_point, tf_inv) # applica TransformStamped() alla posa con le coordinate nel sist di riferimento del marker
            pose_array.poses.append(pose_first.pose) # inserisco punto nell'array

            # punto 2
            second_point = geometry_msgs.msg.PoseStamped()      # geometry_msgs PoseStamped
            second_point.pose.position.z = pose_transformed.pose.position.z  + ((self.radius + 0.001) * math.cos(150 * math.pi/180))
            second_point.pose.position.x = pose_transformed.pose.position.x  + ((self.radius + 0.001) * math.sin(150 * math.pi/180))
            second_point.pose.position.y = pose_transformed.pose.position.y  +  0.000    #  + 0.4 OFFSET FOR ARUCO MARKER RECOGNITION
            # print (second_point)
            second_point.pose.orientation = geometry_msgs.msg.Quaternion(0,0,0,1)
            pose_second = tf2_geometry_msgs.do_transform_pose(second_point, tf_inv)
            pose_array.poses.append(pose_second.pose) # inserisco punto nell'array

            # punto 3
            third_point = geometry_msgs.msg.PoseStamped()      # geometry_msgs PoseStamped
            third_point.pose.position.z = pose_transformed.pose.position.z  + ((self.radius + 0.002) * math.cos(30 * math.pi/180))
            third_point.pose.position.x = pose_transformed.pose.position.x  + ((self.radius + 0.002) * math.sin(30 * math.pi/180))
            third_point.pose.position.y = pose_transformed.pose.position.y  +  0.000    #  + 0.4 OFFSET FOR ARUCO MARKER RECOGNITION
            # print (third_point)
            third_point.pose.orientation = geometry_msgs.msg.Quaternion(0,0,0,1)
            pose_third = tf2_geometry_msgs.do_transform_pose(third_point, tf_inv)
            pose_array.poses.append(pose_third.pose) # inserisco punto nell'array

            # punto 4
            fourth_point = geometry_msgs.msg.PoseStamped()      # geometry_msgs PoseStamped
            fourth_point.pose.position.z = pose_transformed.pose.position.z
            fourth_point.pose.position.x = pose_transformed.pose.position.x
            fourth_point.pose.position.y = pose_transformed.pose.position.y  +  0.000 + 0.04 # OFFSET FOR ARUCO MARKER RECOGNITION
            # print (fourth_point)
            fourth_point.pose.orientation = geometry_msgs.msg.Quaternion(0,0,0,1)
            pose_fourth = tf2_geometry_msgs.do_transform_pose(fourth_point, tf_inv)
            pose_array.poses.append(pose_fourth.pose) # inserisco punto nell'array
            # self.aruco_pose_pub.publish(pose_)  pubblica una posa su self._pointPose

            self.aruco_pose_array_pub.publish(pose_array) # publish dell'array

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print('error')


if __name__=='__main__':
    try:
        rospy.init_node('arucoGlobalFrame_py')

        wc = arucoGlobalFrame()

        while not rospy.is_shutdown():
            rospy.spin() # loop di callback ??

    except rospy.ROSInterruptException:
        print ('[arucoGlobamFrame] Interrupt.',file=sys.stderr)

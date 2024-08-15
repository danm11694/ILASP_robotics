#!/usr/bin/env python

import rospy
import math
import dvrk
import PyKDL
import copy
from geometry_msgs.msg import PoseStamped

import tf2_ros
import tf2_geometry_msgs

from tf_conversions import posemath as pm


pose = PoseStamped()
posePSM = PoseStamped()
pose_transformed = PoseStamped()
posePSM_transformed = PoseStamped()

rospy.init_node('test_grasping')

tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)

def pose_cb(data):
    global pose
    pose = copy.deepcopy(data)

def psm_pose_cb(data):
    global posePSM
    posePSM = copy.deepcopy(data)

rospy.Subscriber('/grasping_point', PoseStamped, pose_cb)
rospy.Subscriber('/dvrk/PSM1/position_cartesian_current', PoseStamped, psm_pose_cb)
pt_pub = rospy.Publisher('/psm_pose', PoseStamped,queue_size=1)

try:
    transform = tfBuffer.lookup_transform('PSM1_base',
                               'world', #source frame
                               rospy.Time(0), #get the tf at first available time
                               rospy.Duration(1.0)) #wait for 1 second
                 
    pose_transformed = tf2_geometry_msgs.do_transform_pose(pose, transform)


    # transformPSM = tfBuffer.lookup_transform('world',
    #                            'PSM1_base', #source frame
    #                            rospy.Time(0), #get the tf at first available time
    #                            rospy.Duration(1.0)) #wait for 1 second
                 
    # posePSM_transformed = tf2_geometry_msgs.do_transform_pose(posePSM, transformPSM)


    
except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    print('error')

arm = dvrk.psm('PSM1')
arm.move_jaw(math.pi/4.)

# arm.get_current_position().M

# pose_transformed.pose.orientation.x =  0.707106781172
# pose_transformed.pose.orientation.y =  0.707106781191
# pose_transformed.pose.orientation.z =  2.59734823723e-06
# pose_transformed.pose.orientation.w =  -2.59734823723e-06



# pose_transformed.pose.position.z += 0.01
frame = pm.fromMsg(pose_transformed.pose)
arm.move(frame)

while not rospy.is_shutdown():
    # pt_pub.publish(posePSM_transformed)
    # print(pose_transformed)
    rospy.spin()


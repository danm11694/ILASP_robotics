#!/usr/bin/env python

import rospy
import dvrk
import PyKDL
import copy
from geometry_msgs.msg import PoseStamped, Pose

import tf2_ros
import tf2_geometry_msgs

from tf_conversions import posemath as pm


pose = PoseStamped()
posePSM = PoseStamped()
pose_transformed = PoseStamped()
posePSM_transformed = PoseStamped()

rospy.init_node('psm_pose')

tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)


pt_pub = rospy.Publisher('/dvrk/PSM1/set_position_cartesian', Pose ,queue_size=1)
# Test with KINE TEST dummy in the scene
posePSM.header.frame_id = "world"
posePSM.pose.position.x = -1.550
posePSM.pose.position.y = -0.070
posePSM.pose.position.z = 0.7
posePSM.pose.orientation.x = 0
posePSM.pose.orientation.y = 0
posePSM.pose.orientation.z = 0
posePSM.pose.orientation.w = 1

try:


    transformPSM = tfBuffer.lookup_transform('PSM1_base',
                               'world', #source frame
                               rospy.Time(0), #get the tf at first available time
                               rospy.Duration(3.0)) #wait for 1 second
                 
    posePSM_transformed = tf2_geometry_msgs.do_transform_pose(posePSM, transformPSM)
    print (transformPSM)


    
except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    print('error')

arm = dvrk.psm('PSM1')

# arm.get_current_position().M

posePSM_transformed.pose.orientation.x =  0.707106781172
posePSM_transformed.pose.orientation.y =  0.707106781191
posePSM_transformed.pose.orientation.z =  2.59734823723e-06
posePSM_transformed.pose.orientation.w =  -2.59734823723e-06


# posePSM_transformed.header.frame_id = "PSM1_base"
# frame = pm.fromMsg(posePSM_transformed.pose)
# arm.move(frame)

# arm.home()
# dp = arm.get_desired_position()
cp = arm.get_current_position()

# print(posePSM)
# print("-------------")
# # print(dp)
# print("-------------")
print(cp)

pm_t = Pose()
pm_t = posePSM_transformed.pose
pm_t.position.z += 0.01

# arm.dmove(PyKDL.Vector(0.0, 0.0, 0.05))
# print(arm.get_current_position())

while not rospy.is_shutdown():
    pt_pub.publish(pm_t)
    # print(pose_transformed)
    rospy.spin()


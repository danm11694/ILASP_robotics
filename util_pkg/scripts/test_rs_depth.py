#!/usr/bin/env python

import sys
import rospkg
import rospy
import numpy as np
import random
import operator
from sensor_msgs.msg import Image as IMG
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge,CvBridgeError

import ctypes

from sensor_msgs.msg import PointCloud2
import cv2
import time
from std_msgs.msg import String, Int32, Float64, MultiArrayLayout, Float64MultiArray, Bool, MultiArrayDimension

from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseArray
import geometry_msgs.msg


class dot_recognition:
  def __init__(self):
    # setup ros
    self.bridge = CvBridge()
    self.rospack = rospkg.RosPack()
    self.__name = rospy.get_name()


    self.camera_sub = rospy.Subscriber('camera/color/image_raw', IMG, self.callback)
    self.camera_info_sub = rospy.Subscriber('camera/color/camera_info', CameraInfo, self.camera_info_cb)
    self.camera_depth_sub = rospy.Subscriber('/camera/aligned_depth_to_color/image_raw', IMG, self.depth_callback)
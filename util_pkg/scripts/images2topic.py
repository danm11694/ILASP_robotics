#!/usr/bin/env python

import sys
import rospkg
import rospy
import numpy as np
from sensor_msgs.msg import Image as IMG
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge,CvBridgeError

import cv2
import glob

import time
from std_msgs.msg import String, Int32, Float64, MultiArrayLayout, Float64MultiArray, Bool, MultiArrayDimension

class Images2Topic:
  def __init__(self):
	# setup ros
	self.bridge = CvBridge()
	self.rospack = rospkg.RosPack()
	self._pkg = self.rospack.get_path('endoscope_project') 
	# print (self._pkg)
	
	# self.camera_sub = rospy.Subscriber('/camera/color/image_raw', IMG, self.callback)

	self.camera_pub = rospy.Publisher('/output_video', IMG, queue_size=1)
	self.camera_info_pub = rospy.Publisher('/camera/camera_info', CameraInfo, queue_size=1)

	self.output_image = []
		
		
		
  def readImgs(self):
		jpg_imgs =  self._pkg + '/dataCollected/WinterSchool2019/Test1/'#  +'*.png'
		# print(jpg_imgs)
		
		for i in range(0,288):
			if i < 10:
				temp_img = jpg_imgs + '00000'+str(i)	+'.png'
			elif i >=10 and i < 100:
				temp_img = jpg_imgs + '0000'+str(i)	+'.png'
			else :
				temp_img = jpg_imgs + '000'+str(i)	+'.png'

			# print temp_img	
			img = cv2.imread(temp_img,1)
			self.output_image.append(img)
			# print (self.output_image[i])
			# cv2.imshow("tt", img)
			# cv2.waitKey(0)
		
		# print(type(self.output_image[0]))


		# self.output_image = [cv2.imread(file,1) for file in glob.glob( jpg_imgs)]
		# 000000 - 000288

		# height,width,channels = self.output_image[0].shape
		# print (str(height) + ", " + str(width) + ", " + str(channels))
		#  to crop crop_img = img[y1:y2, x1:x2]
		# img = cv2.resize(self.output_image[0],(width/4,height/4))
		# cv2.imshow("tt",self.output_image[0])
		# cv2.waitKey(0)
		


  def update(self):
	  for img in self.output_image:
	  	self.camera_pub.publish(self.bridge.cv2_to_imgmsg(img, 'bgr8'))

####################################################################################################
#                 MAIN
####################################################################################################
def main(args):
  i2t = Images2Topic()
  rospy.init_node('Images2Topic', anonymous=True)

  i2t.readImgs()


  rate = 3 # hz  # 3   Hz
  ros_rate = rospy.Rate(rate)
			  
  idx = 0
  while not rospy.is_shutdown():
	#   i2t.update()
	  
	  img_msg = IMG()
	  img_msg = i2t.bridge.cv2_to_imgmsg(i2t.output_image[idx], 'bgr8')
	  img_msg.header.frame_id = 'endoscope'
	  img_msg.header.stamp = rospy.get_rostime()
	#   i2t.camera_pub.publish(i2t.bridge.cv2_to_imgmsg(i2t.output_image[idx], 'bgr8'))
	  i2t.camera_pub.publish(img_msg)
	  idx = idx+1

	  camera_info = CameraInfo()
	  camera_info.header.frame_id = 'camera' # 'endoscope'
	  camera_info.header.stamp = rospy.get_rostime()
	  camera_info.height = 1080 # img_msg.height
	  camera_info.width = 1920 # img_msg.width
	  camera_info.distortion_model = 'plumb_bob'
	  fx = 786.1712 
	  fy = 786.735
	  cx = 972.7853636781329
	  cy = 512.0478475286909
	  camera_info.K = [fx, 0, cx, 0, fy, cy, 0, 0, 1]
	  camera_info.D = [0, 0, 0, 0]
	  camera_info.P = [fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1, 0]
	  i2t.camera_info_pub.publish(camera_info)

	  ros_rate.sleep()
	  
# Focal length: 786.1712            786.735
# Intrinsic Matrix: 7.861712323275880e+02,0,0;0,7.867350092044225e+02,0;9.727853636781329e+02,5.120478475286909e+02,1
# Radial Distortion: -0.3897      0.1404
# frame_id: "camera"
# height: 1080
# width: 1920
# distortion_model: "plumb_bob"
# D: [0.0, 0.0, 0.0, 0.0, 0.0]
# K: [1.0, 0.0, 960.0, 0.0, 1.0, 540.0, 0.0, 0.0, 1.0]
# R: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
# P: [1.0, 0.0, 960.0, 0.0, 0.0, 1.0, 540.0, 0.0, 0.0, 0.0, 1.0, 0.0]


	#   if idx == len(i2t.output_image):
	#   	break

if __name__ == '__main__':
	main(sys.argv)
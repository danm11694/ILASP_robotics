#!/usr/bin/env python

import sys
import rospkg
import rospy
import numpy as np
from sensor_msgs.msg import Image as IMG
from cv_bridge import CvBridge,CvBridgeError

import cv2
import time
from std_msgs.msg import String, Int32, Float64, MultiArrayLayout, Float64MultiArray, Bool, MultiArrayDimension

class saveImage:
  def __init__(self):
    # setup ros
    self.bridge = CvBridge()
    self.rospack = rospkg.RosPack()
    self.camera_sub = rospy.Subscriber('/device_0/sensor_1/Color_0/image/data', IMG, self.callback)

    self.output_image = []
    self.counter = 0

  def save(self):
    str_counter = ''
    if(self.counter < 10): 
      str_counter = '000' + str(self.counter)
    elif(self.counter >=10 and self.counter < 100):
      str_counter = '00' + str(self.counter)
    elif(self.counter >=100 and self.counter < 1000):
      str_counter = '0' + str(self.counter)
    else:
      str_counter = str(self.counter)
    print ('Saving ... %s ' % str_counter )
    cv2.imwrite('/home/andrea/bags/bag_1Sett/img_test/img_' + str_counter + '.jpg',self.output_image)
    self.counter +=1

  def callback(self,data):
    try:
      image = self.bridge.imgmsg_to_cv2(data, 'bgr8')     
      self.output_image = image
      self.saveIMG = True
      self.save()

    except CvBridgeError as e:
      print('CvBridgeError', e)
    except Exception as e:
      print('Unknown exception', e)


####################################################################################################
#                 MAIN
####################################################################################################
def main(args):
  ot = saveImage()
  rospy.init_node('saveImage', anonymous=True)

  rate = 100 # hz
  ros_rate = rospy.Rate(rate)
              
  while not rospy.is_shutdown():
      # ot.save()
      
      ros_rate.sleep()

if __name__ == '__main__':
    main(sys.argv)
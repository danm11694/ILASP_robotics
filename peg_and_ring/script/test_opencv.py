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

class opencv_test:
  def __init__(self):
    # setup ros
    self.bridge = CvBridge()
    self.rospack = rospkg.RosPack()
    self.camera_sub = rospy.Subscriber('/camera/color/image_raw', IMG, self.callback)
    self.camera_pub = rospy.Publisher('/track_ring/output_video', IMG, queue_size=1)

    self.output_image = []
    self.counter = 0

    self.multx10 = 0
    self.test_sub = rospy.Subscriber('/test_integer', Int32, self.test_cb)
    self.saveIMG = False
    self.array_pub = rospy.Publisher('/test_array', Float64MultiArray, queue_size=10)


  def _make_multiarray(self, iterable, label):
    array_list = []
    for el in iterable:
      array_list.append(el)

    dim = MultiArrayDimension()
    dim.size = len(array_list)
    dim.label = label
    dim.stride = len(array_list)
    temp_array = Float64MultiArray()
    temp_array.data = array_list
    temp_array.layout.dim.append(dim)
    temp_array.layout.data_offset = 0
    return temp_array

  def test_cb(self, msg):
    self.multx10 = msg.data * 10
    self.saveIMG = True
    

  def test_pub(self):
    results = self._make_multiarray( [0,1,2,3,4] , 'testing')
    self.array_pub.publish(results)

    
  def save(self):
    if self.saveIMG:
      print ('Saving ... %s ' % str(self.counter) )
      cv2.imwrite('/home/andrea/phantom_prostate/phantom_img_' + str(self.counter) + '.jpg',self.output_image)
      self.counter +=1
    self.saveIMG = False
    
    

  def callback(self,data):
    try:
      image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
      

      
      
      self.output_image = image
    #   print('YES')

    except CvBridgeError as e:
      print('CvBridgeError', e)
    except Exception as e:
      print('Unknown exception', e)

  def update(self):
      cv2.circle(self.output_image,(447,63), 63, (0,0,255), -1)
      self.camera_pub.publish(self.bridge.cv2_to_imgmsg(self.output_image, 'bgr8'))

####################################################################################################
#                 MAIN
####################################################################################################
def main(args):
  ot = opencv_test()
  rospy.init_node('opencv_test', anonymous=True)
  # ot.update()
        
  rate = 100 # hz
  ros_rate = rospy.Rate(rate)
              
  while not rospy.is_shutdown():
      # ot.update()
      # ot.test_pub()
      ot.save()
      
      ros_rate.sleep()

if __name__ == '__main__':
    main(sys.argv)
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
    self._pointPoseArray = rospy.get_param(self.__name + '/pointPoseArray', 'pointPoseArray')

    self.camera_sub = rospy.Subscriber('camera/color/image_raw', IMG, self.callback)
    self.camera_info_sub = rospy.Subscriber('camera/color/camera_info', CameraInfo, self.camera_info_cb)
    self.camera_depth_sub = rospy.Subscriber('/camera/aligned_depth_to_color/image_raw', IMG, self.depth_callback)
    self.marker_pose_sub = rospy.Subscriber('/aruco_single/pose', geometry_msgs.msg.PoseStamped, self.marker_cb)
    self.marker_pose_sub = rospy.Subscriber('/stag_ros/pose', geometry_msgs.msg.PoseStamped, self.marker_cb)


    self.camera_pub = rospy.Publisher('/debug/output_video', IMG, queue_size=1)
    # self.cloud_pub = rospy.Publisher("/debug/cloud", PointCloud2, queue_size=2)
    self.aruco_pose_array_pub = rospy.Publisher(self._pointPoseArray, geometry_msgs.msg.PoseArray, queue_size=1)

    self.output_image = []
    self.depth_output_image = []
    self.image_width = 0
    self.image_height = 0
    self.K = []
    self.D = []
    self.marker_position = []
    self.marker_orientation = []
    self.cloud_out = []
    self.points = []
    self.id = [0,4,8]

    #test mask
    size = (640,480,3)
    self.mask_mat_temp = np.zeros(size,dtype= np.uint8)
    cv2.rectangle(self.mask_mat_temp, (150,150), (250,250), (255,255,255), -1)
    self.mask_mat_temp = cv2.cvtColor(self.mask_mat_temp, cv2.COLOR_BGR2GRAY)
    ret,self.mask_mat_temp = cv2.threshold(self.mask_mat_temp,127,255,cv2.THRESH_BINARY)
    # self.output_image = cv2.imread("/home/andrea/ars_workspace/src/utils/util_pkg/resources/scene.png")
    # cv2.imshow("mask_mat", self.mask_mat_temp)
    # cv2.waitKey(0)


#----------------------------------------------------------------------------
#                 CALLBACKS

  def callback(self,data):
    try:
      image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
      self.image_width = data.width
      self.image_height = data.height
      self.output_image = image


    except CvBridgeError as e:
      print('CvBridgeError', e)
    except Exception as e:
      print('Unknown exception', e)


  def depth_callback(self,data):
    try:
      image = self.bridge.imgmsg_to_cv2(data, '16UC1')
      self.depth_output_image = image
      #cv2.imshow("image", image)
      #cv2.waitKey(0)

    except CvBridgeError as e:
      print('CvBridgeError', e)
    except Exception as e:
      print('Unknown exception', e)

  def camera_info_cb(self,data):
    self.K = np.matrix(np.array(data.K ,dtype=np.float)).reshape(-1,3)
    self.D = np.matrix(np.array(data.D ,dtype=np.float)).reshape(-1,5)

  def marker_cb(self,msg):
    self.marker_position = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
    self.marker_orientation = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
    # print(self.marker_position)


#                   END CALLBACKS
#---------------------------------------------------------------------------

  # square above marker
  def filter_marker(self):
    # print(self.marker_position)
    # name = rospy.get_param_names()
    # print name
    if(len(self.marker_position) > 0 ):
        #if self.marker_pose_sub == '/aruco_single/pose':
        #    print('aruco')
        # aruco - white marker
        #start_point_list = self.convert3D2D([self.marker_position[0]+0.030, self.marker_position[1]-0.030, self.marker_position[2]])
        #end_point_list = self.convert3D2D([self.marker_position[0]-0.030, self.marker_position[1]+0.030, self.marker_position[2]])
        #start_point = (int(start_point_list[0][0][0]), int(start_point_list[0][0][1]))
        #end_point = (int(end_point_list[0][0][0]), int(end_point_list[0][0][1]))
        # stag marker
        start_point_list = self.convert3D2D([self.marker_position[0]+0.025, self.marker_position[1]-0.025, self.marker_position[2]])
        end_point_list = self.convert3D2D([self.marker_position[0]-0.025, self.marker_position[1]+0.025, self.marker_position[2]])
        start_point = (int(start_point_list[0][0][0]), int(start_point_list[0][0][1]))
        end_point = (int(end_point_list[0][0][0]), int(end_point_list[0][0][1]))
        # Blue color in BGR
        color = (255, 255, 255)
        # Line thickness of 2 px
        thickness = -1
        cv2.rectangle(self.output_image, start_point, end_point, color, thickness)

  # convert 2x pixel points to 3d points
  def _3DFrom2D(self, pixel_points, img_depth):
    pose_array = PoseArray()
    pose_array.header.frame_id = "camera_color_optical_frame"
    pose_array.header.stamp =  rospy.Time.now()

    cx = self.K.item(2)
    cy = self.K.item(5)
    fx_inv = 1.0 / self.K.item(0)
    fy_inv = 1.0 / self.K.item(4)
    counter = 0
    print (pixel_points)
    for i in pixel_points:
      v = i[1]
      u = i[0]
      z_raw = img_depth[v,u]

      print('z_raw is :' + str(z_raw) , i)

      if (z_raw != 0.0) :
          # print (z_raw)
        counter +=1
        z_metric = z_raw *0.001
        dot_pose = geometry_msgs.msg.Pose()
        dot_pose.position.x = z_metric * (( u - cx) * fx_inv)
        dot_pose.position.y = z_metric * (( v - cy) * fy_inv)
        dot_pose.position.z = z_metric
        if not self.marker_position:
            dot_pose.orientation.w = 1.0
        else:
            dot_pose.orientation.x = self.marker_orientation[0]
            dot_pose.orientation.y = self.marker_orientation[1]
            dot_pose.orientation.z = self.marker_orientation[2]
            dot_pose.orientation.w = self.marker_orientation[3]
        pose_array.poses.append(dot_pose)
        print dot_pose

    if len(pixel_points) != 4:
        print('marker')
        dot_pose = geometry_msgs.msg.Pose()
        dot_pose.position.x = self.marker_position[0]
        dot_pose.position.y = self.marker_position[1]
        dot_pose.position.z = self.marker_position[2]
        dot_pose.orientation.x = self.marker_orientation[0]
        dot_pose.orientation.y = self.marker_orientation[1]
        dot_pose.orientation.z = self.marker_orientation[2]
        dot_pose.orientation.w = self.marker_orientation[3]
        pose_array.poses.append(dot_pose)
        print dot_pose

    self.aruco_pose_array_pub.publish(pose_array) # publish dell'array

          # self.cloud_out.append([dot_pose.position.x, dot_pose.position.y, dot_pose.position.z])
    return pose_array

  def convert3D2D(self, pose):
    # Map to 2d image points
    point_3d = np.array(pose, dtype=np.float).reshape(-1, 3)
    (point_2d, _) = cv2.projectPoints(point_3d, np.array([0,0,0],dtype=np.float), np.array([0,0,0],dtype=np.float), self.K, self.D)
    return point_2d

  def get_points(self):
      # read grayscale image
      # img = cv2.imread("/home/katia/catkin_ws/src/utils/util_pkg/resources/image_from_camera.png", cv2.IMREAD_GRAYSCALE)
      # img = cv2.imread("/ /image_from_camera.png", cv2.IMREAD_GRAYSCALE)
      img = cv2.cvtColor(self.output_image, cv2.COLOR_BGR2GRAY)
      #img = self.output_image

      # make the image binary
      ret,th = cv2.threshold(img,127,255,cv2.THRESH_BINARY_INV)
      #cv2.imshow("th1", th)

      # horizontal line cancellation
      horizontal_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (25,1))
      detected_lines_h = cv2.morphologyEx(th, cv2.MORPH_OPEN, horizontal_kernel, iterations=2)
      #cv2.imshow("detected lines horiz", detected_lines_h)
      #detected_lines_h_conv.convertTo(detected_lines_h, CV_8UC1, 1.0/255.0);
      cnts_h = cv2.findContours(detected_lines_h, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
      cnts_h = cnts_h[0] if len(cnts_h) == 2 else cnts_h[1]
      for c in cnts_h:
          cv2.drawContours(img, [c], -1, (255,255,255), 2)

      repair_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (1,6))
      res_h = 255 - cv2.morphologyEx(255 - img, cv2.MORPH_CLOSE, repair_kernel, iterations=1)
      #cv2.imshow("res1", res_h)

      # vertical line cancellation
      ret2,th2 = cv2.threshold(res_h,127,255,cv2.THRESH_BINARY_INV)
      #cv2.imshow("th2", th2)
      vertical_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (1,25))
      detected_lines_v = cv2.morphologyEx(th2, cv2.MORPH_OPEN, vertical_kernel, iterations=2)
      #cv2.imshow("detected lines vert", detected_lines_v)
      cnts_v = cv2.findContours(detected_lines_v, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
      cnts_v = cnts_v[0] if len(cnts_v) == 2 else cnts_v[1]
      for c in cnts_v:
          cv2.drawContours(res_h, [c], -1, (255,255,255), 2)

      repair_kernel_v = cv2.getStructuringElement(cv2.MORPH_RECT, (5,1))
      res_v = 255 - cv2.morphologyEx(255 - res_h, cv2.MORPH_CLOSE, repair_kernel_v, iterations=1)
      res = res_v
      # cv2.imshow("res2", res_v)
      # cv2.waitKey(0)

      # locate points in the image
      params = cv2.SimpleBlobDetector_Params()
      params.filterByCircularity = True
      params.minCircularity = 0.3
      params.filterByInertia = True
      params.minInertiaRatio = 0.04
      params.filterByConvexity = True
      params.minConvexity = 0.7

      detector = cv2.SimpleBlobDetector_create(params)
      keypoints = detector.detect(res)

      # draw detected blobs as red circles.
      # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
      self.output_image = cv2.drawKeypoints(res, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

      print len(keypoints)
      # print self.marker_position

      # trasformation keypoints in pixel coordinates
      coords = [[0,0]]
      for c in range(1,len(keypoints)):
          coords.append([0,0])
      min_x = 1000
      max_x = 0
      min_y = 1000
      max_y = 0
      for i in range(0,len(keypoints)):
          x = keypoints[i].pt[0] #i is the index of the blob you want to get the position
          y = keypoints[i].pt[1]
          coords[i][0] = int(x)  # ogni tanto crasha questo, bisogna controllare sempre la dimensione dei keypoints
          coords[i][1] = int(y)

          # x: min - max
          if coords[i][0] < min_x:
              min_x = coords[i][0]
          if coords[i][0] > max_x:
              max_x = coords[i][0]

      med_x = (min_x + max_x) / 2
      # print med_x

#      if [0,0] in coords:
#          coords.remove([0,0])
#      print coords

      # sort coords according to y
      coords.sort(key=operator.itemgetter(1))
      # print coords

      # number the points found from 1 to 12
      half_1 = [[0,0]]
      for h1 in range(1,len(keypoints)/2 +1):
          half_1.append([0,0])
      half_2 = [[0,0]]
      for h2 in range(1,len(keypoints)/2 +1):
          half_2.append([0,0])

      m = 0
      n = 0
      for l in range(0,len(coords)):
          if coords[l][0] < med_x:
              half_1[m][0] = coords[l][0]
              half_1[m][1] = coords[l][1]
              m = m + 1
          elif coords[l][0] > med_x:
              half_2[n][0] = coords[l][0]
              half_2[n][1] = coords[l][1]
              n = n + 1

      if [0,0] in half_1:
          half_1.remove([0,0])
      if [0,0] in half_2:
          half_2.remove([0,0])
      print half_1
      print half_2
      #half_1.remove([0,0])
      #half_2.remove([0,0])
      half_1.reverse()
      # print half_1
      # print half_2

      r = 0
      for p in range(0,len(half_1)):
          coords[r][0] = half_1[p][0]
          coords[r][1] = half_1[p][1]
          #print half_1[p]
          r = r + 1

      for q in range(0,len(half_2)):
          coords[r][0] = half_2[q][0]
          coords[r][1] = half_2[q][1]
          #print half_2[q]
          r = r + 1

      # print coordinates of all points
      s = "Punto {}: {} - {}"
      for z in range(0, len(coords)):
          print(s.format(z+1, coords[z][0], coords[z][1]))

      # select 3 points
      points = [[0,0],[0,0],[0,0],[0,0]]
      points[0][0] = coords[0][0]
      points[0][1] = coords[0][1]
      points[1][0] = coords[4][0]
      points[1][1] = coords[4][1]
      points[2][0] = coords[8][0]
      points[2][1] = coords[8][1]

      self.points = points
      # print 3 points
      for i in range(0,3):
          print(self.id[i]+1,coords[self.id[i]][0], coords[self.id[i]][1] )

      # found fourth point
      if not self.marker_position:
          points[3][0] = (points[0][0] + points[1][0] + points[2][0]) / 3
          points[3][1] = (points[0][1] + points[1][1] + points[2][1]) / 3
          print('no marker')
          print points[3]

      if points[3] == [0,0]:
          points.remove([0,0])

      # number points
      font = cv2.FONT_HERSHEY_SIMPLEX
      for i in range(0,len(keypoints)):
          cv2.putText(self.output_image, str(i+1), (coords[i][0],coords[i][1]), font, 0.5, (255,255,255), 1)

      # show keypoints
      cv2.imshow("Keypoints", self.output_image)
      cv2.waitKey(0)

  def test_convert3D2D(self):
      point_2d = self.convert3D2D([0,0,0.30])
      cv2.circle(self.output_image,(int(point_2d[0][0][0]), int(point_2d[0][0][1])), 10, (0,0,255), -1)

  def test_pc2DMask(self):
      pose_array = PoseArray()
      pose_array = self._3DFrom2D(self.points, self.depth_output_image)
      self.aruco_pose_array_pub.publish(pose_array) # publish dell'array

  def update(self):

      self.filter_marker()
      self.get_points()
      self.test_pc2DMask()

      self.camera_pub.publish(self.bridge.cv2_to_imgmsg(self.output_image, 'bgr8'))

####################################################################################################
#                 MAIN
####################################################################################################
def main(args):
  ot = dot_recognition()
  rospy.init_node('dot_recognition', anonymous=True)

  rate = 30 # hz
  ros_rate = rospy.Rate(rate)

  while not rospy.is_shutdown():
      ot.update()

      ros_rate.sleep()

if __name__ == '__main__':
    main(sys.argv)

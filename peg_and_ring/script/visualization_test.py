#!/usr/bin/env python

import sys
import rospkg
import rospy

from visualization_msgs.msg import Marker


class visualization:
    def __init__(self):

        self.marker_pub = rospy.Publisher('/sphere', Marker, queue_size=1)

        self.sphere = Marker()

    def update(self):
        self.sphere.header.frame_id = "sphere"      # or you can use the "fixed frame" in rviz -> world
        self.sphere.header.stamp = rospy.Time.now()
        self.sphere.ns = "sphere"
        self.sphere.action = 0 # self.sphere.ADD
        self.sphere.type = 2 # self.sphere.SPHERE  
        self.sphere.id = 0

        self.sphere.scale.x = 0.1
        self.sphere.scale.y = 0.1
        self.sphere.scale.z = 0.1
        
        self.sphere.pose.position.x = 0.0
        self.sphere.pose.position.y = 0.0
        self.sphere.pose.position.z = 1.5

        self.sphere.color.r = 1.0
        self.sphere.color.g = 0.0
        self.sphere.color.b = 0.0
        self.sphere.color.a = 1.0


        self.marker_pub.publish(self.sphere)
        


####################################################################################################
#                 MAIN
####################################################################################################
def main(args):
  vm = visualization()
  rospy.init_node('visualization_test', anonymous=True)
        
  rate = 100 # hz
  ros_rate = rospy.Rate(rate)
              
  while not rospy.is_shutdown():
      vm.update()
      ros_rate.sleep()

if __name__ == '__main__':
    main(sys.argv)
    
#!/usr/bin/env python

from __future__ import print_function


import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
import sys
import geometry_msgs.msg

import tf2_ros
import tf2_py as tf2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Int32
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud


class ConvertPCD:
    def __init__(self,*args):
        self.cloud = []
        self.cloud_out = []
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.transform = []
        
        self.cloud_updated = False

        self.__name = rospy.get_name()
        self._in_topic = rospy.get_param(self.__name + '/in', 'output_cloud')
        self._out_topic = rospy.get_param(self.__name + '/out', 'output_cloud_transformed')
        self.cloud_sub = rospy.Subscriber(self._in_topic, PointCloud2, self.cloudCallback,queue_size=2)
        self.pub = rospy.Publisher(self._out_topic , PointCloud2, queue_size=2)
        self.target_frame = rospy.get_param('target_frame', 'world')
        self.source_frame = rospy.get_param('source_frame', 'ECM')
    
    def cloudCallback(self,msg):
        self.cloud_out = []
        self.cloud = []
        if len(self.cloud) == 0:
            for p in point_cloud2.read_points(msg):
                self.cloud.append([p[0], p[1], p[2]])
        try:
            self.transform = self.tf_buffer.lookup_transform(self.target_frame, #target frame
                                           self.source_frame, #source frame
                                           rospy.Time(0), #get the tf at first available time
                                           rospy.Duration(1.0)) #wait for 1 second
            # print(transform)
            cloud_pc = do_transform_cloud(msg, self.transform)
            self.pub.publish(cloud_pc)
            if len(self.cloud_out) == 0:
                for p1 in point_cloud2.read_points(cloud_pc):
                    self.cloud_out.append([p1[0], p1[1], p1[2]])

            self.cloud_updated = True

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print('[GEOMETRY GRASP] Error - No RING POINT CLOUD')
   
    def quatmult(self, q1, q2):
        x1, y1, z1, w1 = q1
        x2, y2, z2, w2 = q2
        x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
        y = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2
        z = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2
        w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
        return x, y, z, w

    def update(self):
        print('[ConvertPCD] TO DO - send transform')


if __name__=='__main__':
    try:
        rospy.init_node('ConvertPCD_py')
        
        gg = ConvertPCD()
        # gg.update()
        # rospy.spin()
                
        rate = 100 # hz
        ros_rate = rospy.Rate(rate)
                
        while not rospy.is_shutdown():
            gg.update()
            ros_rate.sleep()


    except rospy.ROSInterruptException:
        print ('[ConvertPCD] Interrupt.',file=sys.stderr)
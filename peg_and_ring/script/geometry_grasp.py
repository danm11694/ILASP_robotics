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


class GeometryGrasp:
    def __init__(self,*args):
        self.cloud = []
        self.cloud_out = []
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.transform = []
        

        self.cylinder_cloud = []
        self.cylinder_cloud_out = []

        self.cloud_updated = False

        self.max_x = 0
        self.max_y = 0
        self.max_z = 0
        self.min_x = 0
        self.min_y = 0
        self.min_z = 0
        self.x_list = []
        self.y_list = []
        self.z_list = []
        self.pub_done = False
        self._color_list = ['RED','GREEN','BLUE','YELLOW']
        self.current_color = 'RED'
        self.current_color_num = 0
        
        # [TO DO] TOPIC NAME as parameters
        self.pub = rospy.Publisher("/point_cloud_transformed", PointCloud2, queue_size=2)
        self.cloud_sub = rospy.Subscriber('/ring_cloud', PointCloud2, self.cloudCallback,queue_size=2)
        self.cylinder_pub = rospy.Publisher("/cylinder_transformed", PointCloud2, queue_size=2)
        self.cylinder_cloud_sub = rospy.Subscriber('/cylinder_cloud', PointCloud2, self.cylinderCloudCallback,queue_size=2)
        self.pose_pub = rospy.Publisher('/franka_pub/grasping_point', geometry_msgs.msg.PoseStamped, queue_size=100)
        self.color_sub = rospy.Subscriber('/color', Int32, self.color_cb)
        
    

    def color_cb(self, msg):
        dict_color = {0 : 'RED', 1:'GREEN', 2:'BLUE', 3:'YELLOW'}
        if (msg.data > 3):
            self.current_color_num = 0
            self.current_color = 'RED' 
            # print ('NO VALID COLOR')
        else:
            #print ('ENTERING HERE')
            self.current_color = dict_color.get(msg.data)
            self.current_color_num = msg.data
            self.pub_done = False

    def cloudCallback(self,msg):
        self.cloud_out = []
        self.cloud = []
        if len(self.cloud) == 0:
            for p in point_cloud2.read_points(msg):
                self.cloud.append([p[0], p[1], p[2]])
        try:
            self.transform = self.tf_buffer.lookup_transform('panda_link0', #targert frame
                                           'camera_link', #source frame
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
   
    def cylinderCloudCallback(self,msg):
        self.cylinder_cloud_out = []
        self.cylinder_cloud = []
        if len(self.cylinder_cloud) == 0:
            for p in point_cloud2.read_points(msg):
                self.cylinder_cloud.append([p[0], p[1], p[2]])
        try:
            transform = self.tf_buffer.lookup_transform('panda_link0', #targert frame
                                           'camera_link', #source frame
                                           rospy.Time(0), #get the tf at first available time
                                           rospy.Duration(1.0)) #wait for 1 second
            # print(transform)
            cloud_pc = do_transform_cloud(msg, transform)
            cloud_pc.header.stamp = rospy.get_rostime()
            self.cylinder_pub.publish(cloud_pc)
            # if len(self.cylinder_cloud_out) == 0:
                # for p1 in point_cloud2.read_points(cloud_pc):
                    # self.cylinder_cloud_out.append([p1[0], p1[1], p1[2]])

            # self.cloud_updated = True

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
        # rospy.wait_for_message('/ring_cloud', PointCloud2)       
        if self.cloud_updated == True:
            self.x_list = []
            self.y_list = []
            self.z_list = []

            while len(self.cloud) == 0:
                rospy.sleep(0.01)

            while len(self.cloud_out) == 0:
                rospy.sleep(0.01)

            while len(self.cylinder_cloud) == 0:
                rospy.sleep(0.01)

            for i in range(len(self.cloud_out)):
                self.x_list.append(self.cloud_out[i][0])
                self.y_list.append(self.cloud_out[i][1])
                self.z_list.append(self.cloud_out[i][2])
            self.max_x = max(self.x_list)
            self.max_y = max(self.y_list)
            self.max_z = max(self.z_list)
            self.min_x = min(self.x_list)
            self.min_y = min(self.y_list)
            self.min_z = min(self.z_list)
            centroid_x = (self.max_x + self.min_x ) / 2
            centroid_y = (self.max_y + self.min_y ) / 2
            centroid_z = (self.max_z + self.min_z ) / 2
            pose_ee = geometry_msgs.msg.PoseStamped()
            pose_ee.header.stamp = rospy.Time.now()
            pose_ee.header.frame_id = "panda_link0"
            pose_ee.pose.position.x = self.min_x
            pose_ee.pose.position.y = centroid_y
            pose_ee.pose.position.z = centroid_z  # centroid_z self.min_z # GIOVANNI - MIN_Z
            pose_ee.pose.orientation.x = 0.71 #transform.transform.rotation.x
            pose_ee.pose.orientation.y = 0.7 #transform.transform.rotation.y
            pose_ee.pose.orientation.z = -0.02 #transform.transform.rotation.z
            pose_ee.pose.orientation.w = 0.02 #transform.transform.rotation.w
            # print(pose_ee)
            if  self.pub_done == False:
                rospy.sleep(1)
                self.pose_pub.publish(pose_ee)
                print(pose_ee)
                self.pub_done = True
                self.cloud_updated = False


if __name__=='__main__':
    try:
        rospy.init_node('GeometryGrasp_py')
        
        gg = GeometryGrasp()
        # gg.update()
        # rospy.spin()
                
        rate = 100 # hz
        ros_rate = rospy.Rate(rate)
                
        while not rospy.is_shutdown():
            gg.update()
            ros_rate.sleep()


    except rospy.ROSInterruptException:
        print ('[GeometryGrasp] Interrupt.',file=sys.stderr)
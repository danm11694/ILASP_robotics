#!/usr/bin/env python  

from __future__ import print_function

import roslib
import rospy
import rospkg
import geometry_msgs.msg
import sensor_msgs.msg
import tf2_ros
import tf2_geometry_msgs
import std_msgs.msg

import sys
import threading                                # for locks

from panda_peg_ring.msg import PoseStampedArray
from std_msgs.msg import String, Int32, Float64, MultiArrayLayout, Float64MultiArray, Bool, MultiArrayDimension

class SaveData:
    def __init__(self, *args):
        self.__name = rospy.get_name()
        self._ee_topic = rospy.get_param(self.__name + '/ee_pose', 'ee_pose')
        self._filename = rospy.get_param(self.__name + '/filename', 'test.txt')

        self._rospack = rospkg.RosPack()
        # self.out_file = open(self._rospack.get_path('panda_peg_ring') + '/script/giovanni_'+self._filename,'a')


        # self.peg_pose_pub = rospy.Publisher(self._peg_topic, geometry_msgs.msg.PoseStamped, queue_size=1)
        self.ee_pose_sub = rospy.Subscriber(self._ee_topic, geometry_msgs.msg.PoseStamped, self.ee_callback, queue_size=1)
        self.joint_state_sub = rospy.Subscriber('/joint_states', sensor_msgs.msg.JointState, self.jp_callback, queue_size=1)


        self.array_pub = rospy.Publisher('/test_array', Float64MultiArray, queue_size=1)

        self.string_pub = rospy.Publisher('/output_string', String, queue_size=1)

        self.ee_pos_vector = []
        self.jp_pos_vector = []
        self.test_list = []
        self.count = 0

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


    def jp_callback(self,msg):
        self.jp_pos_vector.append(msg.position)
        # print(self.jp_pos_vector)
        # self.test_list.append([self.count,self.count*10 ])
        # self.count += 1

    def ee_callback(self,msg):
        self.ee_pos_vector.append([msg.header.stamp.secs, msg.pose.position.x,msg.pose.position.y,msg.pose.position.z, msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w ])
        # print(self.ee_pos_vector)
        # print( [msg.pose.position.x,msg.pose.position.y,msg.pose.position.z])

    def update(self):
        if(len(self.ee_pos_vector) > 0) and (len(self.jp_pos_vector) > 0):
            # print (str(self.ee_pos_vector[-1] ))
            self.string_pub.publish( ((str(self.ee_pos_vector[-1] ).replace('[','')).replace(',','')).replace(']','') + ' '
                                    + ((str(self.jp_pos_vector[-1] ).replace('(','')).replace(',','')).replace(')','') )
            
        # if(len(self.test_list) > 0):
        #     self.out_file.write( str(self.test_list[-1]) + '\n')
        #     results = self._make_multiarray( self.test_list[-1], 'testing')
        #     self.array_pub.publish(results)

        # self.out_file.write( str( self.ee_pos_vector) + ' ' + str(self.jp_pos_vector) + '\n')

        
        
        
if __name__=='__main__':
    try:
        rospy.init_node('SaveData_py')        
        pr = SaveData()        
        rate = 30 # hz
        ros_rate = rospy.Rate(rate)                
        while not rospy.is_shutdown():            
            pr.update()            
            ros_rate.sleep()
            # rospy.spin()

    except rospy.ROSInterruptException:
        print ('[SaveData] Interrupt.',file=sys.stderr)
#!/usr/bin/env python  

from __future__ import print_function

import roslib
import rospy
import rospkg
import geometry_msgs.msg
import tf2_ros
import tf2_geometry_msgs
import std_msgs.msg

import sys
import threading                                # for locks

from panda_peg_ring.msg import PoseStampedArray

class PegRegistrator:
    def __init__(self, *args):
        self.__name = rospy.get_name()
        self._peg_topic = rospy.get_param(self.__name + '/peg_pose', 'peg_pose')
        self._ee_topic = rospy.get_param(self.__name + '/ee_pose', 'ee_pose')
        self._load = rospy.get_param(self.__name + '/use_existing', False)
        self._num_pegs = rospy.get_param(self.__name + '/num_pegs', 4)
        self._filename = rospy.get_param(self.__name + '/filename', 'test.txt')
        self._all_pegs = rospy.get_param(self.__name + '/all_pegs', 'all_pegs')
        
        self._color_list = ['RED','GREEN','BLUE','YELLOW']
        self.current_color = 'RED'
        self.current_color_num = 0
        self.goal = []
        self.goalArray = PoseStampedArray()
        self._count = 0

        self._rospack = rospkg.RosPack()
        # self.out_file = open(self._rospack.get_path('panda_peg_ring') + '/config/pegs_record_14_01.txt','a')
        self.out_file = open(self._rospack.get_path('panda_peg_ring') + '/config/'+self._filename,'a')

        # self._ref_lock = threading.Lock()
        self._act_lock = threading.Lock()



        if (self._load):
            self.read_pegs()
            self.color_sub = rospy.Subscriber('/color', std_msgs.msg.Int32, self.color_cb)
            self.peg_pose_pub = rospy.Publisher(self._peg_topic, geometry_msgs.msg.PoseStamped, queue_size=1)
            self.pose_array_pub = rospy.Publisher(self._all_pegs, PoseStampedArray, queue_size=100)
        else:
            self.peg_pose_sub = rospy.Subscriber(self._ee_topic, geometry_msgs.msg.PoseStamped, self.store_pegs, queue_size=1)


        self._recorded = False

        # self.tfBuffer = tf2_ros.Buffer()
        # self.listener = tf2_ros.TransformListener(self.tfBuffer)

    def color_cb(self,msg):
        dict_color = {0 : 'RED', 1:'GREEN', 2:'BLUE', 3:'YELLOW'}
        if (msg.data > 3):
            self.current_color_num = 0
            self.current_color = 'RED' 
            print ('NO VALID COLOR')
        else :            
            self.current_color = dict_color.get(msg.data)
            self.current_color_num = msg.data

    def update(self):
        self._act_lock.acquire() 
        # print (self.current_color_num)
        # print (type(self.goal[self.current_color_num]))
        self.peg_pose_pub.publish(self.goal[self.current_color_num+1])

        self.pose_array_pub.publish(self.goalArray)
        self._act_lock.release() 

    ## TO FIX THIS 
    # def store_pegs(self, msg):
    #     self._act_lock.acquire()
    #     if self._recorded == False:
    #         print(msg)            
            
    #         print('PRESS ENTER')
    #         raw_input()

    #         if self._count < (self._num_pegs+1):
    #             if self_._count == 0:
    #                 self.out_file.write( 'BASE ' + 
    #                                 + str(msg.pose.position.x) + ' ' + str(msg.pose.position.y) + ' ' + str(msg.pose.position.z) + ' '  
    #                                 + str(msg.pose.orientation.x) + ' ' + str(msg.pose.orientation.y) + ' ' + str(msg.pose.orientation.z) + ' ' + str(msg.pose.orientation.w) + '\n')
    #             else:
    #                 self.out_file.write( str(self._color_list[self._count-1]) + ' ' 
    #                                 + str(msg.pose.position.x) + ' ' + str(msg.pose.position.y) + ' ' + str(msg.pose.position.z) + ' '  
    #                                 + str(msg.pose.orientation.x) + ' ' + str(msg.pose.orientation.y) + ' ' + str(msg.pose.orientation.z) + ' ' + str(msg.pose.orientation.w) + '\n')
    #            rospy.sleep(1)
    #            self._count += 1             
    #         else:
    #            self.out_file.close()
    #            self._recorded = True
    #         print (self._count)  
            
    #     self._act_lock.release() 

    def read_pegs(self):
        lines = []
        print('READING FILE')
        try:
            # in_file = open(self._rospack.get_path('panda_peg_ring') + '/config/pegs_record_14_01.txt')
            in_file = open(self._rospack.get_path('panda_peg_ring') + '/config/'+self._filename)            

            self.goalArray.header.frame_id = 'all_pegs'
            self.goalArray.header.stamp = rospy.Time.now()

            for line in in_file:
                line = line.rstrip().split(' ')
                # print(line)

                goal = geometry_msgs.msg.PoseStamped()
                goal.header.frame_id = '/panda_link0'
                goal.header.stamp = rospy.Time.now()
                goal.pose.position.x = float(line[1])
                goal.pose.position.y = float(line[2])
                goal.pose.position.z = float(line[3])
                goal.pose.orientation.x = float(line[4])
                goal.pose.orientation.y = float(line[5])
                goal.pose.orientation.z = float(line[6])
                goal.pose.orientation.w = float(line[7])

                self.goal.append(goal)
                self.goalArray.pose.append(goal)

            in_file.close()

        except IOError:
            print('ERROR - cannot open configuration file')


if __name__=='__main__':
    try:
        rospy.init_node('PegRegistrator_py')
        
        pr = PegRegistrator()
        
        rate = 100 # hz
        ros_rate = rospy.Rate(rate)
                
        while not rospy.is_shutdown():

            if pr._load:
                pr.update()
            
            ros_rate.sleep()
            # rospy.spin()

    except rospy.ROSInterruptException:
        print ('[PegRegistrator] Interrupt.',file=sys.stderr)
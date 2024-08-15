#!/usr/bin/env python3
import numpy as np
import math
import re
import os
import roslib
import rospkg
import rospy
import actionlib
import sys
import copy
from std_msgs.msg import Header, Int32, Bool
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseArray, PoseStamped, Pose, Point, Quaternion
from tf import TransformListener
from tf_conversions import posemath as pm
from tf.transformations import quaternion_from_euler as compute_orient, quaternion_multiply 
import quaternion
from quaternion import distance as orient_dist
from dvrk_task_msgs.msg import CloudArray, ObstArray, ActionRequestFiner, ContextModel, ActionArray, BoolArray, PoseStampedArray
import dvrk
import json
import crtk




#comm class
class Situation_awareness(object):
    def __init__(self, ral):
        rospack = rospkg.RosPack()
        self.path = rospack.get_path("robot_control") + '/config/'
        self.task = "pegring"
        with open(self.path + self.task + ".json") as f:
            actions = json.load(f)["actions"]
        self.bimanual = [a for a in actions if a["policy"] == "move_center"][0]["n_arms"] < 2
        self.dvrk_frame_id = rospy.get_param("dvrk_frame_id")

        self.ring_rad = 0.008
        self.peg_height = 0.005
        self.peg_rad = 0.0025
        self.psm1 = dvrk.psm(ral, 'PSM1')
        self.psm2 = dvrk.psm(ral, 'PSM2')
        self.psm1_standard_orient = None
        self.psm2_standard_orient = None
        self.psm1_start = Pose()
        self.psm2_start = Pose()
        self.init_orient = []
        self.offset_x = []
        self.offset_y = []
        self.has_goal = []
        self.old_fluents = []
        self.old_actions = []
        self.fixed_fluents = []
        self.almost_placed_psm1 = ""
        self.almost_placed_psm2 = ""
        self.cloud = CloudArray() #ring cloud
        self.peg_poses = [] 
        self.ring_poses = [] 
        self.state = [] #FSM state
        self.num_peg = [None, None]
        self.num_ring = [None, None]
        self.manip_id = [] #the name of the manipulator to be used to perform the prescribed action
        self.failure = False
        self.colors = ['red', 'green', 'blue', 'yellow', 'white1', 'white2', 'white3', 'white4']
        self.location = []
        self.request = 0 # request ID --- 1 for fluents, 2 for failure detection
        self.setup_pose = PoseStamped()
        self.pose_meet = PoseStamped()
        self.tf_list = TransformListener()

        #to guarantee proper grasping at transfer
        self.color_center = None 
        self.arm_center = None

        self.ring_cloud_listener = rospy.Subscriber('/ring_points', CloudArray, self.cloud_cb)
        self.poses_pegs_listener = rospy.Subscriber('/all_pegs', PoseArray, self.poses_peg_callback)
        self.poses_rings_listener = rospy.Subscriber('/all_rings', PoseArray, self.poses_ring_callback)
        self.state_listener = rospy.Subscriber('/action_for_sensing', ActionArray, self.state_callback)
        self.sensing_listener = rospy.Subscriber('/ask_for_sensing', Int32, self.on_sensing_request)
        self.setup_pose_listener = rospy.Subscriber('/setup_pose', PoseStamped, self.setup_pose_cb)

        self.obstacle_pub = rospy.Publisher('/obstacles', ObstArray, queue_size=1)
        self.fluent_pub = rospy.Publisher('/context/model', ContextModel, queue_size=1)
        self.target_pose_pub = rospy.Publisher('/target_pose', PoseStampedArray, queue_size=1)
        self.failure_pub = rospy.Publisher('/failure', Bool, queue_size=1)
        # self.fluent_comp_status_pub = rospy.Publisher('/computed_fluents', Bool, queue_size=1)


    def on_sensing_request(self, data):
        self.request = data.data
    
    def setup_pose_cb(self, data):
        self.setup_pose = data
        if self.pose_meet == PoseStamped():
            self.pose_meet = copy.deepcopy(data)
            self.pose_meet.header.frame_id = 'world'
            self.pose_meet.header.stamp = rospy.Time(0.)
            # self.pose_meet.pose.position.x = self.setup_pose.pose.position.x
            # self.pose_meet.pose.position.y = self.setup_pose.pose.position.y
            self.pose_meet.pose.position.z = self.setup_pose.pose.position.z + 0.04

    def state_callback(self, data):
        self.reset()
        self.compute_fluents()
        self.old_actions = []
        self.psm1_start = pm.toMsg(self.psm1.measured_cp(wait=5.))
        self.psm2_start = pm.toMsg(self.psm2.measured_cp(wait=5.))
        for i in range(len(data.action_list)):
            self.state.append(data.action_list[i].action)
            self.location.append(data.action_list[i].object)
            self.manip_id.append(data.action_list[i].robot)
            #store old actions
            a_str = data.action_list[i].action + "(" + data.action_list[i].robot
            if data.action_list[i].object != None:
                a_str += "," + data.action_list[i].object
            if data.action_list[i].color != None:
                a_str += "," + data.action_list[i].color
            a_str += ")"
            self.old_actions.append(a_str)
            #save num current ring and peg
            if self.location[i] == 'ring' or self.location[i] == 'center': 
                if self.bimanual:
                    if self.manip_id[i] == 'psm1':
                        self.num_ring[0] = self.colors.index(data.action_list[i].color.replace(" ", ""))
                    elif self.manip_id[i] == 'psm2':
                        self.num_ring[1] = self.colors.index(data.action_list[i].color.replace(" ", ""))
                else:
                    self.num_ring[0] = self.colors.index(data.action_list[i].color.replace(" ", ""))
                    self.num_ring[1] = self.colors.index(data.action_list[i].color.replace(" ", ""))
            elif self.location[i] == 'peg':
                if self.manip_id[i] == 'psm1':
                    self.num_peg[0] = self.colors.index(data.action_list[i].color.replace(" ", ""))
                elif self.manip_id[i] == 'psm2':
                    self.num_peg[1] = self.colors.index(data.action_list[i].color.replace(" ", ""))
            #     if self.manip_id[i] == 'psm1':
            #         self.num_ring[0] = self.colors.index(data.action_list[i].color.replace(" ", ""))
            #     elif self.manip_id[i] == 'psm2':
            #         self.num_ring[1] = self.colors.index(data.action_list[i].color.replace(" ", ""))
            #TO DISTINGUISH BETWEEN BIMANUAL AND SEQUENTIAL EXECUTION WHEN MOVING TO CENTER
            if self.location[i] == 'center' and self.bimanual:
                self.color_center = data.action_list[i].color
                self.arm_center = self.manip_id[i]
            elif self.bimanual and self.manip_id[i] == self.arm_center:
                self.arm_center = None
                self.color_center = None

            #add information about goal from config file
            try:
                with open(self.path + self.task + ".json", "r") as jsonFile:
                    config = json.load(jsonFile)
                current_config = [c for c in config["actions"] if c["name"] == self.state[i] and self.location[i] == c["object"]][0]
                self.has_goal.append(config["actions"][config["actions"].index(current_config)]["has_goal"])
            except:
                pass

        
    def poses_peg_callback(self, data):    
        peg_poses_st = []    
        for pose in data.poses[:4]:
            pose_st = PoseStamped()
            pose_st.header = data.header
            pose_st.pose = pose
            peg_poses_st.append(pose_st)
        #add white pegs
        dist = abs(peg_poses_st[0].pose.position.y - peg_poses_st[1].pose.position.y)
        pose_st = copy.deepcopy(peg_poses_st[1])
        pose_st.pose.position.y += dist/2.
        peg_poses_st.append(pose_st)
        pose_st = copy.deepcopy(peg_poses_st[3])
        pose_st.pose.position.y -= dist/2.
        peg_poses_st.append(pose_st)
        pose_st = copy.deepcopy(peg_poses_st[0])
        pose_st.pose.position.y += dist/2.
        peg_poses_st.append(pose_st)
        pose_st = copy.deepcopy(peg_poses_st[2])
        pose_st.pose.position.y -= dist/2.
        peg_poses_st.append(pose_st)

        # print(len(data.poses))
        # print(len(peg_poses_st))
        self.peg_poses = copy.deepcopy(peg_poses_st)

    def poses_ring_callback(self, data):    
        ring_poses_st = []  
        for pose in data.poses:
            pose_st = PoseStamped()
            pose_st.header = data.header
            pose_st.pose = pose
            ring_poses_st.append(pose_st)
        self.ring_poses = copy.deepcopy(ring_poses_st)

    def cloud_cb(self, data):
        self.cloud = copy.deepcopy(data)

    def target_move_ring(self, i, num_ring):
        peg_poses = self.peg_poses
        ring_poses = self.ring_poses
        pose = PoseStamped()

        obstacles = ObstArray()
        obstacles.header = Header(frame_id="world", stamp=rospy.Time(0))
        for peg in peg_poses:
            obstacles.coeffs.extend([1,1,2])
            obstacles.axes.extend([self.peg_rad + self.ring_rad, self.peg_rad + self.ring_rad, self.peg_height/2.])
            obstacles.poses.append(Pose(position = Point(x = peg.pose.position.x, y = peg.pose.position.y, z = peg.pose.position.z - self.peg_height/2.)))
        self.obstacle_pub.publish(obstacles)

        #bimanual execution, moving to transfer point
        if self.bimanual and self.arm_center != self.manip_id[i] and self.color_center == self.colors[num_ring]: 
            psm_main_base = PoseStamped()
            psm_main_base.header.stamp = rospy.Time(0)
            if self.manip_id[i] == 'psm2':
                psm_main_base.header.frame_id = 'PSM1_base' + self.dvrk_frame_id
                psm_main_base.pose = pm.toMsg(self.psm1.measured_cp(wait=5.))
                self.tf_list.waitForTransform('world', 'PSM1_base' + self.dvrk_frame_id, time=psm_main_base.header.stamp, timeout=rospy.Duration(secs = 5.))
            elif self.manip_id[i] == 'psm1':
                psm_main_base.header.frame_id = 'PSM2_base' + self.dvrk_frame_id
                psm_main_base.pose = pm.toMsg(self.psm2.measured_cp(wait=5.))
                self.tf_list.waitForTransform('world', 'PSM2_base' + self.dvrk_frame_id, time=psm_main_base.header.stamp, timeout=rospy.Duration(secs = 5.))
            pose = self.tf_list.transformPose('world', psm_main_base)
            #PICK THE RING FROM THE OPPOSITE SIDE OF THE CARRIER
            pose.pose.position.x = ring_poses[num_ring].pose.position.x - (pose.pose.position.x - ring_poses[num_ring].pose.position.x)
            pose.pose.position.y = ring_poses[num_ring].pose.position.y - (pose.pose.position.y - ring_poses[num_ring].pose.position.y) 
            pose.pose.position.z = ring_poses[num_ring].pose.position.z - (pose.pose.position.z - 0.008 - ring_poses[num_ring].pose.position.z) # 0.008 offset to control the tip
            pose.pose.position.z += 0.003 + 0.008
            
            #compute grasping orientation. Two orientations are possible, the closest one to the homing orientation will be picked...
            normal_vector = np.array([ring_poses[num_ring].pose.orientation.x, ring_poses[num_ring].pose.orientation.y, ring_poses[num_ring].pose.orientation.z])
            if np.linalg.norm(normal_vector) > 1e-3:
                normal_vector = copy.deepcopy(normal_vector / np.linalg.norm(normal_vector))
            if self.manip_id[i] == 'psm1':            
                grasp_orients = copy.deepcopy([compute_orient(np.arcsin(normal_vector[1]), math.pi + np.arcsin(normal_vector[0]), math.pi/2. + np.arctan2(ring_poses[num_ring].pose.position.y - pose.pose.position.y , ring_poses[num_ring].pose.position.x - pose.pose.position.x), axes='sxyz'),
                                compute_orient(np.arcsin(normal_vector[1]), math.pi + np.arcsin(normal_vector[0]), -math.pi/2. + np.arctan2(ring_poses[num_ring].pose.position.y - pose.pose.position.y , ring_poses[num_ring].pose.position.x - pose.pose.position.x), axes='sxyz')])
                distance1 = orient_dist(np.array([grasp_orients[0][3], grasp_orients[0][0], grasp_orients[0][1], grasp_orients[0][2]]) , self.psm1_standard_orient)
                distance2 = orient_dist(np.array([self.num_ringgrasp_orients[0][3], grasp_orients[0][0], grasp_orients[0][1], grasp_orients[0][2]]) , self.psm1_standard_orient)
            elif self.manip_id[i] == 'psm2':
                grasp_orients = copy.deepcopy([compute_orient(-np.arcsin(normal_vector[1]), math.pi - np.arcsin(normal_vector[0]), math.pi/2. + np.arctan2(ring_poses[num_ring].pose.position.y - pose.pose.position.y , ring_poses[num_ring].pose.position.x - pose.pose.position.x), axes='sxyz'),
                                compute_orient(-np.arcsin(normal_vector[1]), math.pi - np.arcsin(normal_vector[0]), -math.pi/2. + np.arctan2(ring_poses[num_ring].pose.position.y - pose.pose.position.y , ring_poses[num_ring].pose.position.x - pose.pose.position.x), axes='sxyz')])
                distance1 = orient_dist(np.array([grasp_orients[0][3], grasp_orients[0][0], grasp_orients[0][1], grasp_orients[0][2]]) , self.psm2_standard_orient)
                distance2 = orient_dist(np.array([grasp_orients[0][3], grasp_orients[0][0], grasp_orients[0][1], grasp_orients[0][2]]) , self.psm2_standard_orient)
            if distance1 < distance2:
                grasp_orient = copy.deepcopy(grasp_orients[0])
            else:
                grasp_orient = copy.deepcopy(grasp_orients[1])

            grasp_orient = quaternion.normalize(grasp_orient)
            pose.pose.orientation.x = grasp_orient[0]
            pose.pose.orientation.y = grasp_orient[1]
            pose.pose.orientation.z = grasp_orient[2]
            pose.pose.orientation.w = grasp_orient[3]

            # pose.pose.position.z += 0.008 #sim offset
            pose.header.stamp = rospy.Time(0)
            if self.manip_id[i] == 'psm1':
                self.tf_list.waitForTransform(self.psm1.name()+'_base' + self.dvrk_frame_id, 'world', time=pose.header.stamp, timeout=rospy.Duration(secs = 5.))
                pose = copy.deepcopy(self.tf_list.transformPose(self.psm1.name()+'_base' + self.dvrk_frame_id, pose))
            elif self.manip_id[i] == 'psm2':
                self.tf_list.waitForTransform(self.psm2.name()+'_base' + self.dvrk_frame_id, 'world', time=pose.header.stamp, timeout=rospy.Duration(secs = 5.))
                pose = copy.deepcopy(self.tf_list.transformPose(self.psm2.name()+'_base' + self.dvrk_frame_id, pose))

        #standard motion to ring
        elif peg_poses != [] and len(self.cloud.sets) > num_ring and len(self.cloud.sets[num_ring].poses) > 0:
            cloud = copy.deepcopy(self.cloud.sets[num_ring])
            max_distances = []
            # max_poses = []
            for point in cloud.poses:
                distance = 0.
                #to avoid camera occlusion, avoid positioning in front of the camera and discard points farther from the current arm; need to find a more robust way maybe?
                if abs(point.position.y - ring_poses[num_ring].pose.position.y) > 0.003 and not (self.manip_id[i] == 'psm1' and point.position.y > ring_poses[num_ring].pose.position.y) and not (self.manip_id[i] == 'psm2' and point.position.y < ring_poses[num_ring].pose.position.y): 
                    for peg in peg_poses:
                        distance += ((point.position.x - peg.pose.position.x)**2 + (point.position.y - peg.pose.position.y)**2) 
                max_distances.append(np.sqrt(distance))
            pose.header.frame_id = 'world'
            pose.header.stamp = rospy.Time(0)
            pose.pose.position.x = cloud.poses[np.argmax(np.array(max_distances))].position.x
            pose.pose.position.y = cloud.poses[np.argmax(np.array(max_distances))].position.y
            pose.pose.position.z = cloud.poses[np.argmax(np.array(max_distances))].position.z

            #ORIENTATION COMPUTATION
            #compute grasping orientation. Two orientations are possible, the closest one to the homing orientation will be picked...
            normal_vector = np.array([ring_poses[num_ring].pose.orientation.x, ring_poses[num_ring].pose.orientation.y, ring_poses[num_ring].pose.orientation.z])
            if np.linalg.norm(normal_vector) > 1e-3:
                normal_vector = copy.deepcopy(normal_vector / np.linalg.norm(normal_vector))
            if self.manip_id[i] == 'psm1':
                grasp_orients = copy.deepcopy([compute_orient(np.arcsin(normal_vector[1]), math.pi + np.arcsin(normal_vector[0]), math.pi/2. + np.arctan2(ring_poses[num_ring].pose.position.y - pose.pose.position.y , ring_poses[num_ring].pose.position.x - pose.pose.position.x), axes='sxyz'),
                                compute_orient(np.arcsin(normal_vector[1]), math.pi + np.arcsin(normal_vector[0]), -math.pi/2. + np.arctan2(ring_poses[num_ring].pose.position.y - pose.pose.position.y , ring_poses[num_ring].pose.position.x - pose.pose.position.x), axes='sxyz')])
                distance1 = orient_dist(np.array([grasp_orients[0][3], grasp_orients[0][0], grasp_orients[0][1], grasp_orients[0][2]]) , self.psm1_standard_orient)
                distance2 = orient_dist(np.array([grasp_orients[0][3], grasp_orients[0][0], grasp_orients[0][1], grasp_orients[0][2]]) , self.psm1_standard_orient)
            else:
                grasp_orients = copy.deepcopy([compute_orient(-np.arcsin(normal_vector[1]), math.pi - np.arcsin(normal_vector[0]), math.pi/2. + np.arctan2(ring_poses[num_ring].pose.position.y - pose.pose.position.y , ring_poses[num_ring].pose.position.x - pose.pose.position.x), axes='sxyz'),
                                compute_orient(-np.arcsin(normal_vector[1]), math.pi - np.arcsin(normal_vector[0]), -math.pi/2. + np.arctan2(ring_poses[num_ring].pose.position.y - pose.pose.position.y , ring_poses[num_ring].pose.position.x - pose.pose.position.x), axes='sxyz')])
                distance1 = orient_dist(np.array([grasp_orients[0][3], grasp_orients[0][0], grasp_orients[0][1], grasp_orients[0][2]]) , self.psm2_standard_orient)
                distance2 = orient_dist(np.array([grasp_orients[0][3], grasp_orients[0][0], grasp_orients[0][1], grasp_orients[0][2]]) , self.psm2_standard_orient)
            if distance1 < distance2:
                grasp_orient = copy.deepcopy(grasp_orients[0])
            else:
                grasp_orient = copy.deepcopy(grasp_orients[1])
        
            grasp_orient = quaternion.normalize(grasp_orient)
            pose.pose.orientation.x = grasp_orient[0]
            pose.pose.orientation.y = grasp_orient[1]
            pose.pose.orientation.z = grasp_orient[2]
            pose.pose.orientation.w = grasp_orient[3]

            pose.pose.position.z += 0.008 #sim offset to control the tip
            pose.pose.position.z += 0.003 #to avoid sliding on the plane while moving to the ring
            pose.header.stamp = rospy.Time(0)
            self.tf_list.waitForTransform("world", self.manip_id[i].upper()+'_base' + self.dvrk_frame_id, time=pose.header.stamp, timeout=rospy.Duration(5.))
            pose = copy.deepcopy(self.tf_list.transformPose(self.manip_id[i].upper()+'_base' + self.dvrk_frame_id, pose))

        return pose

    def target_move_peg(self, i, num_ring, num_peg):
        peg_poses = self.peg_poses
        ring_poses = self.ring_poses

        pose_psm = PoseStamped()
        pose_psm.header.frame_id = self.manip_id[i].upper() + '_base' + self.dvrk_frame_id
        if self.manip_id[i] == 'psm1':
            pose_psm.pose = pm.toMsg(self.psm1.measured_cp(wait=5.))
        else:
            pose_psm.pose = pm.toMsg(self.psm2.measured_cp(wait=5.))
        pose_psm = copy.deepcopy(self.tf_list.transformPose('world', pose_psm))
        # rospy.logwarn("NUM ring IS " + str(num_ring))
        if len(self.offset_x) <= i or len(self.offset_y) <= i: #only for the first computation
            self.init_orient.append([pose_psm.pose.orientation.x, pose_psm.pose.orientation.y, pose_psm.pose.orientation.z, pose_psm.pose.orientation.w])  
            self.offset_x.append(pose_psm.pose.position.x - ring_poses[num_ring].pose.position.x)
            self.offset_y.append(pose_psm.pose.position.y - ring_poses[num_ring].pose.position.y)

        pose = PoseStamped()
        # rospy.logwarn("NUM PEG IS " + str(num_peg))
        pose = copy.deepcopy(peg_poses[num_peg])
        pose.pose.position.z += 0.02
        pose.pose.position.x += self.offset_x[i]
        pose.pose.position.y += self.offset_y[i]
        # normal_vector = np.array([ring_poses[num_ring].pose.orientation.x, ring_poses[num_ring].pose.orientation.y, ring_poses[num_ring].pose.orientation.z])
        # if np.linalg.norm(normal_vector) > 1e-3:
        #     normal_vector = copy.deepcopy(normal_vector / np.linalg.norm(normal_vector))
        # rotation = compute_orient(np.arcsin(normal_vector[1]), -np.arcsin(normal_vector[0]), 0., axes='sxyz')
        # grasp_orient = quaternion_multiply(rotation, self.init_orient[i])

        # grasp_orient = quaternion.normalize(grasp_orient)
        # pose.pose.orientation.x = grasp_orient[0]
        # pose.pose.orientation.y = grasp_orient[1]
        # pose.pose.orientation.z = grasp_orient[2]
        # pose.pose.orientation.w = grasp_orient[3]
        pose.pose.orientation = pose_psm.pose.orientation

        pose.header.stamp = rospy.Time(0)
        self.tf_list.waitForTransform("world", self.manip_id[i].upper()+'_base' + self.dvrk_frame_id, time=pose.header.stamp, timeout=rospy.Duration(5.))
        pose = copy.deepcopy(self.tf_list.transformPose(self.manip_id[i].upper()+'_base' + self.dvrk_frame_id, pose))   
        
        return pose

    def target_move_center(self, i, num_ring):
        ring_poses = self.ring_poses
        poses = []

        pose_base = PoseStamped()
        pose_base.header.frame_id = self.manip_id[i].upper()+'_base' + self.dvrk_frame_id
        pose_base.header.stamp = rospy.Time(0.)
        if self.manip_id[i] == 'psm1':
            pose_base.pose = pm.toMsg(self.psm1.measured_cp(wait=5.))
        elif self.manip_id[i] == 'psm2':
            pose_base.pose = pm.toMsg(self.psm2.measured_cp(wait=5.))
        self.tf_list.waitForTransform(self.manip_id[i].upper()+'_base' + self.dvrk_frame_id, 'world', time=pose_base.header.stamp, timeout=rospy.Duration(secs = 5.))
        pose_base_w = copy.deepcopy(self.tf_list.transformPose('world', pose_base))

        self.pose_meet.pose.orientation = copy.deepcopy(pose_base_w.pose.orientation)
        self.tf_list.waitForTransform(self.manip_id[i].upper()+'_base' + self.dvrk_frame_id, 'world', time=self.pose_meet.header.stamp, timeout=rospy.Duration(secs = 5.))
        pose = copy.deepcopy(self.tf_list.transformPose(self.manip_id[i].upper()+'_base' + self.dvrk_frame_id, self.pose_meet))  

        poses.append(pose)

        if not self.bimanual:
            pose = PoseStamped()
            pose = self.tf_list.transformPose('world', pose_base_w)
            if self.shift_center == None: #to solve slow update of ring pose from camera
                self.shift_center = [pose.pose.position.x - ring_poses[num_ring].pose.position.x, pose.pose.position.y - ring_poses[num_ring].pose.position.y]
                # rospy.logwarn(self.shift_center)
            #PICK THE RING FROM THE OPPOSITE SIDE OF THE CARRIER
            pose.pose.position.x = self.pose_meet.pose.position.x - 2*self.shift_center[0]
            pose.pose.position.y = self.pose_meet.pose.position.y - 2*self.shift_center[1] 
            pose.pose.position.z = self.pose_meet.pose.position.z
            
            #compute grasping orientation. Two orientations are possible, the closest one to the homing orientation will be picked...
            normal_vector = np.array([ring_poses[num_ring].pose.orientation.x, ring_poses[num_ring].pose.orientation.y, ring_poses[num_ring].pose.orientation.z])
            if np.linalg.norm(normal_vector) > 1e-3:
                normal_vector = copy.deepcopy(normal_vector / np.linalg.norm(normal_vector))
            if self.manip_id[i] == 'psm2':            
                grasp_orients = copy.deepcopy([compute_orient(np.arcsin(normal_vector[1]), math.pi + np.arcsin(normal_vector[0]), math.pi/2. + np.arctan2(ring_poses[num_ring].pose.position.y - pose.pose.position.y , ring_poses[num_ring].pose.position.x - pose.pose.position.x), axes='sxyz'),
                                compute_orient(np.arcsin(normal_vector[1]), math.pi + np.arcsin(normal_vector[0]), -math.pi/2. + np.arctan2(ring_poses[num_ring].pose.position.y - pose.pose.position.y , ring_poses[num_ring].pose.position.x - pose.pose.position.x), axes='sxyz')])
                distance1 = orient_dist(np.array([grasp_orients[0][3], grasp_orients[0][0], grasp_orients[0][1], grasp_orients[0][2]]) , self.psm1_standard_orient)
                distance2 = orient_dist(np.array([grasp_orients[0][3], grasp_orients[0][0], grasp_orients[0][1], grasp_orients[0][2]]) , self.psm1_standard_orient)
            else:
                grasp_orients = copy.deepcopy([compute_orient(-np.arcsin(normal_vector[1]), math.pi - np.arcsin(normal_vector[0]), math.pi/2. + np.arctan2(ring_poses[num_ring].pose.position.y - pose.pose.position.y , ring_poses[num_ring].pose.position.x - pose.pose.position.x), axes='sxyz'),
                                compute_orient(-np.arcsin(normal_vector[1]), math.pi - np.arcsin(normal_vector[0]), -math.pi/2. + np.arctan2(ring_poses[num_ring].pose.position.y - pose.pose.position.y , ring_poses[num_ring].pose.position.x - pose.pose.position.x), axes='sxyz')])
                distance1 = orient_dist(np.array([grasp_orients[0][3], grasp_orients[0][0], grasp_orients[0][1], grasp_orients[0][2]]) , self.psm2_standard_orient)
                distance2 = orient_dist(np.array([grasp_orients[0][3], grasp_orients[0][0], grasp_orients[0][1], grasp_orients[0][2]]) , self.psm2_standard_orient)
            if distance1 < distance2:
                grasp_orient = copy.deepcopy(grasp_orients[0])
            else:
                grasp_orient = copy.deepcopy(grasp_orients[1])
            
            grasp_orient = quaternion.normalize(grasp_orient)
            pose.pose.orientation.x = grasp_orient[0]
            pose.pose.orientation.y = grasp_orient[1]
            pose.pose.orientation.z = grasp_orient[2]
            pose.pose.orientation.w = grasp_orient[3]

            # pose.pose.position.z += 0.008 #sim offset
            pose.header.stamp = rospy.Time(0)
            if self.manip_id[i] == 'psm1':
                self.tf_list.waitForTransform(self.psm2.name()+'_base' + self.dvrk_frame_id, 'world', time=pose.header.stamp, timeout=rospy.Duration(secs = 5.))
                pose = copy.deepcopy(self.tf_list.transformPose(self.psm2.name()+'_base' + self.dvrk_frame_id, pose))
            else:
                self.tf_list.waitForTransform(self.psm1.name()+'_base' + self.dvrk_frame_id, 'world', time=pose.header.stamp, timeout=rospy.Duration(secs = 5.))
                pose = copy.deepcopy(self.tf_list.transformPose(self.psm1.name()+'_base' + self.dvrk_frame_id, pose))

            poses.append(pose)

        return poses

    
    def target_custom(self, idx):
        if self.manip_id[idx] == "psm1":
            psm = self.psm1
            psm_start = self.psm1_start
        else:
            psm = self.psm2
            psm_start = self.psm2_start

        final_position = np.array(self.has_goal[idx][0]) + np.array([psm_start.position.x, psm_start.position.y, psm_start.position.z])
        final_orientation = quaternion_multiply(self.has_goal[idx][1], [psm_start.orientation.w, psm_start.orientation.x, psm_start.orientation.y, psm_start.orientation.z])
        return PoseStamped(header=Header(frame_id=psm.name().upper()+"_base", stamp = rospy.Time(0.)), pose=Pose(position=Point(x=final_position[0], y=final_position[1], z=final_position[2]), orientation=Quaternion(x=final_orientation[1], y=final_orientation[2], z=final_orientation[3], w=final_orientation[0])))


    def compute_target(self):  
        target_poses = PoseStampedArray()
        peg_poses = self.peg_poses

        for i in range(len(self.state)):
            num_ring = None
            if self.manip_id[i] == 'psm1':
                num_ring = self.num_ring[0]
                num_peg = self.num_peg[0]
            elif self.manip_id[i] == 'psm2':
                num_ring = self.num_ring[1]
                num_peg = self.num_peg[1]
            while len(self.has_goal) <= i: #waiting for config update
                # rospy.logwarn("WAITING FOR HAS_GOAL UPDATE")
                pass
            if type(self.has_goal[i]) == bool and not self.has_goal[i]:
                if self.location[i] == 'ring' and num_ring != None:
                    target_poses.poses.append(self.target_move_ring(i, num_ring))
                elif self.location[i] == 'peg' and peg_poses[num_peg].header.frame_id != '':
                    obstacles = ObstArray()
                    obstacles.header = Header(frame_id="world", stamp=rospy.Time(0))
                    for peg in peg_poses:
                        obstacles.coeffs.extend([1,1,2])
                        obstacles.axes.extend([self.peg_rad + self.ring_rad, self.peg_rad + self.ring_rad, self.peg_height/2.])
                        obstacles.poses.append(Pose(position = Point(x = peg.pose.position.x, y = peg.pose.position.y, z = peg.pose.position.z - self.peg_height/2.)))
                    self.obstacle_pub.publish(obstacles)
                    target_poses.poses.append(self.target_move_peg(i, num_ring, num_peg))
                elif self.location[i] == 'center' and num_ring != None:
                    obstacles = ObstArray()
                    obstacles.header = Header(frame_id="world", stamp=rospy.Time(0))
                    for peg in peg_poses:
                        obstacles.coeffs.extend([1,1,2])
                        obstacles.axes.extend([self.peg_rad + self.ring_rad, self.peg_rad + self.ring_rad, self.peg_height/2.])
                        obstacles.poses.append(Pose(position = Point(x = peg.pose.position.x, y = peg.pose.position.y, z = peg.pose.position.z - self.peg_height/2.)))
                    self.obstacle_pub.publish(obstacles)
                    poses = self.target_move_center(i, num_ring)
                    for pose in poses:
                        target_poses.poses.append(pose)
            elif len(self.has_goal[i]) > 0: #a new learned action with pre-defined goal
                target_poses.poses.append(self.target_custom(i))

        self.target_pose_pub.publish(target_poses)

    def compute_fluents(self):
        rospy.sleep(2.)
        distances = []
        fluents = copy.deepcopy(self.fixed_fluents)

        #to force placed on same-colored peg (camera issue)
        if len(self.old_actions) > 0 and len(self.old_fluents) > 0: #need to evaluate last action and context
            color_ring = ""
            last_release = [el for el in self.old_actions if "release" in el]
            for el in last_release:
                agent = re.split('[(,)]', el)[1]
                at_peg = [f for f in self.old_fluents if "at" in f and "peg" in f and agent in f]
                at_ring = [f for f in self.old_fluents if "at" in f and "ring" in f and agent in f]
                closed = [f for f in self.old_fluents if "closed" in f and agent in f]
                if len(closed) > 0 and len(at_peg) > 0 and len(at_ring) > 0:
                    color_ring = re.split('[(,)]', at_ring[0])[3]
                    color_peg = re.split('[(,)]', at_peg[0])[3]
                    fluents.append("placed(ring,"+color_ring+",peg,"+color_peg+")")
                    self.fixed_fluents.append("placed(ring,"+color_ring+",peg,"+color_peg+")")
                # for j in range(len(self.old_action_psm[:-2])):
                #     last_agent = [el1 for el1 in self.old_action_psm[-2-j-1] if agent in el1]
                #     if len(last_agent) > 0:
                #         if "move" in last_agent[0] and "peg" in last_agent[0]:
                #             color_peg = last_agent[0].split("_")[3]
                #             for k in range(len(self.old_action_psm[:-2-j])):
                #                 last_action = [el1 for el1 in self.old_action_psm[-2-j-k-1] if agent in el1]
                #                 if len(last_action) > 0 and "ring" in last_action[0]:
                #                     color_ring = last_action[0].split("_")[3]
                #                     fluents.append("placed(ring,"+color_ring+",peg,"+color_peg+")")
                #                     self.fixed_fluents.append("placed(ring,"+color_ring+",peg,"+color_peg+")")
                #                     break
                #             if color_ring != "":
                #                 break
                #         else:
                #             break

        #to choose region of operation of the two arms
        setup_center_position = self.setup_pose.pose.position.y #y coords, SIM VALUES!

        #compute fluents in world frame
        psm1_base = PoseStamped()
        psm1_base.header.frame_id = self.psm1.name()+'_base'
        psm1_base.pose = pm.toMsg(self.psm1.measured_cp(wait=5.))
        psm1_base.header.stamp = rospy.Time.now()
        self.tf_list.waitForTransform('world', self.psm1.name()+'_base', time=psm1_base.header.stamp, timeout=rospy.Duration(secs = 5.))
        psm1_pose = self.tf_list.transformPose('world', psm1_base)
        psm2_base = PoseStamped()
        psm2_base.header.frame_id = self.psm2.name()+'_base'
        psm2_base.pose = pm.toMsg(self.psm2.measured_cp(wait=5.))
        psm2_base.header.stamp = rospy.Time.now()
        self.tf_list.waitForTransform('world', self.psm2.name()+'_base', time=psm2_base.header.stamp, timeout=rospy.Duration(secs = 5.))
        psm2_pose = self.tf_list.transformPose('world', psm2_base)

        #gripper status
        if abs(self.psm1.jaw.measured_js(wait=5.)[0]) < math.pi/18.:
            fluents.append('closed_gripper(psm1)')
        if abs(self.psm2.jaw.measured_js(wait=5.)[0]) < math.pi/18.:
            fluents.append('closed_gripper(psm2)')

        #distances from center
        distance1 = np.linalg.norm([self.setup_pose.pose.position.x - psm1_pose.pose.position.x, 
                                    self.setup_pose.pose.position.y - psm1_pose.pose.position.y])
        distance2 = np.linalg.norm([self.setup_pose.pose.position.x - psm2_pose.pose.position.x, 
                                    self.setup_pose.pose.position.y - psm2_pose.pose.position.y])

        if distance1 < 0.005: 
            fluents.append('at(psm1,center)')

        if distance2 < 0.005:
            fluents.append('at(psm2,center)')


        ring_poses = self.ring_poses
        peg_poses = self.peg_poses
        distance1_prev = 100.
        distance2_prev = 100.
        stop_psm1 = False
        stop_psm2 = False
        placed = dict()
        for peg in peg_poses:
            placed[self.colors[peg_poses.index(peg)]] = []
            #peg reachability
            if not (peg.pose.orientation.x == 0. and peg.pose.orientation.y == 0. and peg.pose.orientation.z == 0. and peg.pose.orientation.w == 0.): #peg visible
                if peg.pose.position.y < setup_center_position:
                    fluents.append('reachable(psm1,peg,' + self.colors[peg_poses.index(peg)] + ')')
                else:
                    # print(peg_poses)
                    # print(len(peg_poses))
                    fluents.append('reachable(psm2,peg,' + self.colors[peg_poses.index(peg)] + ')')
                
                #distances from pegs
                distance1 = np.linalg.norm([peg.pose.position.x - psm1_pose.pose.position.x, 
                                            peg.pose.position.y - psm1_pose.pose.position.y])
                distance2 = np.linalg.norm([peg.pose.position.x - psm2_pose.pose.position.x, 
                                            peg.pose.position.y - psm2_pose.pose.position.y])

                if distance1 < 0.015 and psm1_pose.pose.position.z - 0.002 > peg.pose.position.z and psm1_pose.pose.position.z - peg.pose.position.z < 0.025 and not stop_psm1: 
                    fluents.append('at(psm1,peg,' + self.colors[peg_poses.index(peg)] + ')')
                    stop_psm1 = True
                if distance2 < 0.015 and psm2_pose.pose.position.z - 0.002 > peg.pose.position.z and psm2_pose.pose.position.z - peg.pose.position.z < 0.025 and not stop_psm2:
                    fluents.append('at(psm2,peg,' + self.colors[peg_poses.index(peg)] + ')')
                    stop_psm2 = True  

                for ring in ring_poses:
                    if not (ring.pose.orientation.x == 0. and ring.pose.orientation.y == 0. and ring.pose.orientation.z == 0. and ring.pose.orientation.w == 0.): #ring visible
                        #check for rings on pegs
                        dist = np.linalg.norm([ring.pose.position.x - peg.pose.position.x, 
                                            ring.pose.position.y - peg.pose.position.y])
                        if dist < 0.015 and ring.pose.position.z < self.setup_pose.pose.position.z + 0.01: #less than ring's radius
                            placed[self.colors[peg_poses.index(peg)]].append(('placed(ring,' + self.colors[ring_poses.index(ring)] + ',peg,' + self.colors[peg_poses.index(peg)] + ')', dist))

        #filter to have only one ring per peg and one peg per ring
        placed_list = []
        final_placed = []
        for k in placed.keys():
            if len(placed[k]) > 0:
                placed[k].sort(key = lambda pl: pl[-1])
                placed_list.append(placed[k][0])
        # rospy.logwarn(placed_list)
        for c in self.colors:
            c_list = [el for el in placed_list if "ring,"+c in el[0]]
            if len(c_list) > 0:
                c_list.sort(key = lambda item : item[-1])
                final_placed.append(c_list[0][0])
        # rospy.logwarn(final_placed)

        fluents += final_placed   

        distance2_prev = 100.
        distance1_prev = 100.
        stop_psm2 = False
        stop_psm1 = False
        for ring in ring_poses:
            if not (ring.pose.orientation.x == 0. and ring.pose.orientation.y == 0. and ring.pose.orientation.z == 0. and ring.pose.orientation.w == 0.): #ring visible
                #distances from rings
                distance1 = np.linalg.norm([ring.pose.position.x - psm1_pose.pose.position.x, 
                                            ring.pose.position.y - psm1_pose.pose.position.y,
                                            ring.pose.position.z - psm1_pose.pose.position.z])
                distance2 = np.linalg.norm([ring.pose.position.x - psm2_pose.pose.position.x, 
                                            ring.pose.position.y - psm2_pose.pose.position.y,
                                            ring.pose.position.z - psm2_pose.pose.position.z])
                distances.append(['psm1', self.colors[ring_poses.index(ring)], distance1])
                distances.append(['psm2', self.colors[ring_poses.index(ring)], distance2])
                
                #ring reachability
                if ring.pose.position.y < setup_center_position:
                    fluents.append('reachable(psm1,ring,' + self.colors[ring_poses.index(ring)] + ')')
                else:
                    fluents.append('reachable(psm2,ring,' + self.colors[ring_poses.index(ring)] + ')')
                
                #check for already grasped / reached rings
                if distance1 < 0.015 and not stop_psm1:
                    fluents.append('at(psm1,ring,' + self.colors[ring_poses.index(ring)] + ')')
                    stop_psm1 = True
                if distance2 < 0.015 and not stop_psm2:
                    fluents.append('at(psm2,ring,' + self.colors[ring_poses.index(ring)] + ')')
                    stop_psm2 = True                

        self.old_fluents = copy.deepcopy(fluents)
        if self.request == 1:
            msg = ContextModel()
            for i in range(len(fluents)):
                msg.atoms.append(fluents[i])
            self.fluent_pub.publish(msg)

            self.request = 0
















    def reset(self):
        self.offset_x = []
        self.offset_y = []
        self.init_orient = []
        # self.num_peg = []
        self.manip_id = []
        self.location = []
        self.state = []
        self.has_goal = []
        self.failure = False

    def check_failure(self):
        peg_poses = self.peg_poses
        ring_poses = self.ring_poses

        for i in range(len(self.state)):
            num_ring = None
            if self.manip_id[i] == 'psm1':
                num_ring = self.num_ring[0]
                num_peg = self.num_peg[0]
            elif self.manip_id[i] == 'psm2':
                num_ring = self.num_ring[1]
                num_peg = self.num_peg[1]
            if num_ring is not None:
                #ring no more detectable
                init_time = rospy.Time.now()
                while  self.request != 0 and ring_poses[num_ring].pose.orientation.x == 0 and ring_poses[num_ring].pose.orientation.y == 0 and ring_poses[num_ring].pose.orientation.z == 0 and ring_poses[num_ring].pose.orientation.w == 0:
                    if rospy.Time.now() - init_time > rospy.Duration(5.):
                        rospy.loginfo('RING LOST, RE-PLANNING NEEDED')
                        self.failure = True
                        self.request = 0
                        return
                
                if (self.location[i] == 'peg' or self.location[i] == 'center'):
                    if self.manip_id[i] == 'psm1':
                        psm = self.psm1
                    elif self.manip_id[i] == 'psm2':
                        psm = self.psm2
                    psm_base = PoseStamped()
                    psm_base.header.frame_id = self.manip_id[i].upper()+'_base' + self.dvrk_frame_id
                    psm_base.pose = pm.toMsg(psm.measured_cp(wait=5.))                
                    psm_pose = copy.deepcopy(self.tf_list.transformPose('world', psm_base))            

                    # CHECK CONDITION OF THE TARGET PEG AND OF THE SAME-COLORED PEG
                    if self.location[i] == 'peg':
                        peg1 = peg_poses[num_peg]
                        peg2 = peg_poses[num_ring]
                        for ring in ring_poses:
                            if ring != ring_poses[num_ring]:
                                if np.linalg.norm([ring.pose.position.y - peg1.pose.position.y,
                                                    ring.pose.position.x - peg1.pose.position.x]) < 0.005:
                                    rospy.loginfo('TARGET PEG IS OCCUPIED, RE-PLANNING NEEDED')
                                    self.request = 0
                                    self.failure = True
                                    return

                        if num_ring != num_peg: #carrying to a white peg
                            peg_occupied = False
                            for ring in ring_poses:
                                if ring != ring_poses[num_ring]:
                                    if np.linalg.norm([ring.pose.position.y - peg2.pose.position.y,
                                                        ring.pose.position.x - peg2.pose.position.x]) < 0.005:
                                        peg_occupied = True
                                        break
                            if not peg_occupied:
                                rospy.loginfo('SAME-COLORED PEG HAS BECOME FREE, RE-PLANNING NEEDED')
                                self.request = 0
                                self.failure = True
                                return

                    init_time = rospy.Time.now()
                    while np.linalg.norm([ring_poses[num_ring].pose.position.x - psm_pose.pose.position.x,
                                        ring_poses[num_ring].pose.position.y - psm_pose.pose.position.y,
                                        ring_poses[num_ring].pose.position.z - psm_pose.pose.position.z]) > 0.015 and self.request != 0: #hand tuned for sim
                        if rospy.Time.now() - init_time > rospy.Duration(secs = 5.):
                            rospy.loginfo('RING FALLEN, RE-PLANNING NEEDED')
                            self.request = 0
                            self.failure = True
                            return
                
                elif (self.location[i] == 'ring' and self.state[i] == 'move') and not (self.arm_center != self.manip_id[i] and self.color_center == self.colors[num_ring]) and self.request != 0: #CHECK THE RING REMAINS IN THE REACHABLE ZONE
                    setup_center_position = self.setup_pose.pose.position.y #SIM VALUE
                    if (ring_poses[num_ring].pose.position.y > setup_center_position and self.manip_id[i] == 'psm1') or (ring_poses[num_ring].pose.position.y < setup_center_position and self.manip_id[i] == 'psm2'):
                        rospy.loginfo('RING NO MORE REACHABLE BY THE CURRENT ARM, RE-PLANNING NEEDED')
                        self.request = 0
                        self.failure = True

            







def main():

    # rospy.init_node('sensing_node')
    ral = crtk.ral('sensing_node')
    sa = Situation_awareness(ral)
    rospy.sleep(1.)

    #save homing orientation for more "natural" motion
    psm1_base = PoseStamped()
    psm1_base.header.frame_id = sa.psm1.name()+'_base' + sa.dvrk_frame_id
    psm1_base.pose = pm.toMsg(sa.psm1.measured_cp(wait=5.))
    psm1_base.header.stamp = rospy.Time(0)
    sa.tf_list.waitForTransform('world', sa.psm1.name()+'_base' + sa.dvrk_frame_id, time=psm1_base.header.stamp, timeout=rospy.Duration(secs = 5.))
    psm1_pose = sa.tf_list.transformPose('world', psm1_base)
    sa.psm1_standard_orient = np.array([psm1_pose.pose.orientation.w, psm1_pose.pose.orientation.x, psm1_pose.pose.orientation.y, psm1_pose.pose.orientation.z]) #DMP order
    psm2_base = PoseStamped()
    psm2_base.header.frame_id = sa.psm2.name()+'_base' + sa.dvrk_frame_id
    psm2_base.pose = pm.toMsg(sa.psm2.measured_cp(wait=5.))
    psm2_base.header.stamp = rospy.Time(0)
    sa.tf_list.waitForTransform('world', sa.psm2.name()+'_base' + sa.dvrk_frame_id, time=psm2_base.header.stamp, timeout=rospy.Duration(secs = 5.))
    psm2_pose = sa.tf_list.transformPose('world', psm2_base)
    sa.psm2_standard_orient = np.array([psm2_pose.pose.orientation.w, psm2_pose.pose.orientation.x, psm2_pose.pose.orientation.y, psm2_pose.pose.orientation.z]) #DMP order

    while not rospy.is_shutdown():
        if sa.request == 1:
            sa.compute_fluents()
        elif sa.request == 2:
            try:
                sa.compute_target()
            except:
                pass
            # ONLINE RE-PLANNING
            # sa.check_failure()
            # if sa.failure:
            #     sa.failure_pub.publish(Bool(True))
        #to overcome slow ring pose update from camera for transfer
        if "center" not in sa.location:
            sa.shift_center = None

    ral.spin()







if __name__ == '__main__':
    main()




from typing_extensions import final
import numpy as np
import math
import os
import roslib
import rospkg
import rospy
import actionlib
import sys
import copy
from dvrk_task_msgs.msg import ObstArray, ActionRequestFiner, ActionArray, PoseStampedArray, BoolArray
from std_msgs.msg import Header, Int32, Float64, Bool, String, Int16MultiArray as IntArray
from geometry_msgs.msg import PoseArray, PoseStamped, Pose
from tf import TransformListener
from tf_conversions import posemath as pm
from tf.transformations import quaternion_from_euler as compute_orient
from dmp_poses import DMPs_pose
import quaternion
from pyquaternion import Quaternion
from obstacle import Obstacle_Dynamic as dyn_ob, Obstacle_Static as st_ob
import dvrk
import PyKDL
import json
import crtk














#MOTION CONTROL
class motion_manager(object):
    def __init__(self,ral):
        rospack = rospkg.RosPack()
        self.path = rospack.get_path("robot_control") + '/config/'
        self.task = "pegring"
        with open(self.path + self.task + ".json") as f:
            actions = json.load(f)["actions"]
        self.bimanual = [a for a in actions if a["policy"] == "move_center"][0]["n_arms"] < 2
        self.dvrk_frame_id = rospy.get_param("dvrk_frame_id")


        self.psm1 = dvrk.psm(ral, "PSM1")
        self.psm2 = dvrk.psm(ral, "PSM2")
        self.dmp_weights = []
        self.agents = []
        self.n_arms = [] #for dual-arm actions
        self.policies = []
        self.policy_types = []
        self.fixed_obst = True
        self.obstacles = ObstArray()
        self.target_pose = PoseStampedArray() #grasping pose
        self.state = [] #ASP action
        self.manip_id = [] #the name of the manipulator to be used to perform the prescribed action
        self.color = []
        self.failure = False
        self.fluents = []
        self.location = []
        self.from_user = []
        self.got_action = False
        self.traj = [[], []] #for imitation of new trajectories
        self.new_action = [] # to keep track of unexperienced actions requiring learning a new policy
        self.target_pose_listener = rospy.Subscriber('/target_pose', PoseStampedArray, self.target_pose_callback)
        self.state_listener = rospy.Subscriber('/actions/request', ActionArray, self.state_callback)
        self.failure_listener = rospy.Subscriber('/failure', Bool, self.failure_cb)
        self.tf_list = TransformListener()
        self.sensing_pub = rospy.Publisher('/ask_for_sensing', Int32, queue_size=1)
        # self.flag_pub = rospy.Publisher('/flag_topic', Bool, queue_size=1)
        self.obst_sub = rospy.Subscriber('/obstacles', ObstArray, self.obstacles_callback)
        self.psm1_sub = rospy.Subscriber("dvrk/PSM1/position_cartesian_current", PoseStamped, self.record_trajectory_psm1)
        self.psm1_sub = rospy.Subscriber("dvrk/PSM2/position_cartesian_current", PoseStamped, self.record_trajectory_psm2)

        #PEG-RING SPECIFIC
        self.colors = ['red', 'green', 'blue', 'yellow', 'white1', 'white2', 'white3', 'white4']
        self.setup_pose = PoseStamped()
        self.setup_pose_sub = rospy.Subscriber('/setup_pose', PoseStamped, self.setup_pose_cb)
        self.custom_policies = { 
                                 "extract" : self.extract,
                                 "grasp" : self.grasp,
                                 "release" : self.release,
                                 "move_ring" : self.move_ring,
                                 "move_center" : self.move_center,
                                 "move_peg" : self.move_peg,
                                 "recovery" : self.recovery
                               } 

        #FAKE GRASP FOR PEG-RING IN V-REP
        self.num_ring = [0,0] #for V-REP for peg ring
        self.num_rings = IntArray(data=[0,0]) 
        self.num_arms = IntArray(data=[0,0])
        self.current_color_pub = rospy.Publisher('/current_color', IntArray, queue_size=1)
        self.current_arm_pub = rospy.Publisher('/current_psm', IntArray, queue_size=1)

    def failure_cb(self, data):
        self.failure = data.data

    def target_pose_callback(self, data):
        self.target_pose = copy.deepcopy(data)

    def setup_pose_cb(self, data):
        self.setup_pose = copy.deepcopy(data)

    def obstacles_callback(self, data):  
        self.obstacles = data   

    def state_callback(self, data):
        for i in range(len(data.action_list)):
            self.new_action.append(data.action_list[i].new)
            self.from_user.append(data.action_list[i].from_user)
            self.location.append(data.action_list[i].object)
            self.manip_id.append(data.action_list[i].robot)
            self.color.append(data.action_list[i].color)
            #FOR V-REP FOR PEG-RING
            if self.location[i] == 'ring' or self.location[i] == 'center': 
                if self.manip_id[i] == 'psm1':
                    self.num_ring[0] = self.colors.index(data.action_list[i].color)
                elif self.manip_id[i] == 'psm2':
                    self.num_ring[1] = self.colors.index(data.action_list[i].color)
            #FOR V-REP FOR PEG-RING
            self.state.append(data.action_list[i].action)
        self.got_action = True


    def record_trajectory_psm1(self, data):
        pose = data.pose
        self.traj[0].append([pose.position.x, pose.position.y, pose.position.z, pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z])
        


    def record_trajectory_psm2(self, data):
        pose = data.pose
        self.traj[1].append([pose.position.x, pose.position.y, pose.position.z, pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z])


    def extract(self, psm_name):
        height = 0.035
        if psm_name == "psm1":
            psm = self.psm1
        elif psm_name == "psm2":
            psm = self.psm2
        start = psm.measured_cp(wait=5.)
        start.p[2] += height
        # psm.dmove(PyKDL.Vector(0., 0., height), blocking = True)
        psm.move_cp(start).wait(is_busy=True)

    def grasp(self, psm_name):
        if psm_name == "psm1":
            psm = self.psm1
        elif psm_name == "psm2":
            psm = self.psm2
        psm.jaw.close().wait()

        #V-REP FOR PEG-RING 
        self.num_arms.data[int(psm_name[-1])-1] = 1 
        self.num_rings.data[int(psm_name[-1])-1] = self.num_ring[int(psm_name[-1])-1] + 1
        #V-REP FOR PEG-RING

    def release(self, psm_name):
        height = 0.018
        width=math.pi/4.
        if psm_name == "psm1":
            psm = self.psm1
        elif psm_name == "psm2":
            psm = self.psm2
        psm.jaw.open(width).wait()

        #indexing for V-REP for peg-ring
        self.num_arms.data[int(psm_name[-1])-1] = 0 

    
    def move_ring(self, psm_name):
        height=-0.003
        if psm_name == "psm1":
            psm = self.psm1
        elif psm_name == "psm2":
            psm = self.psm2
        start = psm.measured_cp(wait=5.)
        start.p[2] += height
        # psm.dmove(PyKDL.Vector(0., 0., height), blocking = True)
        psm.move_cp(start).wait(is_busy=True)
        # psm.dmove(PyKDL.Vector(0., 0., height), blocking=True) #go down to the ring
    
    def move_center(self, psm_name):
        if not self.bimanual:
            height=-0.003
            if psm_name == "psm1":
                psm = self.psm2
            elif psm_name == "psm2":
                psm = self.psm1
            # psm.dmove(PyKDL.Vector(0., 0., height), blocking=True) #go down to the ring
            start = psm.measured_cp(wait=5.)
            start.p[2] += height
            # psm.dmove(PyKDL.Vector(0., 0., height), blocking = True)
            psm.move_cp(start).wait(is_busy=True)
    
    def move_peg(self, psm_name):
        # height=-0.018
        # if psm_name == "psm1":
        #     psm = self.psm1
        # elif psm_name == "psm2":
        #     psm = self.psm2
        # psm.dmove(PyKDL.Vector(0., 0., height)) #go down to the ring
        pass

    def dmp_init(self, dmp_indices):
        agents = [self.agents[idx] for idx in dmp_indices]
        n_arms = [self.n_arms[idx] for idx in dmp_indices]
        dmps = []
        psms = []
        for i in range(len(agents)):
            for j in range(n_arms[i]):
                weights = self.dmp_weights[i][j]
                if weights == []:
                    w_cart = np.zeros((3,41))
                    w_quat = np.zeros((3,41))
                else:
                    w_cart = np.array(weights[0])
                    w_quat = np.array(weights[1])
                dmps.append(DMPs_pose(n_bfs = np.shape(w_cart)[1]-1))
                dmps[-1].w_cart = w_cart
                dmps[-1].w_quat = w_quat
                dmps[-1].tol = 2e-2

                if j == 0:
                    if agents[i] == "psm1":
                        psm = self.psm1
                    elif agents[i] == "psm2":
                        psm = self.psm2
                #FOR DUAL ARM EXECUTION, THE SECOND ARM IS DIFFERENT FROM THE AGENT
                else:
                    if agents[i] == "psm1":
                        psm = self.psm2
                    elif agents[i] == "psm2":
                        psm = self.psm1

                pose = pm.toMsg(psm.measured_cp(wait=5.))
                dmps[-1].x_0 = np.array([pose.position.x, pose.position.y, pose.position.z])
                dmps[-1].q_0 = np.array([pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z])
                dmps[-1].t = 0

                dmps[-1].reset_state()
                psms.append(psm)
        
        return np.array(dmps), psms

    def dmp_execute(self, dmp_indices):
        # self.target_pose = PoseStampedArray()
        dmps, psms = self.dmp_init(dmp_indices)
        y_track_s = []
        dy_track_s = []
        q_track_s = []
        stops = []
        for i in range(len(dmps)):
            y_track_s.append(dmps[i].x_0)
            q_track_s.append(dmps[i].q_0) 
            dy_track_s.append(np.zeros(np.shape(dmps[i].x_0)))
            stops.append(False)    

        #NOTIFY SENSING FOR ANOMALIES
        # self.sensing_pub.publish(Int32(2))
        # rospy.sleep(2.)
        #NOTIFY SENSING FOR ANOMALIES

        stop = 0
        rate = rospy.Rate(20.)
        target = Pose()
        self.target_pose = PoseStampedArray()
        dmp_obs = []

        #in simulation, to emulate manual motion
        height = 0.015
        num_steps = 20
        count_steps = 0

        while stop == 0 and not self.failure:
            stop = np.prod(stops)
            for i in range(len(dmps)):
                if not self.new_action[i]: #known action
                    self.sensing_pub.publish(Int32(2))
                    if (dmps[i].t == 0):
                        dmps[i].first = True
                    else:
                        dmps[i].first = False

                    if len(self.target_pose.poses) > i and self.target_pose.poses[i].header.frame_id != '':
                        target_pose = copy.deepcopy(self.target_pose.poses[i].pose)
                        dmps[i].x_goal = copy.deepcopy(np.array([target_pose.position.x, target_pose.position.y, target_pose.position.z]))
                        dmps[i].q_goal = copy.deepcopy(np.array([target_pose.orientation.w, target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z]))
                        stops[i] = ((np.nan_to_num(np.linalg.norm(y_track_s[i] - dmps[i].x_goal) / (np.linalg.norm(dmps[i].x_goal - dmps[i].x_0) + 1e-6)) <= (dmps[i].tol))      and          quaternion.distance(dmps[i].q_goal, q_track_s[i])  <= dmps[i].tol)

                        if stops[i] == False:
                            if not self.fixed_obst or dmps[i].t == 0:
                                dmp_obs = []
                                obstacles = copy.deepcopy(self.obstacles)
                                self.tf_list.waitForTransform('world', psms[i].name()+'_base' + self.dvrk_frame_id, time=obstacles.header.stamp, timeout = rospy.Duration(5.))
                                for j in range(len(obstacles.poses)):
                                    obstacle = copy.deepcopy(self.tf_list.transformPose(psms[i].name()+'_base' + self.dvrk_frame_id, PoseStamped(header=obstacles.header, pose=obstacles.poses[j])))
                                    dmp_obs.append(dyn_ob(center = np.array([obstacle.pose.position.x, obstacle.pose.position.y, obstacle.pose.position.z]), 
                                                        axis = np.array([obstacles.axes[3*j], obstacles.axes[3*j+1], obstacles.axes[3*j+2]]), 
                                                        coeffs = np.array([obstacles.coeffs[3*j], obstacles.coeffs[3*j+1], obstacles.coeffs[3*j+2]])))
                            
                            F = np.zeros(3)
                            for ob in dmp_obs:
                                F += ob.compute_forcing_term(y_track_s[i], dy_track_s[i]) #considering dynamic obstacles

                            y_track_s[i], dy_track_s[i], _, q_track_s[i], _, _, _ = dmps[i].step(adapt=True, external_force=F)   
                            
                            target.position.x = y_track_s[i][0]
                            target.position.y = y_track_s[i][1]
                            target.position.z = y_track_s[i][2]
                            target.orientation.x = q_track_s[i][1]
                            target.orientation.y = q_track_s[i][2]
                            target.orientation.z = q_track_s[i][3]
                            target.orientation.w = q_track_s[i][0]
                            
                            psms[i].move_cp(pm.fromMsg(target)).wait(is_busy=True)#, interpolate=True, blocking=True)

                            dmps[i].t += 1

                        else:                       
                            target.position.x = dmps[i].x_goal[0]
                            target.position.y = dmps[i].x_goal[1]
                            target.position.z = dmps[i].x_goal[2]
                            target.orientation.x = dmps[i].q_goal[1]
                            target.orientation.y = dmps[i].q_goal[2]
                            target.orientation.z = dmps[i].q_goal[3]
                            target.orientation.w = dmps[i].q_goal[0]
                            
                            psms[i].move_cp(pm.fromMsg(target)).wait(is_busy=True)#, interpolate=True, blocking=True)
                    
                        rate.sleep()

        self.sensing_pub.publish(Int32(0))
        rospy.sleep(2.)

    def execute(self):
        dmp_indices = np.where(np.array(self.policy_types) == "dmp")[0]
        if np.shape(dmp_indices)[0] != 0:
            self.dmp_execute(dmp_indices)
        if not self.failure:
            for i in range(len(self.policies)):
                try:
                    self.custom_policies[self.policies[i]](self.agents[i])
                except:
                    pass     

        #FOR V-REP FOR PEG-RING
        self.current_arm_pub.publish(self.num_arms)
        self.current_color_pub.publish(self.num_rings)
        rospy.sleep(2.)
        #FOR V-REP FOR PEG-RING
        
        #THIS CAN BE MOVED INSIDE RELEASE WHEN V-REP IS NOT USED FOR PER-RING
        release_indices = np.where(np.array(self.policies) == "release")[0]
        for i in range(np.shape(release_indices)[0]):
            if self.agents[i] == "psm1":
                psm = self.psm1
            elif self.agents[i] == "psm2":
                psm = self.psm2

            psm_base = PoseStamped()
            psm_base.header.frame_id = psm.name().upper() + '_base' + self.dvrk_frame_id
            psm_base.pose = pm.toMsg(psm.measured_cp(wait=5.))
            psm_base.header.stamp = rospy.Time(0)
            self.tf_list.waitForTransform('world', psm.name().upper() + '_base' + self.dvrk_frame_id, time=psm_base.header.stamp, timeout=rospy.Duration(5.))
            psm_base = copy.deepcopy(self.tf_list.transformPose('world', psm_base))
            if psm_base.pose.position.z < self.setup_pose.pose.position.z + 0.018:
                start = psm.measured_cp(wait=5.)
                start.p[2] += 0.02
                # psm.dmove(PyKDL.Vector(0., 0., height), blocking = True)
                psm.move_cp(start).wait(is_busy=True)
                # psm.dmove(PyKDL.Vector(0., 0., 0.02), blocking=True) #to ensure free vision to the camera
        #THIS CAN BE MOVED INSIDE RELEASE WHEN V-REP IS NOT USED FOR PER-RING

    def recovery(self):
        psm1_base = PoseStamped()
        psm1_base.header.frame_id = 'PSM1_base' + self.dvrk_frame_id
        psm1_base.pose = pm.toMsg(self.psm1.measured_cp(wait=5.))
        psm1_base.header.stamp = rospy.Time(0)
        self.tf_list.waitForTransform('world', 'PSM1_base' + self.dvrk_frame_id, time=psm1_base.header.stamp, timeout=rospy.Duration(5.))
        psm1_base = copy.deepcopy(self.tf_list.transformPose('world', psm1_base))
        if psm1_base.pose.position.z < self.setup_pose.pose.position.z + 0.018:
            # self.psm1.dmove(PyKDL.Vector(0., 0., 0.02), blocking=True) #to ensure free vision to the camera
            start = psm.measured_cp(wait=5.)
            start.p[2] += 0.02
            # psm.dmove(PyKDL.Vector(0., 0., height), blocking = True)
            psm.move_cp(start).wait(is_busy=True)

        psm2_base = PoseStamped()
        psm2_base.header.frame_id = 'PSM2_base' + self.dvrk_frame_id
        psm2_base.pose = pm.toMsg(self.psm2.measured_cp(wait=5.))
        psm2_base.header.stamp = rospy.Time(0)
        self.tf_list.waitForTransform('world', 'PSM2_base' + self.dvrk_frame_id, time=psm2_base.header.stamp, timeout=rospy.Duration(5.))
        psm2_base = copy.deepcopy(self.tf_list.transformPose('world', psm2_base))
        if psm2_base.pose.position.z < self.setup_pose.pose.position.z + 0.018:
            # self.psm2.dmove(PyKDL.Vector(0., 0., 0.02), blocking=True) #to ensure free vision to the camera
            start = psm.measured_cp(wait=5.)
            start.p[2] += 0.02
            # psm.dmove(PyKDL.Vector(0., 0., height), blocking = True)
            psm.move_cp(start).wait(is_busy=True)

    def reset(self):
        self.failure = False
        self.state = []
        self.n_arms = []
        self.policies = []
        self.policy_types = []
        self.agents = []
        self.location = []
        self.from_user = []
        self.manip_id = []
        self.color = []
        self.dmp_weights = []
        self.got_action = False
        self.new_action = []
        self.traj = [[], []]
        self.stop = False # to understand end of human action
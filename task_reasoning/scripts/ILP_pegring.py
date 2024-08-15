#!/usr/bin/env python3
from hashlib import new
import numpy as np
import copy
import time
import math
from subprocess import check_output as out
from dvrk_task_msgs.msg import ActionArray, ActionRequestFiner, ILPExample
from geometry_msgs.msg import PoseStamped, PoseArray
from std_msgs.msg import String
from tf import TransformListener
from tf_conversions import posemath as pm
import rospy
import rospkg
from std_msgs.msg import String, Bool
import dvrk
from gringoParser import string2fun as parse2atom, _string2list as parse2list
from os import listdir
from os.path import isfile, join
import shutil
import json


#comm class
class ILP(object):
    def __init__(self):
        self.path_task_planning = rospkg.RosPack().get_path('task_reasoning')
        self.path_robot_control = rospkg.RosPack().get_path('robot_control')
        self.task = rospy.get_param("task_name")
        self.psm1 = dvrk.psm('PSM1')
        self.psm2 = dvrk.psm('PSM2')
        self.old_fluents = []
        self.fluents = []
        self.peg_poses = [] 
        self.ring_poses = [] 
        self.old_joints = [np.zeros(6), np.zeros(6)]
        self.colors = ['red', 'green', 'blue', 'yellow', 'white1', 'white2', 'white3', 'white4']
        self.setup_pose = PoseStamped()
        self.location = []
        self.file_to_learn = [] #ILASP routines to be run
        self.action_to_learn = [] #Actions corresponding to ILASP routines to be run
        self.learn = False #is learning needed?
        self.ilp_files = [f for f in listdir(self.path_task_planning+"/ilasp/") if isfile(self.path_task_planning+"/ilasp/"+f)]
        self.ilasp_path = "/home/daniele/Documenti/ILASP/"

        self.poses_pegs_listener = rospy.Subscriber('/all_pegs', PoseArray, self.poses_peg_callback)
        self.poses_rings_listener = rospy.Subscriber('/all_rings', PoseArray, self.poses_ring_callback)
        self.setup_pose_listener = rospy.Subscriber('/setup_pose', PoseStamped, self.setup_pose_cb)
        self.ilp_ex_listener = rospy.Subscriber("/ilp_examples", ILPExample, self.ilp_ex_cb)
        self.action_listener = rospy.Subscriber("/actions/request", ActionArray, self.action_cb)

        self.ilp_notifier = rospy.Publisher("/ilp_done", Bool, queue_size=1)
        self.update_config_actions = rospy.Publisher("update_config_actions", Bool, queue_size=1)

        self.tf_list = TransformListener()



    def has_goal(self, action_name):
        return []

        
    def write_json(self, new_data, filename):
        with open(filename,'r+') as file:
            # First we load existing data into a dict.
            file_data = json.load(file)
            # Join new_data with file_data inside emp_details
            file_data["actions"].append(new_data)
            # Sets file's current position at offset.
            file.seek(0)
            # convert back to json.
            json.dump(file_data, file, indent = 4)


    def setup_pose_cb(self, data):
        self.setup_pose = copy.deepcopy(data)


    def poses_peg_callback(self, data):    
        peg_poses_st = []    
        for pose in data.poses:
            pose_st = PoseStamped()
            pose_st.header = data.header
            pose_st.pose = pose
            peg_poses_st.append(pose_st)
        self.peg_poses = copy.deepcopy(peg_poses_st)


    def poses_ring_callback(self, data):    
        ring_poses_st = []  
        for pose in data.poses:
            pose_st = PoseStamped()
            pose_st.header = data.header
            pose_st.pose = pose
            ring_poses_st.append(pose_st)
        self.ring_poses = copy.deepcopy(ring_poses_st)

    
    def action_cb(self, data):
        self.location = []
        for i in range(len(data.action_list)):
            self.location.append(data.action_list[i].object)

    
    def update_ex_count(self, filename):
        with open(filename, 'r') as f:
            ex_count = 0
            lines = f.readlines()
            ex_lines = [l for l in lines if "#pos" in l or "#neg" in l] #lines with an example
            if len(ex_lines) > 0:
                idx = ex_lines[-1].find("ex") + 2 #where id of last example starts
                j = 1
                while j < len(ex_lines[-1]) - idx:
                    try:
                        ex_count = int(ex_lines[-1][idx : idx + j])
                        j += 1
                    except:
                        break
        return ex_count

    
    def ilp_ex_cb(self, data):
        fluents = self.compute_fluents()

        #NEGATIVE EXAMPLES
        for ex in data.neg.action_list:
            #SELECT CORRECT ILP FILE
            ilp_file = [f for f in self.ilp_files if ex.action in f]
            if len(ilp_file) > 1:
                ilp_file = [f for f in ilp_file if ex.object in f]
            
            #UPDATE COUNT OF EXAMPLES FOR CURRENT FILE TO AVOID CONFLICT BETWEEN EXAMPLES
            ex_count = self.update_ex_count(self.path_task_planning + "/ilasp/" + ilp_file[0])
            ex_count += 1

            example = "#neg(ex" + str(ex_count) + ", {" + ex.action + "(" + ex.robot
            if ex.object != 'none':
                example += "," + ex.object
            if ex.color != 'none':
                example += "," + ex.color
            example += ")}, {}, {"
            for fluent in fluents:
                example += fluent + ". "
            example += "})."

            #UPDATE ILP FILE
            with open(self.path_task_planning + "/ilasp/" + ilp_file[0], 'a') as f:
                f.writelines("\n" + example)
            
            #NEED TO LEARN WHEN AN ACTION IS DECLARED AS INFEASIBLE
            if ilp_file[0] not in self.file_to_learn: 
                self.file_to_learn.append(ilp_file[0])
                self.action_to_learn.append(ex)

        #POSITIVE EXAMPLES
        for ex in data.pos.action_list:
            #SELECT CORRECT ILP FILE
            ilp_file = [f for f in self.ilp_files if ex.action in f]
            ilp_file = [f for f in ilp_file if ex.object in f]
            
            #UPDATE COUNT OF EXAMPLES FOR CURRENT FILE TO AVOID CONFLICT BETWEEN EXAMPLES
            ex_count = self.update_ex_count(self.path_task_planning + "/ilasp/" + ilp_file[0])
            ex_count += 1

            example = "#pos(ex" + str(ex_count) + ", {" + ex.action + "(" + ex.robot
            if ex.object != 'none':
                example += "," + ex.object
            if ex.color != 'none':
                example += "," + ex.color
            example += ")}, {}, {"
            for fluent in fluents:
                example += fluent + ". "
            example += "})."
            
            #UPDATE ILP FILE
            with open(self.path_task_planning + "/ilasp/" + ilp_file[0], 'a') as f:
                f.writelines("\n" + example)

            #LEARNING CAN START ONLY WHEN EXTERNAL ACTIONS FROM USER ARE COMMANDED
            if ex.from_user:
                if ilp_file[0] not in self.file_to_learn:
                    self.file_to_learn.append(ilp_file[0])
                    self.action_to_learn.append(ex)
                self.learn = True


    def compute_fluents(self):
        distances = []
        fluents = []
        joints1 = self.psm1.get_current_joint_position()
        joints2 = self.psm2.get_current_joint_position()

        #to choose region of operation of the two arms
        setup_center_position = self.setup_pose.pose.position.y #y coords, SIM VALUES!

        #compute fluents in world frame
        psm1_base = PoseStamped()
        psm1_base.header.frame_id = self.psm1.name()+'_base'
        psm1_base.pose = pm.toMsg(self.psm1.get_current_position())
        psm1_base.header.stamp = rospy.Time.now()
        self.tf_list.waitForTransform('world', self.psm1.name()+'_base', time=psm1_base.header.stamp, timeout=rospy.Duration(secs = 5.))
        psm1_pose = self.tf_list.transformPose('world', psm1_base)
        psm2_base = PoseStamped()
        psm2_base.header.frame_id = self.psm2.name()+'_base'
        psm2_base.pose = pm.toMsg(self.psm2.get_current_position())
        psm2_base.header.stamp = rospy.Time.now()
        self.tf_list.waitForTransform('world', self.psm2.name()+'_base', time=psm2_base.header.stamp, timeout=rospy.Duration(secs = 5.))
        psm2_pose = self.tf_list.transformPose('world', psm2_base)

        #gripper status
        if self.psm1.get_current_jaw_position() < math.pi / 8. :
            fluents.append('closed_gripper(psm1)')
        if self.psm2.get_current_jaw_position() < math.pi / 8. :
            fluents.append('closed_gripper(psm2)')

        #distances from center
        distance1 = np.linalg.norm([self.setup_pose.pose.position.x - psm1_pose.pose.position.x, 
                                    self.setup_pose.pose.position.y - psm1_pose.pose.position.y])
        distance2 = np.linalg.norm([self.setup_pose.pose.position.x - psm2_pose.pose.position.x, 
                                    self.setup_pose.pose.position.y - psm2_pose.pose.position.y])

        if np.linalg.norm(joints1 - self.old_joints[0]) > 0.005:
            if self.psm1.get_current_jaw_position() < math.pi / 8. and distance1 < 0.01: 
                fluents.append('at(psm1,center)')
        else:
            for s in enumerate(self.old_fluents):
                if "at(psm1" in s:
                    fluents.append(s)

        if np.linalg.norm(joints2 - self.old_joints[1]) > 0.005:
            if self.psm2.get_current_jaw_position() < math.pi / 8. and distance2 < 0.01:
                fluents.append('at(psm2,center)')
        else:
            for s in enumerate(self.old_fluents):
                if "at(psm2" in s:
                    fluents.append(s)


        ring_poses = self.ring_poses
        peg_poses = self.peg_poses
        distance1_prev = 100.
        distance2_prev = 100.
        for peg in peg_poses:
            #peg reachability
            if not (peg.pose.orientation.x == 0. and peg.pose.orientation.y == 0. and peg.pose.orientation.z == 0. and peg.pose.orientation.w == 0.): #peg visible
                if peg.pose.position.y < setup_center_position:
                    fluents.append('reachable(psm1,peg,' + self.colors[peg_poses.index(peg)] + ')')
                else:
                    fluents.append('reachable(psm2,peg,' + self.colors[peg_poses.index(peg)] + ')')
                
                #distances from pegs
                distance1 = np.linalg.norm([peg.pose.position.x - psm1_pose.pose.position.x, 
                                            peg.pose.position.y - psm1_pose.pose.position.y])
                distance2 = np.linalg.norm([peg.pose.position.x - psm2_pose.pose.position.x, 
                                            peg.pose.position.y - psm2_pose.pose.position.y])

                if not('peg' in self.location or 'center' in self.location):
                    if np.linalg.norm(joints1 - self.old_joints[0]) > 0.005 and distance1 < 0.015 and distance1 < distance1_prev and psm1_pose.pose.position.z - 0.002 > peg.pose.position.z and self.psm1.get_current_jaw_position() < math.pi / 8.: 
                        fluents.append('at(psm1,peg,' + self.colors[peg_poses.index(peg)] + ')')
                        distance1_prev = distance1
                    if np.linalg.norm(joints2 - self.old_joints[1]) > 0.005 and distance2 < 0.015 and distance2 < distance2_prev and psm2_pose.pose.position.z - 0.002 > peg.pose.position.z and self.psm2.get_current_jaw_position() < math.pi / 8.:
                        fluents.append('at(psm2,peg,' + self.colors[peg_poses.index(peg)] + ')')  
                        distance2_prev = distance2

                for ring in ring_poses:
                    if not (ring.pose.orientation.x == 0. and ring.pose.orientation.y == 0. and ring.pose.orientation.z == 0. and ring.pose.orientation.w == 0.): #ring visible
                        #check for rings on pegs
                        if np.linalg.norm([ring.pose.position.x - peg.pose.position.x, 
                                            ring.pose.position.y - peg.pose.position.y]) < 0.015 and ring.pose.position.z < self.setup_pose.pose.position.z + 0.01: #less than ring's radius
                            fluents.append('placed(ring,' + self.colors[ring_poses.index(ring)] + ',peg,' + self.colors[peg_poses.index(peg)] + ')')

        distance2_prev = 100.
        distance1_prev = 100.
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
                if distance1 < 0.015 and distance1 < distance1_prev: # half of the opening value for jaw, distance tuned by hand FOR SIM
                    if self.psm1.get_current_jaw_position() < math.pi / 8.:
                        fluents.append('in_hand(psm1,ring,' + self.colors[ring_poses.index(ring)] + ')')
                    elif np.linalg.norm(joints1 - self.old_joints[0]) > 0.005:
                        fluents.append('at(psm1,ring,' + self.colors[ring_poses.index(ring)] + ')')
                    distance1_prev = distance1
                if distance2 < 0.015 and distance2 < distance2_prev: # half of the opening value for jaw, distance tuned by hand FOR SIM
                    if self.psm2.get_current_jaw_position() < math.pi / 8.:
                        fluents.append('in_hand(psm2,ring,' + self.colors[ring_poses.index(ring)] + ')')
                    elif np.linalg.norm(joints2 - self.old_joints[1]) > 0.005:
                        fluents.append('at(psm2,ring,' + self.colors[ring_poses.index(ring)] + ')')
                    distance2_prev = distance2

        self.old_fluents = copy.deepcopy(fluents[:-1])
        self.old_joints = copy.deepcopy([joints1, joints2])

        return fluents


    def run_ilasp_scratch_mod(self):
        #READ step PROGRAM
        filename = rospy.get_param("asp_name")
        with open(self.path_task_planning + '/asp/' + filename, "r") as f:
            old_asp = f.readlines()
        start_step = old_asp.index("#program step(t).\n")
        start_check = old_asp.index("#program check(t).\n")

        for i in range(len(self.file_to_learn)):
            #SEARCH FOR KNOWN AXIOMS IN ASP FILE AND ADD MISSING ONES IN ILASP
            original_axioms = [] #to update ASP
            for j in range(start_check-start_step):
                # if ":" in old_asp[start_step+j]:
                #     continue
                formatted_line = old_asp[start_step+j].replace(" ", "").replace(";", ",").replace(":-", "").replace(":", "").replace(".","").replace("0{", "").replace("}1", "") #standard formatting without blank spaces and semi-colon substituted with comma (std clingo)
                parsed_line = parse2list(formatted_line) 
                if any([atom for atom in parsed_line if parse2atom(atom).name == self.action_to_learn[i].action and str(parse2atom(atom).arguments[1]) == self.action_to_learn[i].object]):
                    original_axioms.append(old_asp[start_step+j])

            #SOLVE ILASP
            result = out([self.ilasp_path+"ILASP", "--version=4", "-ml=3", "--max-rule-length=3", self.path_task_planning + "/ilasp/" + self.file_to_learn[i]])
            result = result.decode("utf-8")
            lines = result.splitlines()
            ilasp_axioms = [line.replace(";", ",").replace(" ", "") for line in lines if (":-" in line)]# and "{" not in line)] #only output containing axioms and not choice rules

            self.update_asp(self.action_to_learn[i], ilasp_axioms, original_axioms)
        
        
        self.learn = False
        self.action_to_learn = []
        self.file_to_learn = []
        self.ilp_notifier.publish(Bool(True))
    

    def update_asp(self, action, new_ax, old_ax):
        filename = rospy.get_param("asp_name")

        #READ OLD PROGRAM
        with open(self.path_task_planning + '/asp/' + filename, "r") as f:
            old_asp = f.readlines()

        #COMMENT OLD AXIOMS
        ending_aggregate = False #if pre-condition is last in the aggregate, remember to close the aggregate when replacing
        for j in range(len(old_ax)):
            idx = old_asp.index(old_ax[j])
            old_asp[idx] = old_asp[idx].replace(old_asp[idx].replace("\n", ""), "%" + old_asp[idx].replace("\n", ""))
            if "}" in old_ax[j]:
                ending_aggregate = True

        #ADD NEW AXIOMS
        start_check = old_asp.index("#program check(t).\n")
        for j in range(len(new_ax)):
            #if action not in axioms, add it
            if action.action not in new_ax[j]:
                new_ax[j] = new_ax[j][0:-1] + "," + action.action +"(_"
                if action.object != "none":
                    new_ax[j] += "," + action.object + ",_"
                new_ax[j] += ")."
            #add time parameter, except for reachable atoms
            new_ax[j] = new_ax[j].replace(" ", "")
            atoms = parse2list(new_ax[j].replace(";", ",").replace(":-", ",").replace(":", ",").replace(".","").replace("0{", "").replace("1{", "").replace("}1", ""))
            for i in range(len(atoms)):
                if 'reachable' not in atoms[i] and "(" in atoms[i]:
                    new_ax[j] = new_ax[j].replace(atoms[i], atoms[i][0:-1]+",t)")
                    atoms[i] = atoms[i].replace(atoms[i], atoms[i][0:-1]+",t)")

            #add constraint
            if "{" not in new_ax[j]:
                old_asp.insert(start_check-1, new_ax[j] + "\n")

            #add aggregate pre-condition
            else:
                head = [a for a in atoms if action.action in a][0]
                new_prec = head + " : "
                for a in atoms:
                    if a != head:
                        new_prec += a + ", "
                if ending_aggregate:
                    new_prec = new_prec[:-2] 
                    new_prec += "}1.\n"
                else:
                    new_prec = new_prec[:-2] 
                    new_prec += ";\n"
                last_prec = [l for l in old_asp if "}" in l][0] #last precondition
                old_asp.insert(old_asp.index(last_prec), new_prec)



        with open(self.path_task_planning + '/asp/' + filename, "w") as f:
            f.writelines(old_asp)

            
            
                    







def main():

    rospy.init_node('ILP_node', disable_signals=True)
    ilp = ILP()
    rospy.sleep(1.)

    while not rospy.is_shutdown():
        if ilp.learn:
            ilp.run_ilasp_scratch_mod() #IGNORE ANY KNOWN AXIOMS

    rospy.spin()







if __name__ == '__main__':
    main()




#!/usr/bin/env python3
import numpy as np
import math
import os
import roslib
import rospkg
import rospy
import actionlib
import json
import sys
import copy
from std_msgs.msg import Int32, Bool
from dvrk_task_msgs.msg import ActionRequestFiner, ActionArray, ILPExample
import dvrk






class Listener(object):
    def __init__(self):

        rospack = rospkg.RosPack()
        self.path = rospack.get_path("robot_control") + '/config/'
        self.task = rospy.get_param("task_name")
        with open(self.path + self.task + ".json") as f:
            self.actions = json.load(f)["actions"]

        self.is_valid = []

        self.ilp_ex_pub = rospy.Publisher("/ilp_examples", ILPExample, queue_size=1)
        self.wrong_plan_pub = rospy.Publisher("/wrong_plan_notify", Bool, queue_size=1)
        self.end_pub = rospy.Publisher('/action_feedback', Bool, queue_size=1)
        rospy.Subscriber("/update_config_actions", Bool, self.config_cb) #for never experienced actions

    def config_cb(self, data):
        with open(self.path + self.task + ".json") as f:
            self.actions = json.load(f)["actions"]
        rospy.logwarn("UPDATED FROM JSON")
        









def main():

    rospy.init_node('TaskDVRK')

    listener = Listener()

    #DEFINE MOTION CONTROL
    if listener.task == "pegring":
        from motion_pegring import motion_manager
    else: #default
        from motion_pegring import motion_manager
    
    motion = motion_manager()
    rospy.sleep(1.)

    while not rospy.is_shutdown():

        # rospy.logwarn('Waiting for the next action from the reasoner...')
        if motion.got_action:
            #ASK FOR VALIDITY CHECK
            for i in range(len(motion.state)):
                rospy.logwarn("ACTION: " + motion.state[i] + "(" + motion.manip_id[i] + "," + motion.location[i] + "," + motion.color[i] + ")")
                is_valid = input("IS ACTION VALID? (y/n)")
                if is_valid == "y":
                    listener.is_valid.append(True)
                else: #invalid by default
                    listener.is_valid.append(False)

            #INFORM ILP MODULE
            ilp_ex = ILPExample()
            for i in range(len(listener.is_valid)):
                if listener.is_valid[i]:
                    ilp_ex.pos.action_list.append(ActionRequestFiner(robot=motion.manip_id[i], action=motion.state[i], object=motion.location[i], color=motion.color[i], from_user=motion.from_user[i], new = motion.new_action[i]))
                else:
                    ilp_ex.neg.action_list.append(ActionRequestFiner(robot=motion.manip_id[i], action=motion.state[i], object=motion.location[i], color=motion.color[i], from_user=motion.from_user[i]))    
            listener.ilp_ex_pub.publish(ilp_ex)

            #EXECUTE ONLY IN CASE OF VALIDITY
            if np.prod(listener.is_valid):
                for i in range(len(motion.state)):
                    current_action = [a for a in listener.actions if a["name"]==motion.state[i] and a["object"]==motion.location[i]]
                    motion.new_action.append(False)
                    while len(current_action) == 0: #new unexperienced action, waiting for update in config file
                        current_action = [a for a in listener.actions if a["name"]==motion.state[i] and a["object"]==motion.location[i]]
                        motion.new_action[i] == True # new unexperienced action -> new motion policy
                    current_action = current_action[0]
                    motion.agents.append(motion.manip_id[i])
                    motion.policies.append(current_action["policy"])
                    motion.policy_types.append(current_action["policy_type"])
                    if current_action["policy_type"] == "dmp":
                        motion.dmp_weights.append(current_action["weights"])
                    motion.n_arms.append(current_action["n_arms"])

                motion.execute() 
                
                #FEEDBACK
                end_msg = Bool(motion.failure)
                listener.end_pub.publish(end_msg)
            
            else:
                listener.wrong_plan_pub.publish(Bool(True))

            #RESET
            motion.reset()
            listener.is_valid = []

    rospy.spin()


if __name__ == '__main__':
    main()
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
import crtk






class Listener(object):
    def __init__(self):

        rospack = rospkg.RosPack()
        self.path = rospack.get_path("robot_control") + '/config/'
        self.task = rospy.get_param("task_name")
        with open(self.path + self.task + ".json") as f:
            self.actions = json.load(f)["actions"]


        self.wrong_plan_pub = rospy.Publisher("/wrong_plan_notify", Bool, queue_size=1)
        self.end_pub = rospy.Publisher('/action_feedback', Bool, queue_size=1)
        self.correct_action_pub = rospy.Publisher("/action_for_sensing", ActionArray, queue_size=1)
        









def main():

    # rospy.init_node('TaskDVRK')
    name = 'TaskDVRK'
    ral = crtk.ral(name)

    listener = Listener()

    #DEFINE MOTION CONTROL
    if listener.task == "pegring":
        from motion_pegring import motion_manager
    else: #default
        from motion_pegring import motion_manager
    
    motion = motion_manager(ral)
    rospy.sleep(1.)

    while not rospy.is_shutdown():
        if motion.got_action:
            wrong_plan = False
            act_for_sensing = ActionArray()
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

                act_for_sensing.action_list.append(ActionRequestFiner(robot=motion.manip_id[i], action=motion.state[i], object=motion.location[i], color=motion.color[i]))

            listener.correct_action_pub.publish(act_for_sensing)
            motion.execute() 
            rospy.logwarn("FINISHED MOTION")
                
            #FEEDBACK
            end_msg = Bool(motion.failure)
            listener.end_pub.publish(end_msg)

            motion.reset()
            listener.wrong_plan_pub.publish(Bool(wrong_plan)) #for ASP manager
            
            rospy.logwarn("MOVING TO NEXT ACTION")

            #RESET
            listener.is_valid = []
            listener.end_ex = False
        

    ral.spin()


if __name__ == '__main__':
    main()
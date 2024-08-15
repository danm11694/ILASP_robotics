#!/usr/bin/env python3

import rospy
import rospkg
import numpy as np
from clingo import Function as Fun, Control, SolveResult, Model, Number
from dvrk_task_msgs.msg import ContextModel, ActionRequestFiner, ActionArray
from std_msgs.msg import Bool, Int32
import copy
from gringoParser import string2fun as parse2atom, _string2list as parse2list
import json



class Listener(object):
    def __init__(self):
        self.path = rospkg.RosPack().get_path("robot_control") + "/config/"
        self.task = rospy.get_param("task_name")

        self.feedback = None
        self.context = None
        self.new_axioms = []
        self.received_context = False
        self.received_axioms = False
        self.wrong_plan = None
        self.got_action = False
        self.asked_context_update = False
        rospy.Subscriber('/context/model', ContextModel, self.context_cb)
        rospy.Subscriber('/action_feedback', Bool, self.feedback_cb)
        rospy.Subscriber("/ilp_done", Bool, self.new_axioms_cb)
        rospy.Subscriber("/wrong_plan_notify", Bool, self.wrong_plan_cb)
        rospy.Subscriber('/actions/request', ActionArray, self.action_cb)

        self.action_pub = rospy.Publisher('/actions/request', ActionArray, queue_size=1)
        self.sensing_pub = rospy.Publisher('/ask_for_sensing', Int32, queue_size=1)


    def feedback_cb(self, data):
        self.feedback = data.data

    def wrong_plan_cb(self, data):
        self.wrong_plan = data.data

    def context_cb(self, data):
        self.context = copy.deepcopy(data)
        self.received_context = True
    
    def new_axioms_cb(self, data):
        self.received_axioms = True

    def action_cb(self, data):
        #ONLY CATCH ACTIONS FROM USER
        for action in data.action_list:
            if action.from_user:
                self.got_action = True

    def ask_user(self):
        confirm = "n"
        while confirm != "y":
            user_action = input("ENTER NEXT ACTION AS NAME(AGENT, object, property) - lower-case = optional : ")
            confirm = input("ENTERED" + user_action + ": IS IT CORRECT? PLEASE TYPE y FOR YES OR n FOR NO")
        user_action.replace(" ", "").lower() #standard formatting
        n_args = 0
        is_new = False
        name = 'none'
        robot = 'none'
        obj = 'none'
        color = 'none'
        for i in range(len(user_action)):
            if user_action[i] == '(':
                i_name = i
                name = user_action[0:i_name]
            elif user_action[i] == ',' or user_action[i] == ')':
                n_args += 1
                if n_args == 1:
                    i_robot = i
                    robot = user_action[i_name+1 : i_robot]
                elif n_args == 2:
                    i_object = i
                    obj = user_action[i_robot+1 : i_object]
                elif n_args == 3:
                    i_color = i
                    color = user_action[i_object+1 : i_color]

        with open(self.path + self.task + ".json", "r") as jsonFile:
            config = json.load(jsonFile)
        new_config = [c for c in config["actions"] if c["name"] == name and obj == c["object"]]
        if len(new_config) == 0:
            is_new = True    

        self.action_pub.publish(ActionArray(action_list = [ActionRequestFiner(action=name, robot=robot, object=obj, color=color, from_user=True, new=is_new)]))
            











class Solver(object):    
    def __init__(self, filename):
        self.filename = filename

        self.control = Control([])
        path = rospkg.RosPack().get_path('task_reasoning')
        self.control.load(path + '/asp/' + self.filename)
        self.control.ground([("base", [])])        
        self.atoms = []
        self.hidden_atoms = []
        self.ordered_atoms = []
        self.action_time = 0  
        self.action_index = 0
        self.step = 1 #for iterative solver

    def restart(self, axioms=[]):
        self.control = Control([])
        path = rospkg.RosPack().get_path('task_reasoning')
        self.control.load(path + '/asp/' + self.filename)
        self.control.ground([("base", [])])        
        self.atoms = []
        self.hidden_atoms = []
        self.step = 1 #for iterative solver
        

    def on_model(self, model):
        self.atoms[:] = model.symbols(shown=True)
        self.hidden_atoms[:] = model.symbols(atoms=True)

    def solve(self, context):
        self.ordered_atoms = copy.deepcopy(self.ordered_atoms[0:self.action_index])
        #DELETE EXTERNALS FROM PREVIOUS CALL
        for atom in self.hidden_atoms:
            self.control.assign_external(atom, False)
        #GROUND NEW EXTERNALS
        for atom in context.atoms:
            self.control.assign_external(parse2atom(atom), True)

        init_time = rospy.Time.now()
        while rospy.Time.now() - init_time < rospy.Duration(secs=10.):
            parts = []

            self.control.cleanup()
            parts.append(("check", [Number(self.step)]))
            self.control.release_external(Fun("query", [Number(self.step-1)]))

            parts.append(("step", [Number(self.step)]))

            self.control.ground(parts)
            self.control.assign_external(Fun("query", [Number(self.step)]), True)
            result = self.control.solve(on_model = self.on_model)
            self.step += 1
            if result.satisfiable:
                break

        if result.satisfiable:
            if self.atoms != []:
                rospy.logwarn("FOUND A PLAN!")
                for atom in self.atoms:
                    tmp_list = ['none', 'none', 'none', 'none', 0]
                    tmp_list[0] = atom.name # name...
                    for i in range(len(atom.arguments)-1): # ...arguments...
                        tmp_list[i+1] = str(atom.arguments[i])
                    tmp_list[-1] = int(str(atom.arguments[-1])) + self.action_time # ...time
                    self.ordered_atoms.append(tmp_list)

                self.ordered_atoms.sort(key = lambda action: action[-1])
                rospy.logwarn('SEQUENCE OF POSSIBLE ACTIONS IS: ')
                rospy.logwarn(self.ordered_atoms)
            
            return False

        else:
            # rospy.signal_shutdown('UNSATISFIABLE PLANNING! EXITING...')
            rospy.logwarn("UNSATISFIABLE! ASKING FOR USER INPUT...")
            return True

    def get_action_msg(self):
        actions = ActionArray()
        while len(self.ordered_atoms) > self.action_index and self.ordered_atoms[self.action_index][-1] == self.action_time + 1:
            action = ActionRequestFiner()
            action.robot = str(self.ordered_atoms[self.action_index][1])
            action.action = str(self.ordered_atoms[self.action_index][0])
            action.object = str(self.ordered_atoms[self.action_index][2])
            action.color = str(self.ordered_atoms[self.action_index][3])

            actions.action_list.append(action)
            self.action_index += 1

        if actions != ActionArray():
            self.action_time += 1
        return actions












def main():
    rospy.init_node('ASP_manager')
    filename = rospy.get_param("asp_name")
    # failed_planning = False

    listener = Listener()
    solver = Solver(filename)

    #ASK FOR FLUENTS
    listener.sensing_pub.publish(Int32(1))

    while not rospy.is_shutdown():
        if listener.received_context:
            # failed_planning = False
            rospy.logwarn('FLUENTS ARE:')
            rospy.logwarn(listener.context)
            solver.restart()
            failed_planning = solver.solve(listener.context)
            listener.received_context = False
            if failed_planning:
                rospy.logwarn("ACTION INPUT NEEDED BY USER! (in failed planning)")
                listener.ask_user()

        #ACTION FROM AUTONOMOUS SYSTEM
        action = solver.get_action_msg()
        if action != ActionArray():
            listener.action_pub.publish(action)
            #ACTION FEEDBACK
            rospy.logwarn("WAITING FOR ACTION FEEDBACK...")
            while listener.wrong_plan is None:
                pass
            rospy.logwarn("GOT FEEDBACK")
            if listener.feedback:
                #ASK FOR FLUENTS IF FAILURE OCCURS
                rospy.logwarn("FAILURE")
                solver.ordered_atoms = solver.ordered_atoms[:solver.action_index]
                listener.sensing_pub.publish(Int32(1))
            elif listener.wrong_plan:
                #ASK FOR USER INPUT
                rospy.logwarn("FORBIDDEN ACTION")
                solver.ordered_atoms = solver.ordered_atoms[:solver.action_index]
                listener.ask_user()

            listener.feedback = None
            listener.wrong_plan = None

        elif not listener.got_action:
            #IF GOAL ALREADY SATISFIED, ASK FOR NEW CONTEXT DESCRIPTION
            if not listener.asked_context_update: #to temporize requests of new context, avoiding conflicts
                init_time = rospy.Time.now()
                listener.asked_context_update = True
                rospy.logwarn("ASKING FOR NEW CONTEXT")
                listener.sensing_pub.publish(Int32(1))
            if rospy.Time.now() - init_time > rospy.Duration(10.):
                rospy.logwarn("RE-ASKING FOR NEW CONTEXT")
                listener.asked_context_update = False

        #ACTION COMES FROM EXTERNAL USER 
        if listener.got_action:
            listener.got_action = False
            #ACTION FEEDBACK
            rospy.logwarn("WAITING FOR ACTION FEEDBACK (commanded from user)...")
            while listener.wrong_plan is None:
                pass
            rospy.logwarn("GOT FEEDBACK")
            # if listener.feedback:
            #     #ASK FOR FLUENTS IF FAILURE OCCURS
            #     rospy.logwarn("FAILURE IN USER ACTION")
            #     listener.sensing_pub.publish(Int32(1))

            listener.feedback = None
            listener.wrong_plan = None

            #ALWAYS ASK FOR FLUENTS WHEN USER ACTION IS RECEIVED
            listener.sensing_pub.publish(Int32(1))

    rospy.spin()








if __name__ == '__main__':
    main()

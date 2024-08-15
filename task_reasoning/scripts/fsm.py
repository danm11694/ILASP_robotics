#!/usr/bin/env python

import rospy
import smach
import smach_ros
import crtk
import copy
import rospkg
import dvrk
import math
from std_msgs.msg import Bool, Int32
from dvrk_task_msgs.msg import ContextModel, ActionRequestFiner, ActionArray


class Listener(object):
    def __init__(self):
        self.path = rospkg.RosPack().get_path("robot_control") + "/config/"
        self.task = rospy.get_param("task_name")

        self.start = True
        self.current_ring = None # the ring to be placed
        self.current_arm_ring = None # the arm which can reach the current ring
        self.current_arm_peg = None # the arm which can reach the peg corresponding to the current ring
        self.rings = [] # all rings
        self.pegs = [] # all pegs
        self.arm_rings = [] # arms which can reach all rings
        self.arm_pegs = [] # arms which can reach all pegs
        self.feedback = None
        self.context = None
        self.received_context = False

        rospy.Subscriber('/context/model', ContextModel, self.context_cb)
        rospy.Subscriber('/action_feedback', Bool, self.feedback_cb)

        self.action_pub = rospy.Publisher('/actions/request', ActionArray, queue_size=1)
        self.correct_action_pub = rospy.Publisher("/action_for_sensing", ActionArray, queue_size=1)
        self.sensing_pub = rospy.Publisher('/ask_for_sensing', Int32, queue_size=1)


    def feedback_cb(self, data):
        self.feedback = data.data

    def context_cb(self, data):
        self.context = copy.deepcopy(data.atoms)
        self.received_context = True
            










class MoveToRing(smach.State):
    def __init__(self, listener):
        smach.State.__init__(self, outcomes=['done'], output_keys=['arm'])
        self.listener = listener

    def execute(self, userdata):
        # Logic for reaching the ring
        
        # open grippers at the beginning of task only
        if self.listener.start:
            actions = ActionArray()
            for psm in ["psm1", "psm2"]:
                action = ActionRequestFiner(robot=psm, action='release', object='none', color='none')
                actions.action_list.append(action)
            self.listener.action_pub.publish(actions)
            self.listener.correct_action_pub.publish(actions)
            while self.listener.feedback is None:
                pass
            self.listener.feedback = None
            self.listener.start = False


        self.listener.current_ring = self.listener.rings.pop()
        self.listener.current_arm_ring = self.listener.arm_rings.pop()
        self.listener.current_arm_peg = [self.listener.arm_pegs[i] for i in range(len(self.listener.arm_pegs)) if self.listener.pegs[i] == self.listener.current_ring][0]

        actions = ActionArray()
        action = ActionRequestFiner(robot=self.listener.current_arm_ring, action='move', object='ring', color=self.listener.current_ring)
        actions.action_list.append(action)

        self.listener.action_pub.publish(actions)
        self.listener.correct_action_pub.publish(actions)
        while self.listener.feedback is None:
            pass
        self.listener.feedback = None

        userdata.arm = self.listener.current_arm_ring
        return 'done'

class GraspRing(smach.State):
    def __init__(self, listener):
        smach.State.__init__(self, outcomes=['at_ring'], input_keys=['arm'], output_keys=['arm'])
        self.listener = listener

    def execute(self, userdata):
        # Logic for grasping the ring
        actions = ActionArray()
        action = ActionRequestFiner(robot=userdata.arm, action='grasp', object='ring', color=self.listener.current_ring)
        actions.action_list.append(action)

        self.listener.action_pub.publish(actions)
        self.listener.correct_action_pub.publish(actions)
        while self.listener.feedback is None:
            pass
        self.listener.feedback = None
        
        # MODIFY TRANSITION USING userdata.arm
        return 'at_ring'

class ExtractRing(smach.State):
    def __init__(self, listener):
        smach.State.__init__(self, outcomes=['reachable_peg'])
        self.listener = listener

    def execute(self, userdata):
        # Logic for extracting the ring
        actions = ActionArray()
        action = ActionRequestFiner(robot=self.listener.current_arm_ring, action='extract', object='ring', color=self.listener.current_ring)
        actions.action_list.append(action)

        self.listener.action_pub.publish(actions)
        self.listener.correct_action_pub.publish(actions)
        while self.listener.feedback is None:
            pass
        self.listener.feedback = None
        
        # MODIFY TRANSITION
        return 'reachable_peg'

class MoveToPeg(smach.State):
    def __init__(self, listener):
        smach.State.__init__(self, outcomes=['done'], output_keys=['arm'])
        self.listener = listener

    def execute(self, userdata):
        # Logic for moving to the peg
        actions = ActionArray()
        action = ActionRequestFiner(robot=self.listener.current_arm_peg, action='move', object='peg', color=self.listener.current_ring)
        actions.action_list.append(action)

        self.listener.action_pub.publish(actions)
        self.listener.correct_action_pub.publish(actions)
        while self.listener.feedback is None:
            pass
        self.listener.feedback = None

        userdata.arm = self.listener.current_arm_peg
        return 'done'

class MoveToCenter(smach.State):
    def __init__(self, listener):
        smach.State.__init__(self, outcomes=['done'], output_keys=['arm'])
        self.listener = listener

    def execute(self, userdata):
        # Logic for moving to the center
        actions = ActionArray()
        action = ActionRequestFiner(robot=self.listener.current_arm_ring, action='move', object='center', color=self.listener.current_ring)
        actions.action_list.append(action)

        self.listener.action_pub.publish(actions)
        self.listener.correct_action_pub.publish(actions)
        while self.listener.feedback is None:
            pass
        self.listener.feedback = None
        
        userdata.arm = self.listener.current_arm_peg
        return 'done'

class ReleaseRing(smach.State):
    def __init__(self, listener):
        smach.State.__init__(self, outcomes=['at_peg'], input_keys=['arm'])
        self.listener = listener

    def execute(self, userdata):
        # Logic for placing the ring on the peg
        actions = ActionArray()
        action = ActionRequestFiner(robot=userdata.arm, action='release', object='none', color='none')
        actions.action_list.append(action)

        self.listener.action_pub.publish(actions)
        self.listener.correct_action_pub.publish(actions)
        while self.listener.feedback is None:
            pass
        self.listener.feedback = None

        # MODIFY TRANSITION USING userdata.arm and list of available rings
        return 'at_peg'




def open_grippers(ral):
    psm1 = dvrk.psm(ral, "PSM1")
    psm1.jaw.open(math.pi/4.).wait(is_busy=True)
    psm2 = dvrk.psm(ral, "PSM2")
    psm2.jaw.open(math.pi/4.).wait(is_busy=True)







def main():
    ral = crtk.ral("FSM")
    listener = Listener()

    #ASK FOR FLUENTS
    listener.sensing_pub.publish(Int32(1))
    start = rospy.Time.now()
    while not listener.received_context:
        rospy.logwarn("WAITING FOR CONTEXT")
        if rospy.Time.now() - start > rospy.Duration(secs=3.):
            rospy.logwarn("UPDATING TIME")
            listener.sensing_pub.publish(Int32(1))
            start = rospy.Time.now()

    listener.rings = [el.replace(")", "").replace("(", "").split(",")[-1] for el in listener.context if "reachable" in el and "ring" in el]
    listener.arm_rings = [el.replace(")", "").replace("(", "").split(",")[0].replace("reachable", "") for el in listener.context if "reachable" in el and "ring" in el]
    listener.pegs = [el.replace(")", "").replace("(", "").split(",")[-1] for el in listener.context if "reachable" in el and "peg" in el]
    listener.arm_pegs = [el.replace(")", "").replace("(", "").split(",")[0].replace("reachable", "") for el in listener.context if "reachable" in el and "peg" in el]

    #FSM WITH SMACH
    sm_top = smach.StateMachine(outcomes=['succeeded'])

    with sm_top:
        smach.StateMachine.add('MOVE_TO_RING', MoveToRing(listener), transitions={'done': 'GRASP_RING'})
        smach.StateMachine.add('GRASP_RING', GraspRing(listener), transitions={'at_ring': 'EXTRACT_RING'}) # add transition to manage move_to_center
        smach.StateMachine.add('EXTRACT_RING', ExtractRing(listener), transitions={'reachable_peg': 'MOVE_TO_PEG'}) # add transition to manage move_to_center
        # smach.StateMachine.add('MOVE_TO_CENTER', MoveToCenter(listener), transitions={'done': 'GRASP_RING'})
        smach.StateMachine.add('MOVE_TO_PEG', MoveToPeg(listener), transitions={'done': 'RELEASE_RING'})
        smach.StateMachine.add('RELEASE_RING', ReleaseRing(listener), transitions={'at_peg': 'succeeded'}) # add transitions to manage move_to_center and end of task

    sis = smach_ros.IntrospectionServer('smach_server', sm_top, '/SM_ROOT')
    sis.start()

    outcome = sm_top.execute()
    ral.spin()
    sis.stop()

if __name__ == '__main__':
    main()

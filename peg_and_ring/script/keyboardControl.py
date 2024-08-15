#!/usr/bin/env python

from dvrk.arm import arm
from dvrk.psm import *
import rospy
import numpy as np

import array
import curses

# giving access to ncurses to class and main
global stdscr

class KeyboardController():

    def __init__(self, name, arm_name):

        self.arm_name = arm_name

        self.robot_ = arm(self.arm_name)
        self.rate_ = 10
        self.rosRate_ = rospy.Rate(self.rate_) # Hz
        self.period_ = 1 / float(self.rate_)
        self.counter_ = 0
        self.__translation_displ = [float(0)]*3
        self.__displ = 0.005
        self.__rot_displ = 0.021
    # MAIN RUNNING THREAD #
    def run(self):
        # FIRST CONTROLLED MOTION TOWARDS GOAL
        global stdscr
        #stdscr.addstr(0,0,"Moving to init position")
        # self.robot_.move_joint_list([float(0),
        #                             float(0),
        #                             0.06,
        #                             float(0),
        #                             float(0),
        #                             float(0),
        #                             float(0)])

        # DIRECT TRAJECTORY CONTROL
        while not rospy.is_shutdown():
            pressedKeys = [] 
            current_pose = self.robot_.get_current_position()
            # checking for initialization condition
            if((('PSM1' in self.arm_name) or ('PSM2' in self.arm_name)) and (current_pose.p[2] > -0.045)):
                self.robot_.delta_move_joint_list([self.__displ], [2], interpolate= False)
                stdscr.addstr(0,0,"Waiting for movement to init pose")
                stdscr.refresh()
            else:
                stdscr.addstr(0,0,"Keyboard control started")
                stdscr.refresh()

                newkey = stdscr.getch()
                pressedKeys.append(newkey)
                # while (newkey != -1):
                #     newkey = stdscr.getch()
                #     pressedKeys.append(newkey)

                cmd = 0.0
                indx = 0

                for i in range(3):
                    self.__translation_displ[i] = float(0)

                if 260 in pressedKeys: # arrowleft neg jnt1
                    cmd = -self.__displ
                    indx = 0
                    #self.robot_.delta_move_joint_list([-self.__displ], [0], interpolate= False)

                if 261 in pressedKeys: # arrowright pos jnt1
                    cmd = self.__displ
                    indx = 0
                    #self.robot_.delta_move_joint_list([self.__displ], [0], interpolate= False)

                if 259 in pressedKeys: # arrowdown neg jnt2
                    cmd = -self.__displ
                    indx = 1
                    #self.robot_.delta_move_joint_list([-self.__displ], [1], interpolate= False)

                if 258 in pressedKeys: # arrowup pos jnt2
                    cmd = self.__displ
                    indx = 1
                    #self.robot_.delta_move_joint_list([self.__displ], [1], interpolate= False)

                if 97 in pressedKeys: # 'a' neg jnt3
                    cmd = -self.__displ / 3.0
                    indx = 2
                    #self.robot_.delta_move_joint_list([-self.__displ/3.0], [2], interpolate= False)

                if 122 in pressedKeys: # 'z' pos jnt3
                    cmd = self.__displ / 3.0
                    indx = 2
                    #self.robot_.delta_move_joint_list([self.__displ/3.0], [2], interpolate= False)

                if 115 in pressedKeys: # 's' pos jnt4
                    cmd = self.__rot_displ
                    indx = 3
                    #self.robot_.delta_move_joint_list([self.__displ], [3], interpolate= False)

                if 120 in pressedKeys: # 'x' neg jnt4
                    cmd = -self.__rot_displ
                    indx = 3
                    #self.robot_.delta_move_joint_list([-self.__displ], [3], interpolate= False)

                self.robot_.dmove_joint_one(cmd, indx, interpolate=True, blocking=False)  # with interpolation

            self.counter_ += 1

            #time.sleep(self.period_)
            curses.flushinp()
            self.rosRate_.sleep()

        pass

#################CLASS END#################
if __name__ == '__main__':

    global stdscr

    try:
        print("... Starting dvrk_KeyboardController ...")
        var = raw_input("Please enter the desired robot [ECM, MTM(L/R), PSM(1/2)]: ")

        print("Selected robot is {} ".format(var))

        #Setting up ncurses
        stdscr = curses.initscr()
        curses.noecho()
        curses.cbreak()
        stdscr.keypad(1)
        #Creating controller instance
        kc = KeyboardController(var+"Keyboard", var)
        kc.run()
        #Closing ncurses
        curses.nocbreak(); stdscr.keypad(0); curses.echo()
        curses.endwin()
    except rospy.ROSInterruptException:
        #if an exception arises, still closing ncurses
        curses.nocbreak(); stdscr.keypad(0); curses.echo()
        curses.endwin()
        pass

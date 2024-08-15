#!/usr/bin/env python3

import rospy
import rosbag
from dmp_poses import DMPs_pose
import numpy as np
import dvrk
from tf_conversions import posemath as pm
from geometry_msgs.msg import Pose
from robot_control.srv import ComputeDmp
import quaternion
import PyKDL
from matplotlib import pyplot as plt
import rospkg




pose = Pose()
got_pose = False



def target_cb(msg):
    """
    EXPECTING PoseStamped MSG
    """
    global pose
    global got_pose
    got_pose = True
    pose = msg.pose




def extract(bagfile, pose_topic, msg_type):
    n = 0
    data = []
    with rosbag.Bag(bagfile, 'r') as bag:
        for (topic, msg, ts) in bag.read_messages(topics=str(pose_topic)):
            if msg_type == "PoseStamped":
                data.append([msg.pose.position.x, msg.pose.position.y,
                         msg.pose.position.z,
                         msg.pose.orientation.w, msg.pose.orientation.x,
                         msg.pose.orientation.y, msg.pose.orientation.z])
            else:
                assert False, "Unknown message type"
            n += 1
    
    return data









def main():
    global pose
    global got_pose

    rospy.init_node("test_dmp_ars_demo")
    rospack = rospkg.RosPack()
    path = rospack.get_path("robot_control") + '/scripts/'


    #read traj
    traj = np.array(extract(path+"dmp2.bag", "/dvrk/PSM1/position_cartesian_current", "PoseStamped"))
    # plt.plot(traj[:,0], label="x")
    # plt.plot(traj[:,1], label="y")
    # plt.plot(traj[:,2], label="z")
    # plt.legend()
    # plt.show()
    
    #compute deltas for relative motion from any starting position
    delta_pos = traj[-1,:3] - traj[0,:3]
    start_or = PyKDL.Rotation.Quaternion(traj[0,3], traj[0,4], traj[0,5], traj[0,6])
    final_or = PyKDL.Rotation.Quaternion(traj[-1,3], traj[-1,4], traj[-1,5], traj[-1,6])
    # rospy.logwarn(final_or)
    delta_quat = final_or * start_or.Inverse()
    # rospy.logwarn(delta_quat * start_or)

    #learn dmp
    new_dmp = DMPs_pose()
    new_dmp.imitate_path(np.array(traj)) #imitate human trajectory



    psm = dvrk.psm("PSM1")
    rospy.sleep(1.)
    start = pm.toMsg(psm.get_current_position())

    #move to init orientation
    # start.orientation.x = traj[0,3]
    # start.orientation.y = traj[0,4]
    # start.orientation.z = traj[0,5]
    # start.orientation.w = traj[0,6]
    # psm.move(pm.fromMsg(start), blocking=True)

    #init dmp
    new_dmp.t = 0
    new_dmp.q_0 = np.array([start.orientation.w, start.orientation.x, start.orientation.y, start.orientation.z])
    new_dmp.x_0 = np.array([start.position.x, start.position.y, start.position.z])
    start_or = PyKDL.Rotation.Quaternion(start.orientation.x, start.orientation.y, start.orientation.z, start.orientation.w)
    rospy.logwarn(new_dmp.x_0)

    rospy.logwarn("WAITING FOR TARGET")
    # while not got_pose:
    #     pass
    rospy.logwarn("GOT TARGET")

    #compute goal pos
    #debug
    goal_pos = new_dmp.x_0 + delta_pos
    pose.position.x = goal_pos[0]
    pose.position.y = goal_pos[1]
    pose.position.z = goal_pos[2]
    new_dmp.x_goal = np.array([pose.position.x, pose.position.y, pose.position.z])

    #compute goal or
    rot_goal = delta_quat * start_or
    q_goal = rot_goal.GetQuaternion()
    new_dmp.q_goal = np.array([q_goal[3], q_goal[0], q_goal[1], q_goal[2]])





    #exec
    x_track, dx_track, ddx_track, q_track, dq_track, eta_track, deta_track, t_track = new_dmp.rollout()
    target = Pose()

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():

        for i in range(np.shape(x_track)[0]):  
            
            target.position.x = x_track[i][0]
            target.position.y = x_track[i][1]
            target.position.z = x_track[i][2]
            target.orientation.x =  q_track[i][1] # new_dmp.q_0[1]
            target.orientation.y =  q_track[i][2] # new_dmp.q_0[2]
            target.orientation.z =  q_track[i][3] # new_dmp.q_0[3]
            target.orientation.w =  q_track[i][0] # new_dmp.q_0[0]

            psm.move(pm.fromMsg(target), interpolate=True, blocking=False)

            rate.sleep()
        
        rospy.logwarn("FINISHED MOTION")
        break


        # while not stop:
        #     # if (new_dmp.t == 0):
        #     #     new_dmp.first = True
        #     # else:
        #     #     new_dmp.first = False

        #     stop = ((np.nan_to_num(np.linalg.norm(y_track_s - new_dmp.x_goal) / (np.linalg.norm(new_dmp.x_goal - new_dmp.x_0) + 1e-6)) <= (new_dmp.tol))      and          quaternion.distance(new_dmp.q_goal, q_track_s)  <= new_dmp.tol)

        #     rospy.logwarn(np.linalg.norm(y_track_s - new_dmp.x_goal) / (np.linalg.norm(new_dmp.x_goal - new_dmp.x_0) ))
        #     if not stop:
                
        #         y_track_s, dy_track_s, _, q_track_s, _, _, _ = new_dmp.step(adapt=False)   
                
        #         target.position.x = y_track_s[0]
        #         target.position.y = y_track_s[1]
        #         target.position.z = y_track_s[2]
        #         target.orientation.x =  q_track_s[1] # new_dmp.q_0[1]
        #         target.orientation.y =  q_track_s[2] # new_dmp.q_0[2]
        #         target.orientation.z =  q_track_s[3] # new_dmp.q_0[3]
        #         target.orientation.w =  q_track_s[0] # new_dmp.q_0[0]
                
        #         psm.move(pm.fromMsg(target), interpolate=True, blocking=False)

        #         # new_dmp.t += 1

        #     else:      
        #         rospy.logwarn("FINISHING MOTION")                 
        #         target.position.x = new_dmp.x_goal[0]
        #         target.position.y = new_dmp.x_goal[1]
        #         target.position.z = new_dmp.x_goal[2]
        #         target.orientation.x = new_dmp.q_goal[1]
        #         target.orientation.y = new_dmp.q_goal[2]
        #         target.orientation.z = new_dmp.q_goal[3]
        #         target.orientation.w = new_dmp.q_goal[0]
                
        #         psm.move(pm.fromMsg(target), interpolate=True, blocking=True)
        
        #     rate.sleep()





    rospy.spin()











if __name__ == "__main__":
    main()

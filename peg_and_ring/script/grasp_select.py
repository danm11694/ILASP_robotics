#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2

import geometry_msgs.msg
import tf

cloud = [] # global variable to store the point cloud
listener = tf.TransformListener()
trans = []
rot = []


def cloudCallback(msg):
    global listener
    global cloud
    # try:
    #     (trans,rot) = listener.lookupTransform('/world', msg.header.frame_id, rospy.Time(0))
    # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
    #     print("world doesn't exists")

    if len(cloud) == 0:
        for p in point_cloud2.read_points(msg):
            cloud.append([p[0], p[1], p[2]])


# Create a ROS node.
rospy.init_node('select_grasp')

# Subscribe to the ROS topic that contains the grasps.
cloud_sub = rospy.Subscriber('/ring_cloud', PointCloud2, cloudCallback)

# Wait for point cloud to arrive.
while len(cloud) == 0:
    rospy.sleep(0.01)


# Extract the nonplanar indices. Uses a least squares fit AX = b. Plane equation: z = ax + by + c.
import numpy as np
from scipy.linalg import lstsq

cloud = np.asarray(cloud)
X = cloud
A = np.c_[X[:,0], X[:,1], np.ones(X.shape[0])]
C, _, _, _ = lstsq(A, X[:,2])
# a, b, c, d = C[0], C[1], -1., C[2] # coefficients of the form: a*x + b*y + c*z + d = 0.
a = 0.799451 
b = 0.074724  
c = -0.596066  
d= -0.553138
dist = ((a*X[:,0] + b*X[:,1] + d) - X[:,2])**2
err = dist.sum()
idx = np.where(dist > 0.005)


# Publish point cloud and nonplanar indices.
# from gpd.msg import CloudIndexed
# from std_msgs.msg import Header, Int64
# from geometry_msgs.msg import Point

# pub = rospy.Publisher('cloud_indexed', CloudIndexed, queue_size=1)

# msg = CloudIndexed()
# header = Header()
# header.frame_id = "/camera_link"
# header.stamp = rospy.Time.now()
# msg.cloud_sources.cloud = point_cloud2.create_cloud_xyz32(header, cloud.tolist())
# msg.cloud_sources.view_points.append(Point(0,0,0))
# for i in xrange(cloud.shape[0]):
#     msg.cloud_sources.camera_source.append(Int64(0))
# for i in idx[0]:
#     msg.indices.append(Int64(i))    
# s = raw_input('Hit [ENTER] to publish')
# pub.publish(msg)
# rospy.sleep(2)
# print 'Published cloud with', len(msg.indices), 'indices'


# Select a grasp for the robot to execute.
from gpd.msg import GraspConfigList

grasps = [] # global variable to store grasps

def callback(msg):
    global grasps
    grasps = msg.grasps

# Subscribe to the ROS topic that contains the grasps.
grasps_sub = rospy.Subscriber('/detect_grasps/clustered_grasps', GraspConfigList, callback)
pose_pub = rospy.Publisher('/franka_pub/grasping_point', geometry_msgs.msg.PoseStamped, queue_size=100)

# Wait for grasps to arrive.
rate = rospy.Rate(1)

while not rospy.is_shutdown():
    if len(grasps) > 0:
        rospy.loginfo('Received %d grasps.', len(grasps))
        break
    
# grasp = grasps[0] # grasps are sorted in descending order by score
# print (grasp.approach)
# print (grasp.axis)
# print (np.dot([grasp.approach.x,grasp.approach.y,grasp.approach.z], [grasp.axis.x,grasp.axis.y,grasp.axis.z]))
# print 'Selected grasp with score:', grasp.score

selected_one = 0
for i in range(len(grasps)):
    grasp = grasps[i]
    if ((grasp.approach.x > 0 and grasp.axis.x > 0 ) and (grasp.approach.y > 0 and grasp.axis.y > 0 ) and (grasp.approach.z < 0 and grasp.axis.z > 0 ) ):
        if (np.dot([grasp.approach.x,grasp.approach.y,grasp.approach.z], [grasp.axis.x,grasp.axis.y,grasp.axis.z]) < 0):
            selected_one = grasp
            print 'Selected grasp with score:', grasp.score
            break
            
# selected_one = grasps[0]
if(selected_one != 0):
    # print (selected_one.bottom)
    # R = [selected_one.approach.x,selected_one.approach.y,selected_one.approach.z; selected_one.binormal.x,selected_one.binormal.y,selected_one.binormal.z; selected_one.axis.x,selected_one.axis.y,selected_one.axis.z]
    R = np.matrix([[selected_one.approach.x,selected_one.binormal.x,selected_one.axis.x,0], 
                  [selected_one.approach.y,selected_one.binormal.y,selected_one.axis.y,0], 
                  [selected_one.approach.z,selected_one.binormal.z,selected_one.axis.z,0],
                  [0,0,0,1]], dtype='float32')
    # print (R)
    q = tf.transformations.quaternion_from_matrix( R )

    pose_ee = geometry_msgs.msg.PoseStamped()
    pose_ee.header.stamp = rospy.Time.now()
    pose_ee.pose.position.x = selected_one.bottom.x
    pose_ee.pose.position.y = selected_one.bottom.y
    pose_ee.pose.position.z = selected_one.bottom.z
    pose_ee.pose.orientation.x = q[0]
    pose_ee.pose.orientation.y = q[1]
    pose_ee.pose.orientation.z = q[2]
    pose_ee.pose.orientation.w = q[3]
    pose_pub.publish(pose_ee)


    



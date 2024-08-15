#!/usr/bin/env python
import rosbag
import rospy
import time

rospy.init_node('fixTime', anonymous=True)
in_bag = rosbag.Bag('/home/andrea/Downloads/red_blue_first.bag')
tmp = []
out_bag = rosbag.Bag('fix_red_blue_first.bag', 'w') 
timer = 0
pre_t = 0
ty = None
# 18sec max
for topic, msg, t in in_bag.read_messages(topics=['/camera/color/image_rect_color/compressed']):
    out_bag.write('/camera/color/image_rect_color/compressed', msg, t=rospy.Time(secs=pre_t))
    pre_t += 18.0/(3390*12)
    
    
    
in_bag.close()
out_bag.close()
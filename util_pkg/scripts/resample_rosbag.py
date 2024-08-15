#!/usr/bin/env python
import rosbag

in_bag = rosbag.Bag('2018-05-31-16-41-41.bag')
tmp = []
out_bag = rosbag.Bag('angolare.bag', 'w') 
timer = 0
pre_t = 0
ty = None
for topic, msg, t in in_bag.read_messages(topics=['/franka_pub/pose']):
    delta = t.to_sec() - pre_t
    if(timer >= 0.03333333):
        # print str(timer)
        out_bag.write('/franka_pub/pose', msg, t=t)
        timer = 0
    timer = timer + delta
    pre_t = t.to_sec()
in_bag.close()
out_bag.close()
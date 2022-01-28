## 

import rosbag
from std_msgs.msg import Float64

with rosbag.Bag('bag_files/timed_r0.bag', 'w') as outbag:
    for topic, msg, t in rosbag.Bag('bag_files/2Robots_CLAM/r0.bag').read_messages():
        # This also replaces tf timestamps under the assumption 
        # that all transforms in the message share the same timestamp
        # if topic == "/tf" and msg.transforms:
        #     outbag.write(topic, msg, msg.transforms[0].header.stamp)
        # else:
        #     outbag.write(topic, msg, msg.header.stamp if msg._has_header else t)
        
        topic_msg=Float64(data=round(t.to_time(),1))
        
        outbag.write(topic, msg, t)
        outbag.write('/time',topic_msg,t)
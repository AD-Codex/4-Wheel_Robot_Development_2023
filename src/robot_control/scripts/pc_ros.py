#!/usr/bin/env python3

import rospy
from std_msgs.msg import String


if __name__ == '__main__':
    rospy.init_node('remote_pc_node')
    rospy.loginfo("remote pc control node start")

    pub = rospy.Publisher("/pc/avtice", String, queue_size=10)
    
    while not rospy.is_shutdown():
        msg = "remote pc connected"
        pub.publish(msg)

    rospy.spin()
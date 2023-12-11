#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu

def imu_callback(msg : Imu):
    rospy.loginfo(msg.linear_acceleration.x)

if __name__ == '__main__':
    rospy.init_node('test_node')

    rospy.loginfo("hello")
    rospy.logwarn("warning")
    rospy.logerr("error")

    rate = rospy.Rate(1)
    i = 0

    Imu_sub = rospy.Subscriber("/zed2/zed_node/imu/data", Imu, callback=imu_callback)

    while not rospy.is_shutdown():
        text = "Hello " , i
        rospy.loginfo(text)
        rate.sleep()
        i=i+1
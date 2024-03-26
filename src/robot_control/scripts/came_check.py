#!/usr/bin/env python3

import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import numpy as np

shape = (376, 672, 3)
img = np.empty(shape)

shape = (376, 672)
depth = np.empty(shape)

def image_read(msg: Image):
    global img

    bridge = CvBridge()
    orig = bridge.imgmsg_to_cv2(msg, "bgra8")
    img = orig


def depth_read(msg: Image):
    global depth

    bridge = CvBridge()
    orig = bridge.imgmsg_to_cv2(msg, "32FC1")
    depth = orig



if __name__ == '__main__':
    rospy.init_node('camTest_node')
    rospy.loginfo("camera test node start")

    rate = rospy.Rate(30)

    sub_image = rospy.Subscriber("/zed_node/image", Image, callback=image_read)
    sub_depth = rospy.Subscriber("/zed_node/depth", Image, callback=depth_read)
    

    while not rospy.is_shutdown():

        cv2.imshow("new image read", img)
        cv2.imshow("new depth read", depth/5000)

        if cv2.waitKey(1) & 0XFF == ord("q"):
            break
        
        rate.sleep()
    

    rospy.spin()
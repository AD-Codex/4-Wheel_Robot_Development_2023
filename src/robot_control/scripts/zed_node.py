#!/usr/bin/env python3

# # zed came read publish
# # http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython
# # https://industrial-training-master.readthedocs.io/en/melodic/_source/session5/OpenCV-in-Python.html


import rospy
import pyzed.sl as sl
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import numpy as np


zed = sl.Camera()

init_params = sl.InitParameters()
init_params.camera_resolution = sl.RESOLUTION.VGA # Use HD720 opr HD1200 video mode, depending on camera type.
init_params.camera_fps = 30  # Set fps 30, depend on rospy rate

# Open the camera
err = zed.open(init_params)
if err != sl.ERROR_CODE.SUCCESS:
    print("Camera Open : "+repr(err)+". Exit program.")
    exit()

image = sl.Mat()
depth_zed =sl.Mat(672, 376, sl.MAT_TYPE.F32_C1)
depth_image_zed = sl.Mat(672, 376, sl.MAT_TYPE.U8_C4)

runtime_parameters = sl.RuntimeParameters()

if __name__ == '__main__':
    rospy.init_node('zed_node')
    rospy.loginfo("zed capture node start")

    rate = rospy.Rate(30)
    pub_image = rospy.Publisher('/zed_node/image', Image, queue_size=10)
    pub_depth = rospy.Publisher('/zed_node/depth', Image, queue_size=10)

    while not rospy.is_shutdown():

        # Grab an image, a RuntimeParameters object must be given to grab()
        if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
            # Retrieve RGB data
            zed.retrieve_image(image, sl.VIEW.LEFT)

            # Retrieve the normalized depth image
            zed.retrieve_image(depth_image_zed, sl.VIEW.DEPTH)

            # Retrieve depth data (32-bit)
            zed.retrieve_measure(depth_zed, sl.MEASURE.DEPTH)

            # fps read
            current_fps = zed.get_current_fps()

            # Load capture data
            depth_ocv = depth_zed.get_data()
            image_ndarray = image.get_data()
            
            # # image - 3d array [ 376, 672, 4] , but need [ 376, 672, 3] --------------------
            image_ndarray = np.delete(image_ndarray, 3, axis=2)
            print(np.shape(image_ndarray))

            bridge = CvBridge()
            imgMsg = bridge.cv2_to_imgmsg(image_ndarray,"bgr8")
            pub_image.publish(imgMsg)

            depMsg = bridge.cv2_to_imgmsg(depth_ocv, "32FC1")
            pub_depth.publish(depMsg)


            cv2.imshow("new", image_ndarray)
            # cv2.imshow("depth", depth_ocv/5000)

            print("frames per second : ", current_fps)

            if cv2.waitKey(1) & 0XFF == ord("q"):
                zed.close()
                break

        else:
            rospy.logerr("zed grab fail")
        
        rate.sleep()
    
    zed.close()

    rospy.spin()
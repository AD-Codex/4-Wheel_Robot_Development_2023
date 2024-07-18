#!/usr/bin/env python

import rospy
import random
from std_msgs.msg import Float64MultiArray
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from numpy import np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import math


shape = (376, 672)
depth_cap = np.zeros(shape)
depth_success = False

lane_coord = []
mid_coord = []
world_mid_coord = []
world_lane_coord = []

lane_marker = Marker()
mid_marker = Marker()


def lane_marker_init():
    global lane_marker

    lane_marker.header.frame_id = "base_link"
    lane_marker.header.stamp = rospy.Time.now()
    lane_marker.ns = "random_points"
    lane_marker.id = 0
    lane_marker.type = Marker.POINTS
    lane_marker.action = Marker.ADD

    # Points parameters
    lane_marker.scale.x = 0.1  # Width of the points
    lane_marker.scale.y = 0.1  # Height of the points
    lane_marker.color.a = 1.0  # Alpha (transparency)
    lane_marker.color.r = 0.0  # Red
    lane_marker.color.g = 1.0  # Green
    lane_marker.color.b = 0.0  # Blue


def mid_marker_init():
    global mid_marker

    mid_marker.header.frame_id = "base_link"
    mid_marker.header.stamp = rospy.Time.now()
    mid_marker.ns = "random_points"
    mid_marker.id = 1
    mid_marker.type = Marker.POINTS
    mid_marker.action = Marker.ADD

    # Points parameters
    mid_marker.scale.x = 0.1  # Width of the points
    mid_marker.scale.y = 0.1  # Height of the points
    mid_marker.color.a = 1.0  # Alpha (transparency)
    mid_marker.color.r = 0.0  # Red
    mid_marker.color.g = 0.0  # Green
    mid_marker.color.b = 1.0  # Blue



def depth_read(msg: Image):
    global depth_cap
    global depth_success

    bridge = CvBridge()
    orig = bridge.imgmsg_to_cv2(msg, "32FC1")
    depth_cap = orig
    depth_success = True


def coord_read(msg: Float64MultiArray):
    global lane_coord

    lenth = len(msg.data)
    coords = msg.data
    x_coords = coords[: int(lenth/2)]
    y_coords = coords[int(lenth/2) :]

    for i in range(len(x_coords)):
        lane_coord.append([x_coords[i], y_coords[i]])

    return lane_coord


def mid_read(msg: Float64MultiArray):
    global mid_coord
    
    lenth = len(msg.data)
    coords = msg.data
    x_coords = coords[: int(lenth/2)]
    y_coords = coords[int(lenth/2) :]

    for i in range(len(x_coords)):
        mid_coord.append([x_coords[i], y_coords[i]])

    return mid_coord


# depth - pixel depth value, P_X - pixel x value, P_Y - pixel y value
# z - vertical, x - hrizontal depth, y - hrizontal width 
def yxz(depth, p_X, p_Y):
    x_theta = math.atan( 320/(abs(p_X)*1.732))
    z_theta = math.atan( 320/(abs(p_Y)*1.732))
    y = depth * math.tan( x_theta)
    x = depth
    z = - depth * math.tan( z_theta)

    if ( p_X <= 0):
        x = -x
    if ( p_Y <= 0):
        z = -z

    # print(y, x, z)
    return (y,x,z)


def world_coord_generate():
    global depth_cap
    global mid_coord
    global lane_coord
    global world_mid_coord
    global world_lane_coord

    # world mid point generate
    for x,y in mid_coord:
        depth = depth_cap[y,x+16]
        world_mid_coord.append( yxz(depth, x, y))

    # world lane point generate
    for x,y in lane_coord:
        depth = depth_cap[y,x+16]
        world_lane_coord.append( yxz(depth, x, y))



if __name__ == '__main__':
    rospy.init_node('foxglove_lane_node')
    rospy.loginfo("foxglove lane monitor node start")

    rate = rospy.Rate(2)  # 10 Hz (0.5 seconds)

    sub_depth = rospy.Subscriber("/zed_node/depth", Image, callback=depth_read)
    sub_laneCoords = rospy.Subscriber("/lane_node/lane_coord", Float64MultiArray, callback=coord_read)
    sub_midCoords = rospy.Subscriber("/lane_node/mid_coord", Float64MultiArray, callback=mid_read)
    pub_lanePoints = rospy.Publisher("/foxglove_node/lane_points", Marker, queue_size=10)
    pub_midPoints = rospy.Publisher("/foxglove_node/mid_points", Marker, queue_size=10)

    lane_marker_init()
    mid_marker_init()

    while not rospy.is_shutdown():

        # clear thr marker
        lane_marker.action = Marker.DELETEALL
        pub_lanePoints.publish(lane_marker)
        mid_marker.action = Marker.DELETEALL
        pub_midPoints.publish(mid_marker)

        lane_marker.action = Marker.ADD
        mid_marker.action = Marker.ADD

        lane_marker.points = []
        mid_marker.points = []

        if depth_success :

            for i in range(len(world_lane_coord)):
                p = Point()
                p.x = world_lane_coord[i][0]
                p.y = world_lane_coord[i][1]
                p.z = world_lane_coord[i][1]
                lane_marker.points.append(p)

            for i in range(len(world_mid_coord)):
                p = Point()
                p.x = world_mid_coord[i][0]
                p.y = world_mid_coord[i][1]
                p.z = world_mid_coord[i][2]
                mid_marker.points.append(p)

            depth_success = False

        lane_marker.header.stamp = rospy.Time.now()
        pub_lanePoints.publish(lane_marker)
        mid_marker.header.stamp = rospy.Time.now()
        pub_midPoints.publish(mid_marker)

        rate.sleep()
#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

def publish_plane_from_points():
    rospy.init_node('plane_publisher', anonymous=True)
    pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz

    marker = Marker()
    marker.header.frame_id = "base_link"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "plane"
    marker.id = 0
    marker.type = Marker.TRIANGLE_LIST
    marker.action = Marker.ADD

    # Plane color
    marker.color.a = 1.0  # Alpha (transparency)
    marker.color.r = 0.0  # Red
    marker.color.g = 1.0  # Green
    marker.color.b = 0.0  # Blue

    # Define the plane's vertices (example vertices)
    mask_points = [
        {"x": 0, "y": 0, "z": 0},
        {"x": 5, "y": 3, "z": 0},
        {"x": 1, "y": 5, "z": 0},
        {"x": 3, "y": 0, "z": 0},
        # Add more points as needed to form triangles
    ]

    # Assuming mask_points are in a counterclockwise order for each triangle
    triangles = [
        (mask_points[0], mask_points[1], mask_points[2]),
        (mask_points[2], mask_points[3], mask_points[0]),
    ]

    for triangle in triangles:
        for vertex in triangle:
            p = Point()
            p.x = vertex["x"]
            p.y = vertex["y"]
            p.z = vertex["z"]
            marker.points.append(p)

    while not rospy.is_shutdown():
        marker.header.stamp = rospy.Time.now()

        clear_marker = Marker()
        clear_marker.action = Marker.DELETEALL
        pub.publish(clear_marker)

        pub.publish(marker)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_plane_from_points()
    except rospy.ROSInterruptException:
        pass

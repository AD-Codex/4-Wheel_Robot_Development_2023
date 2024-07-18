#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

def generate_closed_loop_from_edge_points(edge_points):
    # Create a closed loop by connecting the last point back to the first point
    if edge_points:
        closed_loop = edge_points + [edge_points[0]]
        return closed_loop
    else:
        return []

def publish_plane_from_edge_points(edge_points):
    rospy.init_node('plane_publisher', anonymous=True)
    pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
    rate = rospy.Rate(2)  # 2 Hz (0.5 seconds)

    while not rospy.is_shutdown():
        # Clear existing markers
        clear_marker = Marker()
        clear_marker.action = Marker.DELETEALL
        pub.publish(clear_marker)

        # Points marker
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "plane_from_edge_points"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.05  # Width of the line
        marker.color.a = 1.0  # Alpha (transparency)
        marker.color.r = 0.0  # Red
        marker.color.g = 1.0  # Green
        marker.color.b = 0.0  # Blue

        # Generate closed loop from edge points
        closed_loop = generate_closed_loop_from_edge_points(edge_points)
        
        # Add points to marker
        for point in closed_loop:
            p = Point()
            p.x = point["x"]
            p.y = point["y"]
            p.z = point["z"]
            marker.points.append(p)

        # Ensure the loop is closed (first point added again)
        if closed_loop:
            first_point = closed_loop[0]
            p = Point()
            p.x = first_point["x"]
            p.y = first_point["y"]
            p.z = first_point["z"]
            marker.points.append(p)

        marker.header.stamp = rospy.Time.now()
        pub.publish(marker)
        rate.sleep()

if __name__ == '__main__':
    # Example edge points (replace with your actual edge points)
    edge_points = [
        {"x": 0, "y": 0, "z": 0},
        {"x": 1, "y": 0, "z": 0},
        {"x": 1, "y": 1, "z": 0},
        {"x": 0, "y": 1, "z": 0},
        {"x": 0, "y": 0, "z": 0},  # Closing the loop
    ]

    try:
        publish_plane_from_edge_points(edge_points)
    except rospy.ROSInterruptException:
        pass

#!/usr/bin/env python

import rospy
import random
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

def publish_random_points():
    rospy.init_node('random_points_publisher', anonymous=True)
    pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
    rate = rospy.Rate(5)  # 2 Hz (0.5 seconds)

    marker = Marker()
    marker.header.frame_id = "base_link"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "random_points"
    marker.id = 0
    marker.type = Marker.POINTS
    marker.action = Marker.ADD

    # Points parameters
    marker.scale.x = 0.1  # Width of the points
    marker.scale.y = 0.1  # Height of the points
    marker.color.a = 1.0  # Alpha (transparency)
    marker.color.r = 0.0  # Red
    marker.color.g = 1.0  # Green
    marker.color.b = 0.0  # Blue

    while not rospy.is_shutdown():
        marker.points = []
        
        # Generate 1000 random points
        for _ in range(1000):
            p = Point()
            p.x = random.uniform(-5, 5)
            p.y = random.uniform(-5, 5)
            p.z = 0
            marker.points.append(p)

        marker.header.stamp = rospy.Time.now()
        pub.publish(marker)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_random_points()
    except rospy.ROSInterruptException:
        pass

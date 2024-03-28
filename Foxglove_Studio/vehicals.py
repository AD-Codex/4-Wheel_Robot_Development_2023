import rospy
from geometry_msgs.msg import Point, Quaternion
from tf.transformations import *
from visualization_msgs.msg import Marker

rospy.init_node('vehicals_marker')

marker_pub = rospy.Publisher("/vehicals", Marker, queue_size = 2)


vehicals = Marker()

vehicals.header.frame_id = "base_link"
vehicals.header.stamp = rospy.Time.now()
vehicals.ns = ""

# Shape (mesh resource type - 10)
vehicals.type = Marker.CUBE
vehicals.id = 0
vehicals.action = 0

# Scale
vehicals.scale.x = 1.1
vehicals.scale.y = 1.7
vehicals.scale.z = 0.8

# Color
vehicals.color.r = 1.0
vehicals.color.g = 0.0
vehicals.color.b = 0.0
vehicals.color.a = 1.0

# Pose
vehicals.pose.position.x = -2
vehicals.pose.position.y = -5
vehicals.pose.position.z = 0.4

# matrix shift
q_orig = quaternion_from_euler(0, 0, 0)
q_rot = quaternion_from_euler(3.14159, 0, 0)
q_new = quaternion_multiply(q_rot, q_orig)
print(q_new)

vehicals.pose.orientation.x = q_new[0]
vehicals.pose.orientation.y = q_new[1]
vehicals.pose.orientation.z = q_new[2]
vehicals.pose.orientation.w = q_new[3]

i=-5



while not rospy.is_shutdown():
    print("vehicals node running ------------------")

    vehicals.pose.position.x = -2
    vehicals.pose.position.y = i
    vehicals.pose.position.z = 0.4

    marker_pub.publish(vehicals)

    i=i+0.2

    if i > 5 :
        i = -5

    rospy.rostime.wallsleep(0.05)
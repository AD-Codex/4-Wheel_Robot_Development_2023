import rospy
from geometry_msgs.msg import Point, Quaternion
from tf.transformations import *
from visualization_msgs.msg import Marker

rospy.init_node('person_marker')

marker_pub = rospy.Publisher("/object", Marker, queue_size = 2)

car = Marker()

car.header.frame_id = "base_link"
car.header.stamp = rospy.Time.now()
car.ns = ""

# Shape (mesh resource type - 10)
car.type = Marker.CYLINDER
car.id = 0
car.action = Marker.ADD

# Scale
car.scale.x = 0.5
car.scale.y = 1
car.scale.z = 1

# Color
car.color.r = 0.0
car.color.g = 0.0
car.color.b = 1.0
car.color.a = 1.0

# Pose
car.pose.position.x = 3
car.pose.position.y = 3
car.pose.position.z = 0.5

# matrix shift
q_orig = quaternion_from_euler(0, 0, 0)
q_rot = quaternion_from_euler(3.14159, 0, 0)
q_new = quaternion_multiply(q_rot, q_orig)
print(q_new)

car.pose.orientation.x = q_new[0]
car.pose.orientation.y = q_new[1]
car.pose.orientation.z = q_new[2]
car.pose.orientation.w = q_new[3]

i=0

while not rospy.is_shutdown():
    print("hello")

    q_orig = quaternion_from_euler(0, 0, i)
    q_rot = quaternion_from_euler(3.14159, 0, 0)
    q_new = quaternion_multiply(q_rot, q_orig)
    print(q_new)

    car.pose.orientation.x = q_new[0]
    car.pose.orientation.y = q_new[1]
    car.pose.orientation.z = q_new[2]
    car.pose.orientation.w = q_new[3]


    marker_pub.publish(car)

    i=i+0.01

    rospy.rostime.wallsleep(0.05)

import rospy
from visualization_msgs.msg import Marker

rospy.init_node('car_marker')

marker_pub = rospy.Publisher("/car", Marker, queue_size = 2)

car = Marker()

car.header.frame_id = "base_link"
car.header.stamp = rospy.Time.now()
car.ns = ""

# Shape (mesh resource type - 10)
car.type = 10
car.id = 0
car.action = 0

# Note: Must set mesh_resource to a valid URL for a model to appear
car.mesh_resource = "file:///home/dell/Desktop/Foxglove_Studio/peugeot_206_glb.glb"
car.mesh_use_embedded_materials = True

# Scale
car.scale.x = 0.5
car.scale.y = 0.5
car.scale.z = 0.5

# Color
car.color.r = 0.0
car.color.g = 0.0
car.color.b = 0.0
car.color.a = 1.0

# Pose
car.pose.position.x = 0
car.pose.position.y = 0
car.pose.position.z = 0
car.pose.orientation.x = 0.0
car.pose.orientation.y = 0.0
car.pose.orientation.z = 0.0
car.pose.orientation.w = 1.0

while not rospy.is_shutdown():
    print("car node running ----------------")
    marker_pub.publish(car)

    rospy.rostime.wallsleep(0.05)
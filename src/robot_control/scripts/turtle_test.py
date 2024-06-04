#!/usr/bin/env python3

# # REMOTE TURTLEBOT TEST
# # add with pc acticity

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from sensor_msgs.msg import JoyFeedbackArray

turtel_linear_x = 0
turtel_linear_y = 0
turtel_angular_z = 0
turtel_mode = 0

read_msg = 1


def msg_callback(msg: String):
    global read_msg
    read_msg = int(msg.data)



# turtel bot callback fn
def pose_callback(msg: Pose):
    global turtel_linear_x
    global turtel_linear_y
    global turtel_angular_z

    msg_write = Twist()
    msg_write.linear.x = turtel_linear_x
    msg_write.linear.y = turtel_linear_y
    msg_write.angular.z = turtel_angular_z
    pub.publish(msg_write)



# joystick callback fn
def joy_callback(msg : Joy):
    global turtel_linear_x
    global turtel_linear_y
    global turtel_angular_z
    global turtel_mode

    if (msg.buttons[2] == 1):
        rospy.loginfo("Current mode arrows")
        turtel_mode = 1
    elif (msg.buttons[0] == 1):
        rospy.loginfo("Current mode Joystick")
        turtel_mode = 2


    if (turtel_mode == 0):
        rospy.loginfo("Please select mode")

    elif (turtel_mode == 1):
        if (msg.buttons[13] == 1):
            turtel_linear_x = 2.0
            turtel_linear_y = 0.0
        elif (msg.buttons[14] == 1):
            turtel_linear_x = -2.0
            turtel_linear_y = 0.0
        elif (msg.buttons[15] == 1):
            turtel_linear_x = 0.0
            turtel_linear_y = 2.0
        elif (msg.buttons[16] == 1):
            turtel_linear_x = 0.0
            turtel_linear_y = -2.0

    elif (turtel_mode == 2) :
        if (msg.axes[1]== 0):
            turtel_linear_x = 0
        else :
            turtel_linear_x = 2*msg.axes[1]

        if (msg.axes[3] == 0):
            turtel_angular_z = 0
        else :
            turtel_angular_z = 5*msg.axes[3]


    if (msg.buttons[4] == 1 or msg.buttons[6] == 1):
        turtel_linear_x = 0.0
        turtel_linear_y = 0.0




if __name__ == '__main__':
    rospy.init_node('turtle_node')
    rospy.loginfo("turtle node start")

    pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)
    sub = rospy.Subscriber("/turtle1/pose", Pose, callback=pose_callback)
    # Joy_sub = rospy.Subscriber("/joy", Joy, callback=joy_callback)
    msg_sub = rospy.Subscriber("/pc/avtice", String, callback=msg_callback)

    rate = rospy.Rate(20)

    old_msg = 0
    while not rospy.is_shutdown():
        if ( read_msg > old_msg):
            print("system go")
        else:
            print("system stop")

        old_msg = read_msg
        
        rate.sleep()

    rospy.spin()
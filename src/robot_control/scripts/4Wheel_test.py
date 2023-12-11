#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy


wheel_linear_x = 0
wheel_linear_z = 0
Wheel_angular_z = 0

total_err = 0
past_err = 0
curr_speed = 0

# joystick callback fn
def joy_callback(msg : Joy):
    global wheel_linear_x
    global wheel_linear_z
    global Wheel_angular_z

    wheel_linear_x = msg.axes[1]
    Wheel_angular_z = msg.axes[3]

    if ( msg.buttons[4] == 1 or msg.buttons[6] == 1) :
        wheel_linear_z = 1
    else :
        wheel_linear_z = 0


def joy_PID(joy_speed):
    global curr_speed
    global past_err
    global total_err

    Kp = 0.1
    Ki = 0.001
    Kd = 0

    err = joy_speed - curr_speed
    total_err = err + past_err
    diff = Kp*err + Ki*total_err + Kd*(err - past_err)

    out_speed = curr_speed + diff

    if ( abs(out_speed) < 10):
        out_speed = 0

    curr_speed = out_speed
    past_err = err

    return out_speed



if __name__ == '__main__':
    rospy.init_node('4Wheel_node')
    rospy.loginfo("4Wheel node start")

    pub = rospy.Publisher("/4_wheel/cmd_vel", Twist, queue_size=10)
    Joy_sub = rospy.Subscriber("/joy", Joy, callback=joy_callback)

    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        msg_write = Twist()
        msg_write.linear.x = joy_PID( wheel_linear_x*100)
        msg_write.angular.z = Wheel_angular_z*100

        msg_write.linear.z = wheel_linear_z
        msg = "in",wheel_linear_x*100 ,"out" , msg_write.linear.x
        rospy.loginfo(msg)

        pub.publish(msg_write)
        rate.sleep()

    rospy.spin()

   

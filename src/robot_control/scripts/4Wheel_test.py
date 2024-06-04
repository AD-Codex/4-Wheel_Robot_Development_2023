#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sensor_msgs.msg import Joy


wheel_linear_x = 0
wheel_linear_z = 0
Wheel_angular_z = 0

total_err = 0
past_err = 0
curr_speed = 0

curr_turnspeed = 0
past_turnerr = 0
total_turnerr = 0

read_msg = 1


def msg_callback(msg: String):
    global read_msg
    read_msg = int(msg.data)


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


def control_callback(msg : Twist):
    global wheel_linear_x
    global wheel_linear_z
    global Wheel_angular_z

    if wheel_linear_z == 0 :
        wheel_linear_x = msg.linear.x
        Wheel_angular_z = msg.angular.z
    else :
        wheel_linear_x = 0
        Wheel_angular_z = 0



def joy_PID(joy_speed):
    global curr_speed
    global past_err
    global total_err

    Kp = 0.2
    Ki = 0.01
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



def joy_PIDturn(joy_turn):
    global curr_turnspeed
    global past_turnerr
    global total_turnerr

    Kp = 0.1
    Ki = 0.01
    Kd = 0

    err = joy_turn - curr_turnspeed
    total_turnerr = err + past_turnerr
    diff = Kp*err + Ki*total_turnerr + Kd*(err - past_turnerr)

    out_turnspeed = curr_turnspeed + diff

    if ( abs(out_turnspeed) < 10):
        out_turnspeed = 0

    curr_turnspeed = out_turnspeed
    past_turnerr = err

    return out_turnspeed



if __name__ == '__main__':
    rospy.init_node('Wheel_run')
    rospy.loginfo("4Wheel node start")

    pub = rospy.Publisher("/4_wheel/cmd_vel", Twist, queue_size=10)
    
    # control_sub = rospy.Subscriber("/came_read/cmd_vel", Twist, callback=control_callback)

    Joy_sub = rospy.Subscriber("/joy", Joy, callback=joy_callback)

    rate = rospy.Rate(20)

    old_msg = 0


    while not rospy.is_shutdown():
        msg_write = Twist()

        msg_write.linear.x = joy_PID( wheel_linear_x * 300) 
        msg_write.angular.z =  joy_PIDturn(Wheel_angular_z *100)

        msg_write.linear.z = wheel_linear_z

        msg = "in",wheel_linear_x ,"out" , msg_write.linear.x
        # print(joy_PID( wheel_linear_x * 300))
        # print(joy_PIDturn( Wheel_angular_z *100))

        pub.publish(msg_write)

        rate.sleep()

    rospy.spin()

   

#!/usr/bin/env python3

# # follow the line
# # Joy PID turn and move
# # control PID, kp=5, ki=0, kd=0
# # liner_x = 80

import rospy
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

remote_mode = 0

displacement = 0
old_displacement = 0
total_displacemnt = 0

total_err = 0
past_err = 0
curr_speed = 0

curr_turnspeed = 0
past_turnerr = 0
total_turnerr = 0

wheel_linear_x = 0
wheel_linear_z = 0
Wheel_angular_z = 0



# joystick callback fn
def joy_callback(msg : Joy):
    global wheel_linear_x
    global wheel_linear_z
    global Wheel_angular_z
    global remote_mode

    wheel_linear_x = msg.axes[1]
    Wheel_angular_z = msg.axes[3]
    remote_mode = msg.buttons[5]

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


def displacement_read(msg : Int32MultiArray):
    global displacement

    if len(msg.data ) != 0 :
        lane_displacement_list = msg.data
        displacement = - lane_displacement_list[0]
        # print("displacement", displacement)
    else :
        displacement = 0


def PIDfollow():
    global displacement
    global old_displacement
    global total_displacemnt
    global Wheel_angular_z

    kp = 5
    ki = 0
    kd = 0

    ref_speed = kp*displacement + ki*total_displacemnt + kd*(displacement-old_displacement)
    # print("ref_speed", ref_speed)
    Wheel_angular_z = ref_speed
    if (Wheel_angular_z >150):
        Wheel_angular_z = 150
    elif (Wheel_angular_z < -150):
        Wheel_angular_z = -150

    total_displacemnt = total_displacemnt + displacement
    old_displacement = displacement




def joy_PID(joy_speed):
    global curr_speed
    global past_err
    global total_err

    Kp = 0.1
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




if __name__ == '__main__' :
    rospy.init_node('lane_follow_node')
    rospy.loginfo("Lane keep start")

    sub_displacement = rospy.Subscriber("/lane_detect/line_displace", Int32MultiArray, callback=displacement_read)
    pub_move = rospy.Publisher("/4_wheel/cmd_vel", Twist, queue_size=50)
    Joy_sub = rospy.Subscriber("/joy", Joy, callback=joy_callback)

    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        msg_write = Twist()

        if (remote_mode == 0):
            msg_write.linear.x = joy_PID( wheel_linear_x * 200) 
            msg_write.angular.z = joy_PIDturn(Wheel_angular_z *100)

            msg_write.linear.z = wheel_linear_z

        else:
            PIDfollow()
            
            msg_write.linear.x = 120
            msg_write.angular.z = Wheel_angular_z
            msg_write.linear.z = 0

        # PIDfollow()
            
        # msg_write.linear.x = 50
        # msg_write.angular.z = Wheel_angular_z
        # msg_write.linear.z = 0

        print("displacement :", displacement, " turnSpeed :", msg_write.angular.z, " moveSpeed :", msg_write.linear.x)

        pub_move.publish(msg_write)

        rate.sleep()

    rospy.spin()
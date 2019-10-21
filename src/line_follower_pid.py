#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from kobuki_msgs.msg import SensorState
import random

RATE = 20

SPEED = .3
TURNINGSPEED = -.15
ROTSPEED = .5

vel = Twist()

RIGHT = 0
CENTER = STRAIGHT = 1
LEFT = 2

RIGHT_THRESHOLD = 1460
CENTER_THRESHOLD = 1700
LEFT_THRESHOLD = 1560

direct = STRAIGHT

Kp = 0.75
Ki = 0.5
Kd = -0.01
dt = 1.0/RATE

odom_speed = 0.0
prev_error = 0.0
i_error = 0.0


def odom_callback(data):
    global odom_speed
    odom_speed = data.twist.twist.linear.x


def pid_speed(target_speed):
    global prev_error, i_error
    error = target_speed - odom_speed
    d_error = (error - prev_error) / dt
    i_error = i_error + error * dt
    out_speed = Kp*error + Ki*i_error + Kd*d_error
    prev_error = error
    return out_speed


def sensor_callback(data):
    global direct
    detect_right = data.bottom[RIGHT] < RIGHT_THRESHOLD
    detect_center = data.bottom[CENTER] < CENTER_THRESHOLD
    detect_left = data.bottom[LEFT] < LEFT_THRESHOLD
    if detect_right: print('detect right')
    if detect_left: print('detect left')
    if detect_center: print('detect center')
    if direct == STRAIGHT:
        if detect_right:
            direct = RIGHT
        if detect_left:
            direct = LEFT
    elif direct == LEFT:
        if detect_center or detect_right:
            direct = STRAIGHT
    elif direct == RIGHT:
        if detect_center or detect_left:
            direct = STRAIGHT

def main():
    global direct
    rospy.init_node('joy_teleop', anonymous=True)
    vel_pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=1)
    rospy.Subscriber("/odom", Odometry, odom_callback)
    rospy.Subscriber("/mobile_base/sensors/core", SensorState, sensor_callback)
    rate = rospy.Rate(RATE)
    direct = STRAIGHT
    vel.linear.x = SPEED
    vel.angular.z = 0
    while not rospy.is_shutdown():
        if direct == STRAIGHT:
            vel.angular.z = 0
            vel.linear.x = pid_speed(SPEED)
        elif direct == LEFT:
            vel.angular.z = ROTSPEED
            vel.linear.x = pid_speed(TURNINGSPEED)
        elif direct == RIGHT:
            vel.angular.z = -ROTSPEED
            vel.linear.x = pid_speed(TURNINGSPEED)
        vel_pub.publish(vel)
        rate.sleep()

if __name__ == "__main__":
    main()


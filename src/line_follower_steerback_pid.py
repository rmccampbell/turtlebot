#!/usr/bin/env python
from __future__ import division, print_function
import sys
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from kobuki_msgs.msg import SensorState, BumperEvent
import random

RATE = 20.0
DT = 1.0/RATE

SPEED = .3
TURNINGSPEED = .1
ROTSPEED = .75
STEERBACK_ROTSPEED = .75
MAX_STEERBACK_TIMER = 1.0

vel = Twist()

RIGHT = 0
CENTER = STRAIGHT = 1
LEFT = 2

RIGHT_THRESHOLD = 1460
CENTER_THRESHOLD = 1700
LEFT_THRESHOLD = 1560

direct = STRAIGHT
steerback_timer = 0

Kp = 0.25
Ki = 0.75
Kd = 0.0
dt = 1.0/RATE

odom_speed = 0.0
prev_error = 0.0
i_error = 0.0

shutdown = False

def bumper_callback(data):
    global shutdown
    shutdown = True

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
    global direct, steerback_timer
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
        if (detect_center and steerback_timer <= 0) or detect_right:
            direct = RIGHT
            steerback_timer = MAX_STEERBACK_TIMER
    elif direct == RIGHT:
        if (detect_center and steerback_timer <= 0) or detect_left:
            direct = LEFT
            steerback_timer = MAX_STEERBACK_TIMER
    if direct != STRAIGHT:
        i_error = 0.0
        prev_error = 0.0

def main():
    global direct, steerback_timer, i_error, prev_error
    rospy.init_node('joy_teleop', anonymous=True)
    vel_pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=1)
    rospy.Subscriber("/mobile_base/sensors/core", SensorState, sensor_callback)
    rospy.Subscriber("/odom", Odometry, odom_callback)
    rospy.Subscriber("/mobile_base/events/bumper", BumperEvent, bumper_callback)
    rate = rospy.Rate(RATE)
    direct = STRAIGHT
    vel.linear.x = SPEED
    vel.angular.z = 0
    while not (rospy.is_shutdown() or shutdown):
        rotspeed = ROTSPEED
        if steerback_timer > 0:
            print('steerback_timer', steerback_timer, 'direct', ['right','straight','left'][direct])
            rotspeed = STEERBACK_ROTSPEED
            steerback_timer -= DT
            if steerback_timer <= 0:
                direct = STRAIGHT
                steerback_timer = 0
        if direct == STRAIGHT:
            vel.angular.z = 0
            vel.linear.x = pid_speed(SPEED)
        elif direct == LEFT:
            vel.angular.z = rotspeed
            vel.linear.x = pid_speed(TURNINGSPEED)
        elif direct == RIGHT:
            vel.angular.z = -rotspeed
            vel.linear.x = pid_speed(TURNINGSPEED)
        vel_pub.publish(vel)
        rate.sleep()

if __name__ == "__main__":
    main()


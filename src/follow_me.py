#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from kobuki_msgs.msg import BumperEvent
from sensor_msgs.msg import LaserScan
import math
import random
import numpy as np

SPEED = 0.25
STEERING = 2.

RATE = 10.0
Kp = 0.25
Ki = 0.75
Kd = 0.0
dt = 1.0/RATE

vel = Twist()
odom_speed = 0.0
odom_pos = None
ranges = None
angles = None
goal = None
stop = False

threshold_min = 0.2
threshold_max = .8

def bumper_callback(data):
    global stop
    stop = True

def odom_callback(data):
    global odom_speed, odom_pos, odom_angle, start_pos
    odom_speed = data.twist.twist.linear.x
    odom_pos = data.pose.pose.position
    odom_angle = 2*math.asin(data.pose.pose.orientation.z)

def scan_callback(data):
    global ranges, angles
    ranges = np.array(data.ranges)
    angles = np.linspace(data.angle_min, data.angle_max, len(ranges))

def sense_nearest(ranges, angles):
    # In this function, we select several nearest obstacle and calculate the angle
    mask = ~np.isnan(ranges) & (ranges < threshold_max) & (ranges > threshold_min)
    if not np.any(mask):
        return 0
    estimate_angle = np.mean(angles[mask])
    return estimate_angle

# def pid_speed(target_speed):
#     global prev_error, i_error
#     error = target_speed - odom_speed
#     d_error = (error - prev_error) / dt
#     i_error = i_error + error * dt
#     out_speed = Kp*error + Ki*i_error + Kd*d_error
#     prev_error = error
#     return out_speed

def main():
    global vel, prev_error, i_error, goal
    rospy.init_node('joy_teleop', anonymous=True)
    vel_pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=1)
    rospy.Subscriber("/odom", Odometry, odom_callback)
    rospy.Subscriber("/scan", LaserScan, scan_callback)
    rospy.Subscriber("/mobile_base/events/bumper", BumperEvent, bumper_callback)

    rate = rospy.Rate(RATE)
    vel.linear.x = 0
    vel.angular.z = 0
    while not rospy.is_shutdown() and not stop:
        if ranges is None or angles is None:
            continue

        theta = sense_nearest(ranges, angles)
        print('theta:', theta)

        vel.angular.z = STEERING * theta
        vel.linear.x = SPEED

        vel_pub.publish(vel)
        rate.sleep()

if __name__ == "__main__":
    main()
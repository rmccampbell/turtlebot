#!/usr/bin/env python
from __future__ import print_function

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from kobuki_msgs.msg import BumperEvent
from sensor_msgs.msg import LaserScan
from apriltag_ros.msg import AprilTagDetectionArray
import math
import random
import numpy as np

SPEED = 0.15
TURNSPEED = 0.1
STEERING = .5
MAXSTEER = .5

NUM_LASERS = 640

RATE = 10.0
Kp = 0.25
Ki = 0.75
Kd = 0.0
dt = 1.0/RATE

vel = Twist()
start_angle = None
odom_angle = 0.0
odom_speed = 0.0
odom_pos = None
ranges = None
angles = None
stop = False

target_dir = 0

def norm_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))

def bumper_callback(data):
    global stop
    stop = True
    print('Bumper hit, stopping')

def odom_callback(data):
    global odom_speed, odom_pos, odom_angle, start_angle, target_dir
    odom_speed = data.twist.twist.linear.x
    odom_pos = data.pose.pose.position
    odom_angle = 2*math.asin(data.pose.pose.orientation.z)
    if start_angle is None:
        start_angle = odom_angle
        target_dir = odom_angle

def scan_callback(data):
    global ranges, angles
    ranges = np.array(data.ranges)
    angles = np.linspace(data.angle_min, data.angle_max, len(ranges)) + odom_angle

def apriltag_callback(data):
    global target_dir, stop
    if len(data.detections) > 0:
        detect = data.detections[0]
        detect_dist = detect.pose.pose.pose.position.z
        print('detect:', detect.id[0], 'distance:', detect_dist)
        if detect_dist < .75:
            if detect.id[0] == 9:
                stop = True
                return
            target_dir = norm_angle(detect.id[0] * math.pi/4 + start_angle)
            print('target_dir:', math.degrees(target_dir), 'deg')

# def pid_speed(target_speed):
#     global prev_error, i_error
#     error = target_speed - odom_speed
#     d_error = (error - prev_error) / dt
#     i_error = i_error + error * dt
#     out_speed = Kp*error + Ki*i_error + Kd*d_error
#     prev_error = error
#     return out_speed

def compute_force(target_dir):
    global ranges, angles
    dx = np.cos(angles)
    dy = np.sin(angles)
    ranges = np.maximum(ranges - .25, 0.05)
    grad_x = dx/ranges
    grad_y = dy/ranges
    mask = ~np.isnan(ranges) & (ranges < .3)# & (abs(angles) < .4)
    f_rep_x = -np.sum(grad_x[mask]) / NUM_LASERS
    f_rep_y = -np.sum(grad_y[mask]) / NUM_LASERS
    # print('f_rep_x', f_rep_x, 'f_rep_y', f_rep_y)
    # print('ranges', ranges)
    # f_rep_pub.publish(Vector3(f_rep_x, f_rep_y, 0))

    f_att_x = math.cos(target_dir)
    f_att_y = math.sin(target_dir)
    # print('f_att_x', f_att_x, 'f_att_y', f_att_y)
    # f_att_pub.publish(Vector3(f_att_x, f_att_y, 0))
    f_x = 20 * f_rep_x + f_att_x
    f_y = 20 * f_rep_y + f_att_y

    f_mag = math.sqrt(f_x**2 + f_y**2)
    f_theta = math.atan2(f_y, f_x)
    return f_mag, f_theta

def main():
    global vel, prev_error, i_error, goal
    rospy.init_node('tag_follower', anonymous=True)
    vel_pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=1)
    rospy.Subscriber("/odom", Odometry, odom_callback)
    rospy.Subscriber("/scan", LaserScan, scan_callback)
    rospy.Subscriber("/mobile_base/events/bumper", BumperEvent, bumper_callback)
    rospy.Subscriber("/tag_detections", AprilTagDetectionArray, apriltag_callback)

    rate = rospy.Rate(RATE)
    vel.linear.x = 0
    vel.angular.z = 0
    while not rospy.is_shutdown() and not stop:
        if ranges is None or angles is None:
            continue

        f_mag, f_theta = compute_force(target_dir)
        angle_err = norm_angle(f_theta - odom_angle)

        vel.angular.z = min(max(STEERING * angle_err, -MAXSTEER), MAXSTEER)
        # vel.linear.x = SPEED * (1 - abs(angle_err)/(2*math.pi))
        vel.linear.x = SPEED if abs(angle_err) < math.pi/8 else TURNSPEED

        vel_pub.publish(vel)
        rate.sleep()

if __name__ == "__main__":
    main()

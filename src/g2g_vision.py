#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
from kobuki_msgs.msg import BumperEvent
from sensor_msgs.msg import LaserScan
import math
import random
import numpy as np

SPEED = 0.25
STEERING = 1.

NUM_POINTS = 640

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

eta_repl = 10
zeta_attr = 1
dist_repl_threshold = 0.1
dist_goal_threshold = 0.5

def bumper_callback(data):
    global stop
    if data.bumper == data.CENTER:
        stop = True

def odom_callback(data):
    global odom_speed, odom_pos, odom_angle, start_pos, goal
    odom_speed = data.twist.twist.linear.x
    odom_pos = data.pose.pose.position
    odom_angle = 2*math.asin(data.pose.pose.orientation.z)
    if goal is None:
        goal = (odom_pos.x + 4*math.cos(odom_angle),
                odom_pos.y + 4*math.sin(odom_angle))

def scan_callback(data):
    global ranges, angles
    ranges = np.array(data.ranges)
    angles = np.linspace(data.angle_min, data.angle_max, len(ranges))

# def pid_speed(target_speed):
#     global prev_error, i_error
#     error = target_speed - odom_speed
#     d_error = (error - prev_error) / dt
#     i_error = i_error + error * dt
#     out_speed = Kp*error + Ki*i_error + Kd*d_error
#     prev_error = error
#     return out_speed

def compute_force(goalx, goaly):
    global ranges, angles
    dx = np.cos(angles)
    dy = np.sin(angles)
    ranges = np.maximum(ranges - .25, 0.05)
    grad_x = dx/ranges
    grad_y = dy/ranges
    mask = ~np.isnan(ranges) & (ranges < .75)# & (abs(angles) < .4)
    f_rep_x = -np.sum(grad_x[mask]) / NUM_POINTS
    f_rep_y = -np.sum(grad_y[mask]) / NUM_POINTS
    print('f_rep_x', f_rep_x, 'f_rep_y', f_rep_y)
    #print('ranges', ranges)
    f_rep_pub.publish(Vector3(f_rep_x, f_rep_y, 0))

    goalr = math.sqrt(goalx**2 + goaly**2)
    goalth = math.atan2(goaly, goalx)
    f_att_x = math.cos(goalth)*goalr**.5
    f_att_y = math.sin(goalth)*goalr**.5
    print('f_att_x', f_att_x, 'f_att_y', f_att_y)
    f_att_pub.publish(Vector3(f_att_x, f_att_y, 0))
    f_x = 20 * f_rep_x + f_att_x
    f_y = 20 * f_rep_y + f_att_y
    
    return f_x, f_y

def main():
    global vel, prev_error, i_error, goal, f_att_pub, f_rep_pub
    rospy.init_node('joy_teleop', anonymous=True)
    vel_pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=1)
    f_att_pub = rospy.Publisher('/f_att', Vector3, queue_size=1)
    f_rep_pub = rospy.Publisher('/f_rep', Vector3, queue_size=1)
    rospy.Subscriber("/odom", Odometry, odom_callback)
    rospy.Subscriber("/scan", LaserScan, scan_callback)
    rospy.Subscriber("/mobile_base/events/bumper", BumperEvent, bumper_callback)

    rate = rospy.Rate(RATE)
    vel.linear.x = 0
    vel.angular.z = 0

    

    while not rospy.is_shutdown() and not stop:
        if goal is None or ranges is None or angles is None:
            continue
        transform = np.array([[np.cos(odom_angle), -np.sin(odom_angle), odom_pos.x],
                              [np.sin(odom_angle),  np.cos(odom_angle), odom_pos.y],
                              [0, 0, 1]])
        i_transform = np.linalg.inv(transform)
        goalx, goaly, _ = i_transform.dot([goal[0], goal[1], 1])
        print("goal[0]", goal[0], "odom_pos x", odom_pos.x, "odom_pos y", odom_pos.y)
        print("out_angle", odom_angle)
        print('goalx:', goalx, 'goaly:', goaly)

        # if abs(goalx) < 1 and abs(goaly) < 1:
        #     break

        if abs(math.sqrt(goaly**2 + goalx**2)) < dist_goal_threshold:
            break

        f_x, f_y = compute_force(goalx, goaly)
        f_mag = math.sqrt(f_x**2 + f_y**2)
        f_theta = math.atan2(f_y, f_x)
        print('f_x: {}, f_y: {}, f_mag: {}, f_theta: {}'.format(f_x, f_y, f_mag, f_theta))

        # if abs(f_theta) < 0.5 * math.pi:
        #     vel.angular.z = STEERING * -1 * (0.5 * math.pi - abs(f_theta))
        # else:
        #     vel.angular.z = STEERING * (abs(f_theta) - 0.5 * math.pi)
        
        # if abs(f_theta) > 0.25:
        #     vel.angular.z = STEERING * f_theta
        #     vel.linear.x = SPEED * 2
        # else:
        #     vel.angular.z = STEERING * f_theta
        #     vel.linear.x = SPEED

        vel.angular.z = STEERING * f_theta
        vel.linear.x = SPEED * 0.8# * f_mag * (1 - f_theta/math.pi)

        vel_pub.publish(vel)
        rate.sleep()

if __name__ == "__main__":
    main()

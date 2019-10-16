#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from kobuki_msgs.msg import BumperEvent
import math
import random

SPEED = 0.25
STEERING = 1.0

GOALX = 1.5
GOALY = 1.5

RATE = 10.0

vel = Twist()
odom_speed = 0.0
start_pos = None

def odom_callback(data):
    global odom_speed, odom_pos, odom_angle, start_pos
    odom_speed = data.twist.twist.linear.x
    odom_pos = data.pose.pose.position
    odom_angle = 2*math.asin(data.pose.pose.orientation.z)
    if start_pos is None:
        start_pos = odom_pos

def main():
    global vel, prev_error, i_error
    rospy.init_node('joy_teleop', anonymous=True)
    vel_pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=1)
    rospy.Subscriber("/odom", Odometry, odom_callback)

    rate = rospy.Rate(RATE)
    vel.linear.x = 0
    vel.angular.z = 0
    while not rospy.is_shutdown():
        if start_pos is None:
            continue
        dx = start_pos.x + GOALX - odom_pos.x
        dy = start_pos.y + GOALY - odom_pos.y
        if abs(dx) < .1 and abs(dy) < .1:
            break
        goal_theta = math.atan2(dy, dx)
        
        theta_e = goal_theta - odom_angle
        theta_e = math.atan2(math.sin(theta_e), math.cos(theta_e))
        
        vel.angular.z = STEERING * theta_e

        vel.linear.x = SPEED * math.sqrt(dx**2 + dy**2)

        vel_pub.publish(vel)
        rate.sleep()

if __name__ == "__main__":
    main()

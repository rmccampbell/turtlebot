#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from kobuki_msgs.msg import BumperEvent
import math
import random

SPEED = 0.25
STEERING = 1.0

GOALS = [(0, 0), (1.5, 0)]

RATE = 10.0

vel = Twist()
odom_speed = 0.0
start_pos = None
goal_ind = 0


Kp = 0.5
Ki = 0.4
Kd = -0.01
dt = 1.0/RATE
prev_error = 0.0
i_error = 0.0

def odom_callback(data):
    global odom_speed, odom_pos, odom_angle, start_pos
    odom_speed = data.twist.twist.linear.x
    odom_pos = data.pose.pose.position
    odom_angle = 2*math.asin(data.pose.pose.orientation.z)
    if start_pos is None:
        start_pos = odom_pos

def pid_speed(target_speed):
    global prev_error, i_error
    error = target_speed - odom_speed
    d_error = (error - prev_error) / dt
    i_error = i_error + error * dt
    # rospy.loginfo("Odom Speed: %s", odom_speed)
    # rospy.loginfo("error: %s", error)
    # rospy.loginfo("d_error: %s", d_error)
    # rospy.loginfo("i_error: %s", i_error)
    out_speed = Kp*error + Ki*i_error + Kd*d_error
    prev_error = error
    # rospy.loginfo("out_speed: %s", out_speed)
    return out_speed


def main():
    global vel, prev_error, i_error, goal_ind
    rospy.init_node('joy_teleop', anonymous=True)
    vel_pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=1)
    rospy.Subscriber("/odom", Odometry, odom_callback)

    rate = rospy.Rate(RATE)
    vel.linear.x = 0
    vel.angular.z = 0
    while not rospy.is_shutdown():
        if start_pos is None:
            continue
        goalx, goaly = GOALS[goal_ind]
        dx = start_pos.x + goalx - odom_pos.x
        dy = start_pos.y + goaly - odom_pos.y
        goal_theta = math.atan2(dy, dx)
        print "Goal ind:", goal_ind
        
        theta_e = goal_theta - odom_angle
        theta_e = math.atan2(math.sin(theta_e), math.cos(theta_e))
        
        vel.angular.z = STEERING * theta_e
        vel.linear.x = pid_speed(SPEED)

        if abs(dx) < .05 and abs(dy) < .05:
            goal_ind = (goal_ind + 1) % len(GOALS)

        vel_pub.publish(vel)
        rate.sleep()

if __name__ == "__main__":
    main()

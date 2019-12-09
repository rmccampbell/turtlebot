#!/usr/bin/env python
from __future__ import print_function
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from kobuki_msgs.msg import BumperEvent
import random
from sensor_msgs.msg import LaserScan
import numpy

TURNSPEED = 2.0
SPEED = 0.3

RATE = 10.0
Kp = 0.25
Ki = 0.75
Kd = 0.0
dt = 1.0/RATE

vel = Twist()
odom_speed = 0.0
prev_error = 0.0
i_error = 0.0

bumpers = {BumperEvent.LEFT: 'LEFT', BumperEvent.RIGHT: 'RIGHT', BumperEvent.CENTER: 'CENTER'}

obstacle = False


def odom_callback(data):
    global odom_speed
    odom_speed = data.twist.twist.linear.x

def scan_callback(data):
    global obstacle
    minimum = numpy.nanmin(data.ranges)
    print("Minimum",minimum)    
    if minimum < 1:
        obstacle = True
    
      
        
def bumper_callback(data):
    global obstacle
    obstacle = True

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
    global vel, prev_error, i_error
    rospy.init_node('joy_teleop', anonymous=True)
    vel_pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=1)
    rospy.Subscriber("/odom", Odometry, odom_callback)
    rospy.Subscriber("/mobile_base/events/bumper", BumperEvent, bumper_callback)
    rospy.Subscriber("/scan", LaserScan, scan_callback)
    rate = rospy.Rate(RATE)
    vel.linear.x = 0
    vel.angular.z = 0
    while not rospy.is_shutdown():
        if obstacle:
            vel.linear.x = 0
        else:
            out_speed = pid_speed(SPEED)
            vel.linear.x = out_speed
        vel_pub.publish(vel)
        rate.sleep()

if __name__ == "__main__":
    main()


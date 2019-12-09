#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from kobuki_msgs.msg import BumperEvent
from sensor_msgs.msg import LaserScan, Image
import math
import random
import numpy as np
import matplotlib.pyplot as plt

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
image = None
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

def image_callback(data):
    global image
    array = np.fromstring(data.data, 'uint8')
    image = array.reshape(data.height, data.width, 3)

# def pid_speed(target_speed):
#     global prev_error, i_error
#     error = target_speed - odom_speed
#     d_error = (error - prev_error) / dt
#     i_error = i_error + error * dt
#     out_speed = Kp*error + Ki*i_error + Kd*d_error
#     prev_error = error
#     return out_speed

def sense_object():
    if image is None:
        return 0

    return 0

def show_image():
    if image is None:
        return
    print('showing image')
    plt.imshow(image)
    plt.pause(.01)
    print('after show image')

def main():
    global vel, prev_error, i_error, goal
    rospy.init_node('joy_teleop', anonymous=True)
    vel_pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=1)
    rospy.Subscriber("/odom", Odometry, odom_callback)
    rospy.Subscriber("/mobile_base/events/bumper", BumperEvent, bumper_callback)
    rospy.Subscriber("/camera/rgb/image_raw", Image, image_callback)

    rate = rospy.Rate(RATE)
    vel.linear.x = 0
    vel.angular.z = 0

    plt.ion()
    plt.subplot()

    counter = 0
    while not rospy.is_shutdown() and not stop:
        theta = sense_object()
        print('theta:', theta)

        if counter % 10 == 0:
            show_image()
        counter += 1

        # vel.angular.z = STEERING * theta
        # vel.linear.x = SPEED

        vel_pub.publish(vel)
        rate.sleep()

if __name__ == "__main__":
    main()

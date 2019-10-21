#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from kobuki_msgs.msg import SensorState
import random

RATE = 20

SPEED = .2
TURNINGSPEED = -.1
ROTSPEED = .5

vel = Twist()

RIGHT = 0
CENTER = STRAIGHT = 1
LEFT = 2

RIGHT_THRESHOLD = 1460
CENTER_THRESHOLD = 1700
LEFT_THRESHOLD = 1560

direct = STRAIGHT

#def imu_callback(data):
#    rospy.loginfo("Angular Velocity = %f", data.angular_velocity.z)

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
#    rospy.Subscriber("/mobile_base/sensors/imu_data", Imu, imu_callback)
    rospy.Subscriber("/mobile_base/sensors/core", SensorState, sensor_callback)
    rate = rospy.Rate(RATE)
    direct = STRAIGHT
    vel.linear.x = SPEED
    vel.angular.z = 0
    while not rospy.is_shutdown():
        if direct == STRAIGHT:
            vel.angular.z = 0
            vel.linear.x = SPEED
        elif direct == LEFT:
            vel.angular.z = ROTSPEED
            vel.linear.x = TURNINGSPEED
        elif direct == RIGHT:
            vel.angular.z = -ROTSPEED
            vel.linear.x = TURNINGSPEED
        vel_pub.publish(vel)
        rate.sleep()

if __name__ == "__main__":
    main()


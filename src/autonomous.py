#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from kobuki_msgs.msg import BumperEvent
import random

FORWARD = 0
TURNING = 1

SPEED = .25
TURNSPEED = 1

vel = Twist()
turntime = 0
state = FORWARD

bumpers = {BumperEvent.LEFT: 'LEFT', BumperEvent.RIGHT: 'RIGHT', BumperEvent.CENTER: 'CENTER'}

#def imu_callback(data):
#    rospy.loginfo("Angular Velocity = %f", data.angular_velocity.z)

def bumper_callback(data):
    global state, turntime, vel
    rospy.loginfo("Bumpber hit: %s: %s", bumpers[data.bumper], data.state)
    if state == FORWARD and data.state == 1:
        state = TURNING
        if data.bumper == data.LEFT:
            direction = -1
            turntime = 15
        elif data.bumper == data.RIGHT:
            direction = 1
            turntime = 15
        elif data.bumper == data.CENTER:
            direction = random.choice([-1, 1])
            turntime = 30
        vel.linear.x = 0
        vel.angular.z = TURNSPEED * direction

def main():
    global state, turntime, vel
    rospy.init_node('joy_teleop', anonymous=True)
    vel_pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=1)
#    rospy.Subscriber("/mobile_base/sensors/imu_data", Imu, imu_callback)
    rospy.Subscriber("/mobile_base/events/bumper", BumperEvent, bumper_callback)
    rate = rospy.Rate(10)
    state = FORWARD
    vel.linear.x = SPEED
    vel.angular.z = 0
    while not rospy.is_shutdown():
        if state == TURNING:
            if turntime > 0:
                turntime -= 1
            else:
                state = FORWARD
                vel.linear.x = SPEED
                vel.angular.z = 0
#        rospy.loginfo('state = ' + ['FORWARD', 'TURNING'][state])
        vel_pub.publish(vel)
        rate.sleep()

if __name__ == "__main__":
    main()


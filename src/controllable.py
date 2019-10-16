#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from kobuki_msgs.msg import BumperEvent
from sensor_msgs.msg import Joy
import random

FORWARD = 0
TURNING = 1
MANUAL = 3

TURNSPEED = 2.0
SPEED1 = 0.3
SPEED2 = 0.5
SPEED3 = 0.6

BLUE = 0
GREEN = 1
RED = 2
YELLOW = 3
LTRIGGER = 6

RATE = 10.0
Kp = 1.0
Ki = 0.4
Kd = -0.05
dt = 1.0/RATE

vel = Twist()
odom_speed = 0.0
target_speed = SPEED1
turntime = 0
state = FORWARD
prev_error = [0.0]*4
i_error = 0.0

bumpers = {BumperEvent.LEFT: 'LEFT', BumperEvent.RIGHT: 'RIGHT', BumperEvent.CENTER: 'CENTER'}

def joy_callback(data):
    global vx, vy, state, target_speed, vel, prev_error, i_error
    vx = data.axes[2]
    vy = data.axes[3]

    if data.buttons[GREEN]:
        target_speed = SPEED1
    if data.buttons[BLUE]:
        target_speed = SPEED2
    if data.buttons[YELLOW]:
        target_speed = SPEED3
    if data.buttons[RED]:
        target_speed = 0

    if state != MANUAL and data.buttons[LTRIGGER]:
        state = MANUAL
    elif state == MANUAL and not data.buttons[LTRIGGER]:
        state = FORWARD
        vel.angular.z = 0.0
        prev_error = [0.0]*4
        i_error = 0.0

def odom_callback(data):
    global odom_speed
    odom_speed = data.twist.twist.linear.x

def bumper_callback(data):
    global state, turntime, vel, target_speed
    rospy.loginfo("Bumpber hit: %s: %s", bumpers[data.bumper], data.state)
    if state == FORWARD and data.state == 1:
        state = TURNING
        if data.bumper == data.LEFT:
            direction = -1
            turntime = 8
        elif data.bumper == data.RIGHT:
            direction = 1
            turntime = 8
        elif data.bumper == data.CENTER:
            direction = random.choice([-1, 1])
            turntime = 15
        vel.linear.x = 0
        vel.angular.z = TURNSPEED * direction

def pid_speed():
    global prev_error, i_error
    avg_prev_error = sum(prev_error)/len(prev_error)
    error = target_speed - odom_speed
    d_error = (error - avg_prev_error) / dt
    i_error = i_error + error * dt
    rospy.loginfo("Odom Speed: %s", odom_speed)
    # rospy.loginfo("error: %s", error)
    # rospy.loginfo("d_error: %s", d_error)
    # rospy.loginfo("i_error: %s", i_error)
    out_speed = max(Kp*error + Ki*i_error + Kd*d_error, 0)
    del prev_error[0]
    prev_error.append(error)
    # rospy.loginfo("out_speed: %s", out_speed)
    return out_speed

def main():
    global state, turntime, vel, prev_error, i_error
    rospy.init_node('joy_teleop', anonymous=True)
    vel_pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=1)
    rospy.Subscriber("/odom", Odometry, odom_callback)
    rospy.Subscriber("/mobile_base/events/bumper", BumperEvent, bumper_callback)
    rospy.Subscriber("/joy", Joy, joy_callback)
    rate = rospy.Rate(RATE)
    state = FORWARD
    speed = target_speed
    vel.linear.x = speed
    vel.angular.z = 0
    while not rospy.is_shutdown():
        if state == MANUAL:
            vel.linear.x = target_speed*vy
            vel.angular.z = 2*(vx if vy >=0 else -vx)
        elif state == TURNING:
            if turntime > 0:
                turntime -= 1
            else:
                state = FORWARD
                vel.angular.z = 0.0
                prev_error = [0.0]*4
                i_error = 0.0
        elif state == FORWARD:
            out_speed = pid_speed()
            vel.linear.x = out_speed
#        rospy.loginfo('state = ' + ['FORWARD', 'TURNING'][state])
        vel_pub.publish(vel)
        rate.sleep()

if __name__ == "__main__":
    main()


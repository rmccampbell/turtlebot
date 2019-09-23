#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, Joy

vx = 0
vy = 0

def joy_callback(data):
    global vx, vy
    vx = data.axes[2]
    vy = data.axes[3]
    rospy.loginfo("x: %s", vx)
    rospy.loginfo("y: %s", vy)

def imu_callback(data):
    pass
#    rospy.loginfo("Angular Velocity = %f", data.angular_velocity.z)

def main():
    rospy.init_node('joy_teleop', anonymous=True)
    vel_pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=1)
#    rospy.Subscriber("/mobile_base/sensors/imu_data", Imu, imu_callback)
    rospy.Subscriber("/joy", Joy, joy_callback)
    rate = rospy.Rate(10)
    vel = Twist()
    while not rospy.is_shutdown():
        vel.linear.x = .5*vy
        vel.angular.z = 2*(vx if vy >=0 else -vx)
        vel_pub.publish(vel)
        rate.sleep()

if __name__ == "__main__":
    main()


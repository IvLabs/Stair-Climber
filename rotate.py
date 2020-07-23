#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist


def rotate():
    rospy.init_node('rotate', anonymous=True)
    velocity_publisher = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)

    vel_msg = Twist()

    r = rospy.Rate(10)

    vel_msg.linear.x = 0
    vel_msg.linear.z = 0
    vel_msg.linear.y = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = PI/2


    while not rospy.is_shutdown():
       velocity_publisher.publish(vel_msg)
       r.sleep()
if __name__ == '__main__':
    try:
       rotate()
    except rospy.ROSInterruptException: rospy.loginfo("move node terminated.")

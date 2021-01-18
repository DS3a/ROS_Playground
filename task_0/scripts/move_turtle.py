#!/usr/bin/env python3

import rospy
import time
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

move_node = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=10)
rospy.init_node('mover', anonymous=False)

pose = Pose()


def move_bot():
    speed = int(input('enter the speed you want it to move with : '))
    distance = int(input('enter the distance you want it to travel for :'))
    time_ = float(distance)/float(speed) # the amount of time the robot should travel for
    vel_msg = Twist()

    vel_msg.linear.x = speed
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0

    move_node.publish(vel_msg)
    time.sleep(time_)
    vel_msg.linear.x = 0
    move_node.publish(vel_msg)

if __name__ == '__main__':
    while not rospy.is_shutdown():
        try:
            move_bot()
        except rospy.ROSInterruptException:
            pass


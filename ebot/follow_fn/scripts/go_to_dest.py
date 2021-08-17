#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int16MultiArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
from tf.transformations import euler_from_quaternion
import time
import os

rospy.init_node("ebot_controller")

pos = [0, 0]

DESTINATION = [12, 0]
Theta = [0]
SCAN = [0, 0, 0, 0]
def got_pose(pose: Odometry):
    pos[0] = pose.pose.pose.position.x
    pos[1] = pose.pose.pose.position.y
    q = pose.pose.pose.orientation
    _x, _y, z = euler_from_quaternion([q.x, q.y, q.z, q.w])
    Theta[0] = z # radians

def got_scan(scan: Int16MultiArray):
    SCAN[0] = scan.data[0]
    SCAN[1] = scan.data[1]
    SCAN[2] = scan.data[2]
    SCAN[3] = scan.data[3]

vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
odom_listener = rospy.Subscriber('/odom', Odometry, got_pose)
scan_listener = rospy.Subscriber('/simplified_scan', Int16MultiArray, got_scan)

while not rospy.is_shutdown():
    destination_reached = False
    vel_msg = Twist()
    time_prev = time_init = time.time()
    while not destination_reached:
        time_curr = time.time() - time_init
        
        vel_msg = Twist()

        need_to_turn = False
        turn_counter = 0
        if SCAN[1]==1 or SCAN[2]==1:
            vel_msg.angular.z = 1.8
            need_to_turn = True
            turn_counter += 1
        elif SCAN[0]==1:
            vel_msg.angular.z = -0.4
            vel_msg.linear.x = 0.75
        elif SCAN[3]==1:
            vel_msg.angular.z = 0.4
            vel_msg.linear.x = 0.75
        elif SCAN[0] == SCAN[1] == SCAN[2] == SCAN[3] == 0:
            if need_to_turn and turn_counter<100000:
                vel_msg.angular.z = 1.2
                turn_counter += 1
                vel_publisher.publish(vel_msg)
                continue
            else:
                need_to_turn = False
                turn_counter = 0
            print('no obstacles, going to destination')
            delta = DESTINATION.copy()
            delta[0] -= pos[0]
            delta[1] -= pos[1]
            req_turn = np.arctan(delta[1]/delta[0])
            print(req_turn, Theta[0])
            print(pos, DESTINATION)
            print("\n\n\n")
            if abs(req_turn) <= 0.3 or need_to_turn:
                print('moving')
                vel_msg.angular.z = 0
                vel_msg.linear.x = 0.3
            else:
                print('turning')
                vel_msg.angular.z = 0.75 if req_turn > 0 else -0.75
                vel_msg.linear.x = 0.25

        vel_publisher.publish(vel_msg)
        time_prev = time.time()


        if abs(pos[0]-DESTINATION[0])<0.2 and abs(pos[1]-DESTINATION[1])<0.2:
            vel_msg = Twist()
            vel_publisher.publish(vel_msg)
            destination_reached = True
    
    if destination_reached:
        break 
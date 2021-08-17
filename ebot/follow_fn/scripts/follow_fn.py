#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Int16MultiArray
from sensor_msgs.msg import LaserScan
import numpy as np
import time
from tf.transformations import euler_from_quaternion

X = [0]
Theta = [0]


pos = [0, 0]

DESTINATION = [12, 0]
Theta = [0]
SCAN = [0, 0, 0, 0]

def got_pose(pose: Odometry):
    X[0] = pose.pose.pose.position.x
    pos[0] = pose.pose.pose.position.x
    pos[1] = pose.pose.pose.position.y
    q = pose.pose.pose.orientation
    _x, _y, z = euler_from_quaternion([q.x, q.y, q.z, q.w])
    Theta[0] = z
rospy.init_node("ebot_controller")
vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
odom_listener = rospy.Subscriber('/odom', Odometry, got_pose)


def y(x):
    return 2.0 * np.sin(x) * np.sin(x/2)

def dy_dx(x):
    return 2 * np.cos(x) * np.sin(x/2) + np.sin(x) * np.cos(x/2)


dt = 1e-5 # accuracy of the algorithm
vel = 0.3

X_init = 0.0
X_final = 2*np.pi
T = 0 # radians


def callback(scan: LaserScan):
    ranges = scan.ranges
    right = ranges[:180]
    front_right = ranges[180:360]
    front_left = ranges[360:540]
    left = ranges[540:]
    occupancy = list(map(lambda x: sum(x)/len(x), [left, front_left, front_right, right]))
    occupancy = list(map(lambda x: x<4, occupancy))
    print(occupancy)

    occupancy_array = Int16MultiArray()
    occupancy_array.data = list(map(int, occupancy))
    SCAN[0] = occupancy_array.data[0]
    SCAN[1] = occupancy_array.data[1]
    SCAN[2] = occupancy_array.data[2]
    SCAN[3] = occupancy_array.data[3]
    print(SCAN)

listener = rospy.Subscriber("/ebot/laser/scan", LaserScan, callback)


while not rospy.is_shutdown():
    destination_reached = False
    vel_msg = Twist()
    time_prev = time_init = time.time()
    while not destination_reached:
        time_curr = time.time() - time_init
        vel_msg.linear.x = vel
        der = dy_dx(X[0])
        req_T = np.arctan(der)
        print(req_T*90/np.pi, T*90/np.pi, (T - req_T)/(time_curr - time_prev))
        vel_msg.angular.z = (T - req_T)/(time_curr - time_prev) * 1.2e10
        vel_publisher.publish(vel_msg)
        T = Theta[0]
        time_prev = time.time()
        if X[0] >= X_final:
            vel_msg = Twist()
            vel_publisher.publish(vel_msg)
            destination_reached = True
    
    if destination_reached:
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
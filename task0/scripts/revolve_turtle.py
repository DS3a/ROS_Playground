#!/usr/bin/env python3

import rospy
import time
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import math

move_node = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=10)
rospy.init_node('mover', anonymous=False)
vel_msg = Twist()


ninety = math.pi / 2

def set_movement(lin, ang):
    vel_msg.linear.x = lin
    vel_msg.angular.z = ang
    move_node.publish(vel_msg)

def move_bot():
    speed = float(input('enter the speed you want it to move with : '))
    distance = float(input('enter the radius of the revolutions you want : '))
    no_revs = float(input('enter the number of revolutions you want : '))
    time_ = float(distance)/float(speed) # the amount of time the robot should travel for
    

    set_movement(speed, 0)
    time.sleep(time_)
    set_movement(0, 0)

# Now to turn 90 degrees
    set_movement(0, speed)
    time.sleep(ninety/speed)
    set_movement(0, 0)    

#    no_revs = 1

    time_1 = no_revs * 2 * math.pi * distance / speed
    ang_vel = 2 * math.pi / time_1 * no_revs
    print(f'itll move for {time_1} seconds')
 
    start = time.time()   
    t = False
    while not t:
        set_movement(speed, ang_vel)
#        time.sleep(time_1)
        if time.time() - start >= time_1:
            t = True
            set_movement(0, 0)



if __name__ == '__main__':
    while not rospy.is_shutdown():
        try:
            move_bot()
        except rospy.ROSInterruptException:
            pass


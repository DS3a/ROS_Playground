#!/usr/bin/env python

import serial
import rospy
import time
from geometry_msgs.msg import Twist
import json


imu_topic_publisher = rospy.Publisher('/imu', Twist, queue_size=10)
rospy.init_node('imu_read', anonymous=False)
SERIAL_PORT = '/USB/TTY0'
baud = 9600
imu_data = serial.Serial(SERIAL_PORT, baud, timeout=0.1)

def serial_read():
    imu_data_output = imu_data.readline()
    return json.loads(imu_data_output)

def publish_vals():
    data = json.loads(serial_read())
    imu_msg = Twist()
    smth = 'to be fileld'

    imu_msg.linear.x = smth
    imu_msg.linear.y = smth
    imu_msg.linear.z = smth

    imu_msg.angular.x = smth
    imu_msg.angular.y = smth
    imu_msg.angular.z = smth

    imu_topic_publisher.publish(imu_msg)

if __name__ == '__main__':
    while not rospy.is_shutdown():
        try:
            publish_vals()
        except rospy.ROSInterruptException:
            pass

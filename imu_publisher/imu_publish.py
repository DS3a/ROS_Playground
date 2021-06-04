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

def do_the_meth(data):
    return data

def publish_vals():
    data = json.loads(serial_read())
    imu_msg = Twist()

    data = do_the_meth(data)
    imu_msg.linear.x = data['linear']['x']
    imu_msg.linear.y = data['linear']['y']
    imu_msg.linear.z = data['linear']['z']

    imu_msg.angular.x = data['angular']['x']
    imu_msg.angular.y = data['angular']['y']
    imu_msg.angular.z = data['angular']['z']

    imu_topic_publisher.publish(imu_msg)

if __name__ == '__main__':
    while not rospy.is_shutdown():
        try:
            publish_vals()
        except rospy.ROSInterruptException:
            pass

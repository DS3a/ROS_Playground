#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import serial
import json

rospy.init_node("bot_teleop", anonymous=False)
SERIAL_PORT = "/dev/ttyACM0"

MAX_SPEED = 180
MAX_ANGULAR_SPEED = 255 - MAX_SPEED
ser = serial.Serial(SERIAL_PORT)

def callback(data: Twist):
    if data.linear.x >= 1:
        data.linear.x = 1
    if data.angular.z >= 2:
        data.angular.z = 2

    left_speed = data.linear.x * MAX_SPEED
    right_speed = data.linear.x * MAX_SPEED

    left_speed += data.angular.z * MAX_ANGULAR_SPEED
    right_speed -= data.angular.z * MAX_ANGULAR_SPEED

    speed = {
        "left_speed": int(left_speed),
        "right_speed": int(right_speed)
        }
    ser.write(json.dumps(speed).encode('ascii'))
    print(json.dumps(speed).encode("ascii"))

if __name__ == "__main__":
    rospy.Subscriber("/cmd_vel", Twist, callback)
    rospy.spin()
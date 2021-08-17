#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int16MultiArray
import numpy as np

rospy.init_node("scan_listener", anonymous=False)
scan_pub = rospy.Publisher("/simplified_scan", Int16MultiArray, queue_size=10)


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
    scan_pub.publish(occupancy_array)


if __name__ == "__main__":
    listener = rospy.Subscriber("/ebot/laser/scan", LaserScan, callback)
    rospy.spin()
#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import main

common_vars = main.CommonVars()


def ack(data):
    print('received data : ', data)


def listener():
    rospy.init_node(common_vars.subscriber_name, anonymous=True)
    rospy.Subscriber(common_vars.topic_name, String, ack)

    rospy.spin()


if __name__ == '__main__':
    listener()

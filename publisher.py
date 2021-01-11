#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import main

common_vars = main.CommonVars()


def talker(rate=common_vars.rate):
    publisher = rospy.Publisher(common_vars.topic_name, String, queue_size=10)
    rospy.init_node(common_vars.publisher_name, anonymous=True)
    rate_mod = rospy.Rate(rate)
    while not rospy.is_shutdown():
        msg = str(input('Enter the message to send to the topic : '))
        rospy.loginfo(msg)
        publisher.publish(msg)
        rate_mod.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

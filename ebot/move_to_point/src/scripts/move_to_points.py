#!/usr/bin/env python3

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


def movebase_client(points):

    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    print("waiting for server")
    client.wait_for_server()
    print("connection to actionlib server established")

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    for point in points:
        print(f'moving to point {point}')
        goal.target_pose.pose.position.x = point[0]
        goal.target_pose.pose.position.y = point[1]
        goal.target_pose.pose.orientation.w = point[1]

        client.send_goal(goal)
        wait = client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
    return client.get_result()

if __name__ == '__main__':
    try:
        print("initializing node")
        rospy.init_node('movebase_client_py')
        result = movebase_client([(10.7, 10.5), (12, -1.9), (-9.1, -1.2), (18.2, -1.4), (-2, 4)])
        if result:
            rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
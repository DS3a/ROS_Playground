#!/usr/bin/env python3

# Import ROS libraries and messages
import rospy
from sensor_msgs.msg import Image

import numpy as np
import open3d as o3d
import ransac
import ros_numpy

from sensor_msgs.msg import PointCloud2
import std_msgs.msg
from std_msgs.msg import Float32
import sensor_msgs.point_cloud2 as pc2
    
def callback(data):
    global distance
    global pub
    th = 10
    pc = ros_numpy.numpify(data)
    points=np.zeros((pc.shape[0],3))
    points[:,0]=pc['x']
    points[:,1]=pc['y']
    points[:,2]=pc['z']

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)

    dist = ransac.find_plane(pcd, th)
    pub.publish(dist)

def distance_node():
    rospy.init_node('distance_node')
    global pub
    pub = rospy.Publisher('distance', Float32, queue_size=1)
    rospy.Subscriber('/d435/depth/color/points', pc2.PointCloud2, callback, queue_size=None)
    rospy.spin()

if __name__ == '__main__':
    distance_node()
#!/usr/bin/env python
import rospy
import numpy as np
import ros_numpy
from sensor_msgs.msg import PointCloud2

def callback(data):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

    xyz_array = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(data)
    print(xyz_array)

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/kinect2/qhd/points", PointCloud2, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()

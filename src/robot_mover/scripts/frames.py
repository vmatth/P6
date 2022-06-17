#!/usr/bin/env python    
import rospy
from geometry_msgs.msg import Pose

import tf2_ros
import tf2_geometry_msgs  # **Do not use geometry_msgs. Use this instead for PoseStamped


def transform_pose(input_pose, input_pose2, from_frame, to_frame):

    # **Assuming /tf2 topic is being broadcasted
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    pose_stamped = tf2_geometry_msgs.PoseStamped()
    pose_stamped.pose = input_pose
    pose_stamped.header.frame_id = from_frame
    pose_stamped.header.stamp = rospy.Time.now()

    pose_stamped2 = tf2_geometry_msgs.PoseStamped()
    pose_stamped2.pose = input_pose2
    pose_stamped2.header.frame_id = to_frame
    pose_stamped2.header.stamp = rospy.Time.now()

    try:
        # ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
        output_pose_stamped = tf_buffer.transform(pose_stamped, to_frame, rospy.Duration(1))
        return output_pose_stamped.pose

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        raise


# Test Case
rospy.init_node("transform_test")

cam_pose = Pose()
cam_pose.position.x = -0.25
cam_pose.position.y = -0.50
cam_pose.position.z = +1.50
cam_pose.orientation.x = 0
cam_pose.orientation.y = 0
cam_pose.orientation.z = 0
cam_pose.orientation.w = 0

robot_pose = Pose()
robot_pose.position.x = 0
robot_pose.position.y = 0
robot_pose.position.z = 0
robot_pose.orientation.x = 0
robot_pose.orientation.y = 0
robot_pose.orientation.z = 0
robot_pose.orientation.w = 0

transformed_pose = transform_pose(robot_pose, robot_pose, "fixture", "world")

print(transformed_pose)
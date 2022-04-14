#! /usr/bin/env python

import rospy
#from __future__ import print_function

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.

import control_msgs.msg
import trajectory_msgs.msg

def UR5e_client():
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    client = actionlib.SimpleActionClient('scaled_pos_joint_traj_controller/follow_joint_trajectory', control_msgs.msg.FollowJointTrajectoryAction)

    # Waits until the action server has started up and started
    # listening for goals.
    print("Waiting for action server")
    client.wait_for_server()
    print("Action server found")

    # Creates a goal to send to the action server.
   # goal = control_msgs.msg.FollowJointTrajectoryActionGoal()
    goal = control_msgs.msg.FollowJointTrajectoryGoal()
    goal.trajectory.joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
    # goal.trajectory.points
    # goal.trajectory.points[0] = [0,0,0,0,0,0]
    # goal.trajectory.points[1] = [0,0,0,0,0,0]  
    # goal.trajectory.points[3] = rospy.Duration(5, 0) 

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()  # A FibonacciResult

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('UR5e_action_client')
        result = UR5e_client()
        #print("Result:", ', '.join([str(n) for n in result.sequence]))
    except rospy.ROSInterruptException:
        print("UR5e client interrupted before completion")

#!/usr/bin/env python
import rospy
import cv2 
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from matplotlib import pyplot as plt
import message_filters
from read_camera.msg import Parcel #Parcel msg


# class eye_hand_calibration:

#     def __init__(self):
#         self.bridge = CvBridge()
#         self.rgb_sub = message_filters.Subscriber("/kinect2/hd/image_color", Image)

import sys
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import math
from moveit_commander.conversions import pose_to_list
from geometry_msgs.msg import Pose  
from scipy.spatial.transform import Rotation

#class here
class mover:

    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('robot_mover', anonymous=True)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()

        self.group_name = "manipulator"
        self.group = moveit_commander.MoveGroupCommander(self.group_name)

        self.planning_frame = self.group.get_planning_frame()

        self.eef_link = self.group.get_end_effector_link()

        self.calibrate()

    def calibrate(self):
        print("hi jepp")
        print("Robot Pose: ", self.group.get_current_pose())

    def calibrate_eye_hand(self, R_gripper2base, t_gripper2base, R_target2cam, t_target2cam, eye_to_hand=True):

        if eye_to_hand:
            # change coordinates from gripper2base to base2gripper
            R_base2gripper, t_base2gripper = [], []
            for R, t in zip(R_gripper2base, t_gripper2base):
                R_b2g = R.T
                t_b2g = -R_b2g @ t
                R_base2gripper.append(R_b2g)
                t_base2gripper.append(t_b2g)
            
            # change parameters values
            R_gripper2base = R_base2gripper
            t_gripper2base = t_base2gripper

        # calibrate
        R, t = cv2.calibrateHandEye(
            R_gripper2base=R_gripper2base,
            t_gripper2base=t_gripper2base,
            R_target2cam=R_target2cam,
            t_target2cam=t_target2cam,
        )

        return R, t


def main():
    #lav en instance af klassen
    mi = mover()

    rospy.spin()    


if __name__ == '__main__':
    main()

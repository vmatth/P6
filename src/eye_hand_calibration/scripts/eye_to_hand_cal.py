#!/usr/bin/env python
import rospy
import cv2 
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from matplotlib import pyplot as plt
import message_filters
from read_camera.msg import Parcel #Parcel msg
from pynput import keyboard
import sys
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import math
from moveit_commander.conversions import pose_to_list
from geometry_msgs.msg import Pose  
from scipy.spatial.transform import Rotation
import glob
from numpy.linalg import inv


class hand_eye:

    def __init__(self):
        self.bridge = CvBridge()
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('robot_mover', anonymous=True)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()

        self.group_name = "manipulator"
        self.group = moveit_commander.MoveGroupCommander(self.group_name)

        self.planning_frame = self.group.get_planning_frame()

        self.eef_link = self.group.get_end_effector_link()

        self.rgb_sub = rospy.Subscriber("/kinect2/hd/image_color", Image, self.callback)

        self.pattern_size = (7,5)
        self.square_size = 0.03


    def quaternion_rotation_matrix(self, r_gripper2base_quat):
        """
        Covert a quaternion into a full three-dimensional rotation matrix.
    
        Input
        :param Q: A 4 element array representing the quaternion (q0,q1,q2,q3) 
    
        Output
        :return: A 3x3 element matrix representing the full 3D rotation matrix. 
                This rotation matrix converts a point in the local reference 
                frame to a point in the global reference frame.
        """
        # Extract the values from r_gripper2base_quat
        q0 = r_gripper2base_quat.x
        q1 = r_gripper2base_quat.y
        q2 = r_gripper2base_quat.z
        q3 = r_gripper2base_quat.w
        
        # First row of the rotation matrix
        r00 = 2 * (q0 * q0 + q1 * q1) - 1
        r01 = 2 * (q1 * q2 - q0 * q3)
        r02 = 2 * (q1 * q3 + q0 * q2)
        
        # Second row of the rotation matrix
        r10 = 2 * (q1 * q2 + q0 * q3)
        r11 = 2 * (q0 * q0 + q2 * q2) - 1
        r12 = 2 * (q2 * q3 - q0 * q1)
        
        # Third row of the rotation matrix
        r20 = 2 * (q1 * q3 - q0 * q2)
        r21 = 2 * (q2 * q3 + q0 * q1)
        r22 = 2 * (q0 * q0 + q3 * q3) - 1
        
        # 3x3 rotation matrix
        R_base2gripper = np.array([[r00, r01, r02],
                            [r10, r11, r12],
                            [r20, r21, r22]])
                            
        R_gripper2base = inv(R_base2gripper)
        return R_gripper2base

    def get_rotation(self):
        print("hi vini")
        r_gripper2base_quat = self.group.get_current_pose().pose.orientation
        print("R_Gripper2Base Quat", r_gripper2base_quat)
        # Returns R_gripper2base
        return self.quaternion_rotation_matrix(r_gripper2base_quat)
        

    def get_translation(self):
        print("hi vinini")
        t_base2gripper = self.group.get_current_pose().pose.position
        print("t_base2gripper", t_base2gripper)
        t_gripper2base = inv(t_base2gripper)
        print("t_gripper2base")
        return t_gripper2base


    def find_corners(self, image):
        found, corners = cv2.findChessboardCorners(image, self.pattern_size)
        term = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT, 30, 0.1)
        if found:
            cv2.cornerSubPix(image, corners, (5, 5), (-1, -1), term)
        return found, corners

    def draw_corners(self, image, corners):
        color_image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        cv2.drawChessboardCorners(color_image, self.pattern_size, corners, True)
        return color_image


    def get_object_pose(self, object_points, image_points, camera_matrix, dist_coeffs):
        ret, rvec, tvec = cv2.solvePnP(object_points, image_points, camera_matrix, dist_coeffs)
        return rvec.flatten(), tvec.flatten()

    def calibrate_lens(self, image_list):
        img_points, obj_points = [], []
        h,w = 0, 0
        for img in image_list:
            h, w = img.shape[:2]
            found,corners = self.find_corners(img)
            if not found:
                raise Exception("chessboard calibrate_lens Failed to find corners in img")
            img_points.append(corners.reshape(-1, 2))
            obj_points.append(self.pattern_points)
        camera_matrix = np.zeros((3,3))
        dist_coeffs = np.zeros(5)
    #    rms, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, (w,h))
        cv2.calibrateCamera(obj_points, img_points, (w,h), camera_matrix, dist_coeffs)
        return camera_matrix, dist_coeffs


def main():
    #lav en instance af klassen
    he = hand_eye()

    rospy.spin()    


if __name__ == '__main__':
    main()



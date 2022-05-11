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

        #self.rgb_sub = rospy.Subscriber("/kinect2/hd/image_color", Image, self.callback)

        r = self.get_rotation()
        t = self.get_translation()
        self.get_transformation_matrix(r, t)



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
        print("R_base2gripper ", R_base2gripper)                    
        #R_gripper2base = inv(R_base2gripper)
        return R_base2gripper

    def get_rotation(self):
        print("hi vini")
        r_gripper2base_quat = self.group.get_current_pose().pose.orientation
        # print("R_Gripper2Base Quat", r_gripper2base_quat)
        # Returns R_gripper2base
        return self.quaternion_rotation_matrix(r_gripper2base_quat)
        

    def get_translation(self):
        print("hi vinini")
        t_base2gripper = self.group.get_current_pose().pose.position
        #t_gripper2base = inv(t_base2grippers)
        #print("t_gripper2base")
        #return t_gripper2base
        t = np.array([[t_base2gripper.x, t_base2gripper.y, t_base2gripper.z]]).T
        print("translation ", t)
        return t


    def get_transformation_matrix(self, R_base2gripper, t_base2gripper):
        H_base2gripper = np.hstack((R_base2gripper, t_base2gripper))
        print("Transformation Matrix: ", H_base2gripper)

        file = open("pose_list.txt", "w+")
        #np.savetxt("file1.txt", H_base2gripper)
        content = str(H_base2gripper)
        file.write(content)
        file.close()





def main():
    #lav en instance af klassen
    he = hand_eye()

    rospy.spin()    


if __name__ == '__main__':
    main()



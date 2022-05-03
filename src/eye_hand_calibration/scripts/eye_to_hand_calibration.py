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

# class eye_hand_calibration:

#     def __init__(self):
#         self.bridge = CvBridge()
#         self.rgb_sub = message_filters.Subscriber("/kinect2/hd/image_color", Image)

#class here
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

        self.calibrate()

        self.pattern_size = (5,7)
        self.corners = None

    def calibrate(self):
        print("hi jepp")
        print("Robot Pose: ", self.group.get_current_pose())

        t_gripper2base = self.group.get_current_pose().pose.position

        print("T_Gripper2Base", t_gripper2base)

        r_gripper2base_quat = self.group.get_current_pose().pose.orientation

        print("R_Gripper2Base Quat", r_gripper2base_quat)

        #r_gripper2base_eul = self.euler_from_quaternion(r_gripper2base_quat.x, r_gripper2base_quat.y, r_gripper2base_quat.z, r_gripper2base_quat.w)
    
        #print("R_Gripper2Base Euler", r_gripper2base_eul)

        r_gripper2base = self.quaternion_rotation_matrix(r_gripper2base_quat)
        print("r gripper 2 base: ", r_gripper2base)

        image_data = rospy.wait_for_message("/kinect2/hd/image_color", Image, timeout=None)
        self.find_checkerboard_corners(image_data)

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
        rot_matrix = np.array([[r00, r01, r02],
                            [r10, r11, r12],
                            [r20, r21, r22]])
                                
        return rot_matrix



    def euler_from_quaternion(self, x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
    
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
    
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
    
        return roll_x, pitch_y, yaw_z # in radians


    def find_checkerboard_corners(self, data):
        try:
            image = self.bridge.imgmsg_to_cv2(data, "passthrough")
            cv2.imshow("ja2", image)
            cv2.waitKey(0)
            ret, corners = cv2.findChessboardCorners(image, (5,7), None)
            print("Corners ", corners)
            cv2.drawChessboardCorners(image, (5,7), corners, ret)
            cv2.imshow("ja2", image)
            cv2.waitKey(0)
        except CvBridgeError as e:
            print(e)


    # def calibrate_eye_hand(self, R_gripper2base, t_gripper2base, R_target2cam, t_target2cam, eye_to_hand=True):

    #     if eye_to_hand:
    #         # change coordinates from gripper2base to base2gripper
    #         R_base2gripper, t_base2gripper = [], []
    #         for R, t in zip(R_gripper2base, t_gripper2base):
    #             R_b2g = R.T
    #             t_b2g = -R_b2g @ t
    #             R_base2gripper.append(R_b2g)
    #             t_base2gripper.append(t_b2g)
            
    #         # change parameters values
    #         R_gripper2base = R_base2gripper
    #         t_gripper2base = t_base2gripper

    #     # calibrate
    #     R, t = cv2.calibrateHandEye(
    #         R_gripper2base=R_gripper2base,
    #         t_gripper2base=t_gripper2base,
    #         R_target2cam=R_target2cam,
    #         t_target2cam=t_target2cam,
    #     )

    #     return R, t

def main():
    #lav en instance af klassen
    he = hand_eye()

    #rospy.spin()    


if __name__ == '__main__':
    main()

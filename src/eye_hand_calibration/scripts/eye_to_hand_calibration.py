#!/usr/bin/env python
from re import S
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
from numpy import median


# class eye_hand_calibration:

#     def __init__(self):
#         self.bridge = CvBridge()
#         self.rgb_sub = message_filters.Subscriber("/kinect2/hd/image_color", Image)

#class here
class hand_eye:

    def __init__(self):
        self.bridge = CvBridge()
        # moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('robot_mover', anonymous=True)

        self.rgb_sub = rospy.Subscriber("/kinect2/hd/image_color", Image, self.callback)

        self.counter = 0
        self.distances = []

        # self.robot = moveit_commander.RobotCommander()
        # self.scene = moveit_commander.PlanningSceneInterface()

        # self.group_name = "manipulator"
        # self.group = moveit_commander.MoveGroupCommander(self.group_name)

        # self.planning_frame = self.group.get_planning_frame()

        # self.eef_link = self.group.get_end_effector_link()

        # self.nr = 0

        # self.pattern_size = (5,7)
        # self.corners = None

        # #self.calibrate_pictures()

        #self.calculate_XYZ(0, 0)

            # while True:
            #     self.take_picture()
            #self.calibrate()


    def calibrate(self):
        print("hi jepp")
        # print("Robot Pose: ", self.group.get_current_pose())

        # t_gripper2base = self.group.get_current_pose().pose.position

        # print("T_Gripper2Base", t_gripper2base)

        # r_gripper2base_quat = self.group.get_current_pose().pose.orientation

        # print("R_Gripper2Base Quat", r_gripper2base_quat)

        # #r_gripper2base_eul = self.euler_from_quaternion(r_gripper2base_quat.x, r_gripper2base_quat.y, r_gripper2base_quat.z, r_gripper2base_quat.w)
    
        # #print("R_Gripper2Base Euler", r_gripper2base_eul)

        # r_gripper2base = self.quaternion_rotation_matrix(r_gripper2base_quat)
        # print("r gripper 2 base: ", r_gripper2base)

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


    def calculate_XYZ(self, u, v, z):
        s = 1
        A = np.matrix([[1.0663355230063235*10**3, 0., 9.4913144897241432*10**2], [0, 1.0676521964588569*10**3, 5.3505238717783232*10**2], [0., 0., 1.]])
        R = np.matrix([[9.9988107827826278e-01, -5.9309422117523802e-04,-1.5410306302711205e-02],[6.1924030152182927e-04, 9.9999837692924043e-01, 1.6919457242115465e-03], [1.5409277807462079e-02, -1.7012871978343688e-03, 9.9987982266836595e-01]])
        t = np.array([[-4.0874634519709227e-02, 1.3982841913969224e-04, 2.7999300285299357e-03]])

        print("u,v,z", u, v, z)
        print("A: ", A)
        print("R: ", R)
        print("t: ", t)

        uv1 = np.array([[u,v,z]])

        #Transpose uv1
        uv1 = uv1.T
        print("uv1 Transpose: ", uv1)

        #Times by scaling factor
        s_uv1 = s*uv1
        print("s*uv1", s_uv1)

        #Invert A
        A_inv = inv(A)
        print("A^-1: ", A_inv)

        #A^-1 * s_uv1
        xyz_c = A_inv.dot(s_uv1)
        print("A^-1 * s_uv1: ", xyz_c)

        #Transpose t
        t = t.T
        print("T Transpose: ", t)

        #Substract t
        xyz_c = xyz_c - t
        print("Subtracted by T: ", xyz_c)

        #Invert R
        R_inv = inv(R)
        print("R^-1: ", R_inv)

        XYZ = R_inv.dot(xyz_c)
        print("Final XYZ", XYZ)



    def callback(self, image_data):
        print("take pic!")
        print("bruh")
        try:
            image = self.bridge.imgmsg_to_cv2(image_data, "passthrough")

            #cx = 890
            #cy = 535

            x = 944
            y = 541
            #Draw circle at xy
            distance = image[y][x]
            cv2.circle(image, (x,y), 3, (255, 0, 0), -1)


            # self.distances.append(distance)
            # self.counter = self.counter + 1
            # if(self.counter > 1000):
            #     print("Calculating median ")
            #     median = np.median(self.distances)
            #     print("Median: ", median)
            #     rospy.sleep(1000)


                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    
            #Get distance (z) to xy value 
            z = 1

            self.calculate_XYZ(x,y,z)
            print("DISTANCE", distance)
            cv2.waitKey(3)
            cv2.imshow("ja2", image)
            #cv2.waitKey(0)

        except CvBridgeError as e:
            print(e)


    def calibrate_pictures(self):

        w = 5
        h = 7
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)


        objp = np.zeros((w * h, 3), np.float32)
        objp[:, :2] = np.mgrid[0:w, 0:h].T.reshape(-1, 2)
        # Arrays to store object points and image points from all the images.
        objpoints = [] # 3d point in real world space
        imgpoints = [] # 2d points in image plane.

      
        print("Calibrating pictures :]")
        images = glob.glob('*')
        for fname in images:
            img = cv2.imread(fname)
            try:
                gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            except:
                gray = img

            ret, corners = cv2.findChessboardCorners(gray, (w,h), None)
            if ret == True:
                objpoints.append(objp)
                corners2 = cv2.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
                imgpoints.append(corners)
                # Draw and display the corners
                cv2.drawChessboardCorners(gray, (w,h), corners2, ret)
                #cv2.imshow("ja2", gray)
                #cv2.waitKey(0)

                chess_board_len = 3 #mm??

                object_points=np.zeros((3,w*h),dtype=np.float64)
                flag=0
                for i in range(h):
                    for j in range(w):
                        object_points[:2,flag]=np.array([(11-j-1)*chess_board_len,(8-i-1)*chess_board_len])
                        flag+=1

                ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

                corner_points=np.zeros((2,corners.shape[0]),dtype=np.float64)
                for i in range(corners.shape[0]):
                    corner_points[:,i]=corners[i,0,:]

                #make into np array
                K = np.matrix([[1.0663355230063235*10**3, 0., 9.4913144897241432*10**2], [0, 1.0676521964588569*10**3, 5.3505238717783232*10**2], [0., 0., 1.]])
                print("Matrix: ", K)

                # K = [ [1.0663355230063235e+03, 0., 9.4913144897241432e+020],
                #     [0, 1.0676521964588569e+03, 5.3505238717783232e+02], 
                #     [0., 0., 1.]]

                rvec, tvec = cv2.solvePnP(object_points, corner_points, K, distCoeffs = dist)



    def find_checkerboard_corners(self, data):
        try:
            #Define values for the checkerboard corners
            w = 5
            h = 7
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

            # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
            objp = np.zeros((w*h,3), np.float32)
            objp[:,:2] = np.mgrid[0:w,0:h].T.reshape(-1,2)

            # Arrays to store object points and image points from all the images.
            objpoints = [] # 3d point in real world space
            imgpoints = [] # 2d points in image plane.

            image = self.bridge.imgmsg_to_cv2(data, "passthrough")
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

            file_name = "image" + str(self.nr) + ".jpg"
            cv2.imwrite(file_name, image)
            print("saving image!!!")
            self.nr = self.nr + 1

            ret, corners = cv2.findChessboardCorners(gray, (w,h), None)

            if ret == True:
                objpoints.append(objp)
                corners2 = cv2.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
                imgpoints.append(corners)
                # Draw and display the corners
                cv2.drawChessboardCorners(gray, (w,h), corners2, ret)
                cv2.imshow("ja2", gray)
                cv2.waitKey(0)

                ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)



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

    rospy.spin()    


if __name__ == '__main__':
    main()

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

        self.rgb_sub = rospy.Subscriber("/kinect2/hd/image_depth_rect", Image, self.callback)

        self.counter = 0
        self.distances = []

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

            x = 946
            y = 537
            #Draw circle at xy
            distance = image[y][x]
            cv2.circle(image, (x,y), 3, (255, 0, 0), -1)


            self.distances.append(distance)
            self.counter = self.counter + 1
            if(self.counter > 1000):
                print("Calculating median ")
                median = np.median(self.distances)
                print("Median: ", median)
                rospy.sleep(1000)


                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    
            #Get distance (z) to xy value 
            z = 1

            #self.calculate_XYZ(x,y,z)
            #print("DISTANCE", distance)
            cv2.waitKey(3)
            cv2.imshow("ja2", image)
            #cv2.waitKey(0)

        except CvBridgeError as e:
            print(e)


def main():
    #lav en instance af klassen
    he = hand_eye()

    rospy.spin()    


if __name__ == '__main__':
    main()

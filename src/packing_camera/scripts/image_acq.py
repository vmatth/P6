#!/usr/bin/env python
from xml.dom import HierarchyRequestErr
import cv2 
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import rospy
from matplotlib import pyplot as plt
import message_filters
from read_camera.msg import Parcel

class image_converter:

    def __init__(self):
        self.bridge = CvBridge()
        self.rgb_sub = message_filters.Subscriber("/kinect2/hd/image_color", Image)
        self.depth_sub = message_filters.Subscriber("/kinect2/hd/image_depth_rect", Image)
        self.ts = message_filters.TimeSynchronizer([self.rgb_sub, self.depth_sub], 10)
        self.ts.registerCallback(self.camera_callback)

        self.pub = rospy.Publisher('parcel_info', Parcel, queue_size=10)

        ###########################################################################
        ################################ SETUP ##################################
        ##########################################################################
        
        #Udregn hver gang setup aendres, ved at dividere bredden eller hoejden i pixels, over bredden eller hoejden af et kendt objekt i cm.
        self.pix_per_cm = 10.2

        #Skal kalibreres hver gang setup aendres.
        self.cam_height = 111.5

    def camera_callback(self, rgb_data, depth_data):
        rospy.loginfo("Receiving info from image topic!")
        try:
            #Get rgb & depth images for this frame
            rgb_image = self.bridge.imgmsg_to_cv2(rgb_data)
            depth_image = self.bridge.imgmsg_to_cv2(depth_data, "16UC1")

            #Crop image to only have the parcel in focus - removes unnecesarry items
            rgb_image = rgb_image[400:950, 600:1100]
            depth_image = depth_image[400:950, 600:1100]            

            #Grayscale image
            gray = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2GRAY)
            
            #Blur
            GaussBlur = cv2.GaussianBlur(gray,(5,5), cv2.BORDER_DEFAULT)

            #Threshold
            Th_val, Th_image = cv2.threshold(GaussBlur, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
            #print("Threshold value", Th_val)

            #Find contour
            im2, contours, hierarchy = cv2.findContours(Th_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
            print(contours)
            #Draw contour
            Cont = rgb_image
            cv2.drawContours(Cont, contours, -1, (0,255,0), 3)
	                

	        # see the results
            cv2.waitKey(33) # 30 FPS
            cv2.imshow("Gray", gray)
            cv2.imshow("Blur", GaussBlur)
            cv2.imshow("Threshold", Th_image)
            #cv2.imshow("Threshold values", Th_image2)
            cv2.imshow("Contours", Cont)

        except CvBridgeError as e:
            print(e)

def main():
    ic = image_converter()
    rospy.init_node('image_converter', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
#!/usr/bin/env python
import imp
from turtle import distance
from cv2 import threshold
import numpy as np
import imageio
from skimage import morphology
from skimage.segmentation import watershed
from skimage.segmentation import random_walker
from skimage.feature import peak_local_max
from scipy import ndimage
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

        #self.counter 

    def camera_callback(self, rgb_data, depth_data):
        rospy.loginfo("Receiving info from image topic!")
        try:
            #Get rgb & depth images for this frame
            rgb_image = self.bridge.imgmsg_to_cv2(rgb_data)

            
            #depth_image = self.bridge.imgmsg_to_cv2(depth_data, "16UC1")

            #Crop image to only have the parcel in focus - removes unnecesarry items
            rgb_image = rgb_image[450:752, 636:1102]
            #depth_image = depth_image[257:752, 626:1172]
            rgb_copy = rgb_image.copy()            

            #Grayscale image
            grayscale_image = cv2.cvtColor(rgb_copy, cv2.COLOR_BGR2GRAY)

            #Blur
            blur_image = cv2.medianBlur(grayscale_image, 5)
            #Threshold
            ret, thresh_image = cv2.threshold(blur_image,0,255,cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU)

            #computing the distance transfrom
            distance_t = ndimage.distance_transform_edt(thresh_image)

            cv2.waitKey(33)
            #Show images
            cv2.imshow("Distance", distance_t)
            cv2.imshow("Raw Image", rgb_image)
        except CvBridgeError as e:
            print(e)

    

    
    def parcel_pub(self, width, height, depth, angle, centerpoint_x, centerpoint_y):
        print("----------")
        print("Publishing parcel to /parcel_info")
        print("Parcel depth [cm]", depth)
        print("Parcel height [cm]", height)
        print("Parcel width [cm]", width)
        print("Parcel angle ", angle)
        msg = Parcel()
        msg.width = width
        msg.height = height
        msg.depth = depth
        msg.angle = angle
        msg.centerpoint_x = centerpoint_x
        msg.centerpoint_y = centerpoint_y
        
        self.pub.publish(msg)
        



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
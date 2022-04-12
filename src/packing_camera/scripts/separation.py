#!/usr/bin/env python
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
        #self.depth_sub = message_filters.Subscriber("/kinect2/hd/image_depth_rect", Image)
        self.ts = message_filters.TimeSynchronizer([self.rgb_sub, self.depth_sub], 10)
        self.ts.registerCallback(self.camera_callback)

        self.pub = rospy.Publisher('parcel_info', Parcel, queue_size=10)

        ###########################################################################
        ################################ SETUP ##################################
        ##########################################################################
        

    def camera_callback(self, rgb_data):
        rospy.loginfo("Receiving info from image topic!")
        try:
            #Get rgb & depth images for this frame
            rgb_image = self.bridge.imgmsg_to_cv2(rgb_data)

            #Crop image to only have the parcel in focus - removes unnecesarry items
            rgb_image = rgb_image[257:752, 626:1172]           

            #Grayscale image
            grayscale_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2GRAY)

            #Blur
            blur_image = cv2.medianBlur(grayscale_image, 5)

            #Threshold
            ret, thresh_image = cv2.threshold(blur_image,0,255,cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU)
            
            #Morphology
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(5,5)) #Can also be MORPH_ELLIPSE or MORPH_CROSS
            opening_image = cv2.morphologyEx(thresh_image, cv2.MORPH_OPEN, kernel)
            
            #Canny edge detection
            edge_image = cv2.Canny(grayscale_image,100,200)
            
            # Overlay of images with edges
            rgb_overlay = rgb_image

            #Find contours
            _, contours, _= cv2.findContours(opening_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) #Image for contours can be opening_image or edge_image
            
            #Bounding box with rotated rect
            i = 0
            for cnt in contours:
                #Create a rotated box around the parcel https://theailearner.com/tag/cv2-minarearect/
                rect = cv2.minAreaRect(cnt)
                box = cv2.boxPoints(rect)
                box = np.int0(box)

                #Overlay contours
                cv2.drawContours(rgb_overlay,[box],0,(0,0,255),2)

                i = i + 1


            cv2.waitKey(3)
            #Show images
            #cv2.imshow("Raw Image", rgb_image)
            #cv2.imshow("Grayscale Image", grayscale_image)
            #cv2.imshow("Blurred Image", blur_image)
            #cv2.imshow("Threshold", thresh_image)
            #cv2.imshow("Canny Edge Detection", edge_image)
            cv2.imshow("Morphology Image", opening_image)
            cv2.imshow("RGB Image w/ Overlay", rgb_overlay)
            #Save image for testing purposes
            # cv2.imwrite('test.jpg', blur_image)
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
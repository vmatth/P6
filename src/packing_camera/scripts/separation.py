#!/usr/bin/env python
from xml.dom import HierarchyRequestErr
import cv2
from cv2 import blur
from cv2 import contourArea
from cv2 import erode
from cv2 import Canny
from matplotlib.contour import ContourSet
import numpy as np
from matplotlib import image, pyplot as plt
import sys
import math
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import rospy
#from matplotlib import pyplot as plt
import message_filters
from read_camera.msg import Parcel



# #import image
# rgb = cv2.imread('test2.jpg', cv2.IMREAD_UNCHANGED)
 
# #################### resizing #################################
# scale_percent = 30                                            #
# width = int(rgb.shape[1] * scale_percent / 100)               #
# height = int(rgb.shape[0] * scale_percent / 100)              #
# dim = (width, height)                                         #
# resized = cv2.resize(rgb, dim, interpolation = cv2.INTER_AREA)#
#  ##############################################################

#image processing

#Grayscale
#gray = cv2.cvtColor(resized,cv2.COLOR_BGR2GRAY)

#Gauss
#blurred = cv2.GaussianBlur(gray,(7,7),0)
#_,thresh = cv2.threshold(blurred,0,255,cv2.THRESH_BINARY + cv2.THRESH_OTSU)


#Edge detection
#canny = cv2.Canny(blurred,10,150,0)

#Dilation
#eroded = cv2.erode(thresh, (1,1), iterations = 1)
#dilated = cv2.dilate(canny, (1,1), iterations = 1)

 
#Finding contours
#cnts = cv2.findContours(dilated.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#(cnts, hierarchy) = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
#cnts = cnts[0] if len(cnts) == 2 else cnts[1]


#Print and draw found contour areas with greater area than...
#min_area = 1000

# for cnt in cnts:
#     area = cv2.contourArea(cnt)
#     if area > min_area:
#         print("Contour areas",area)
#         cv2.drawContours(resized,[cnt],0,(0,255,0),2)
#         #print('Parcles in image: ', len(cnts))


#Drawing Contours
#cv2.drawContours(resized,cnts, -1, (0,255,0),2)


#cv2.imshow("Resized image", resized)
#cv2.imshow("grayscale", gray)
#cv2.imshow("Gauss", blurred)
#cv2.imshow("Edge", canny)
#cv2.imshow("Dilated", dilated)
#cv2.imshow("Eroded", eroded)
#cv2.imshow("Canny", canny)
#cv2.imshow("Contours", resized)
#cv2.waitKey(0)
#cv2.destroyAllWindows()    


#Finding contours
#cnts = cv2.findContours(cdstP, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#(cnts, hierarchy) = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
#cnts = cnts[0] if len(cnts) == 2 else cnts[1]


#Print and draw found contour areas with greater area than...
# min_area = 10000

# for cnt in cnts:
#     area = cv2.contourArea(cnt)
#     if area > min_area:
#         print("Contour areas",area)
#         cv2.drawContours(resized,[cnt],0,(0,255,0),1)
#         epsilon = 0.1*cv2.arcLength(cnt,True)
#         approxrect = cv2.approxPolyDP(cnt,epsilon,True)
#         approxrect = cv2.minAreaRect(cnt)
#         box = cv2.boxPoints(approxrect)
#         box = np.int0(box)
#         cv2.drawContours(resized,[box],0,(0,0,255),2)
#         #print('Parcles in image: ', len(cnts))




#Drawing Contours
#cv2.drawContours(resized,cnts, -1, (0,255,0),1)
    
#cv2.imshow("Source", resized)
#cv2.imshow("Detected Lines (in red) - Standard Hough Line Transform", cdst)
# cv2.imshow("Detected Lines (in red) - Probabilistic Line Transform", dilated)
# cv2.imshow("Contours", resized)
    
# cv2.waitKey()


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
            rgb_image = rgb_image[200:800, 500:1300]
            depth_image = depth_image[257:752, 626:1172]            

            #Grayscale image
            grayscale_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2GRAY)

            #Blur
            blur_image = cv2.medianBlur(grayscale_image,7,7)

            

            #Threshold
            ret, thresh_image = cv2.threshold(blur_image,0,255,cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU)
            #Morphology
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(1,1)) #Can also be MORPH_ELLIPSE or MORPH_CROSS
            opening_image = cv2.morphologyEx(thresh_image, cv2.MORPH_OPEN, kernel)

            #Edge detection
            canny = cv2.Canny(thresh_image, 10,150,0)
            
            
            # Overlay of images with edges
            rgb_overlay = rgb_image
            depth_overlay = depth_image

            #Find contours
            _,cnts,_= cv2.findContours(canny, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE) #Image for contours can be opening_image or edge_image
            
            #Print and draw found contour areas with greater area than...
            min_area = 1300
            # boxes = []
            # for c in cnts:
            #     x,y,w,h = cv2.boundingRect(c)
            #     boxes.append([x,y,w,h])
            #         #print("Contour areas",area)
            #         #approxrect = cv2.minAreaRect(box)
            #         #box = cv2.boxPoints(approxrect)
            #         #box = np.int0(box)
                
            # for box in boxes:
            #     top_left = (box[0],box[1])
            #     bottom_right = (box[0] + box[2], box[1] + box[3])
            #     cv2.rectangle(rgb_image,top_left, bottom_right, (0,255,0), 2)
            #     #cv2.drawContours(rgb_image,[c],-1,(0,255,0),2)

            min_area = 10000

            for cnt in cnts:
                area = cv2.contourArea(cnt)
                if area > min_area:
                    print("Contour areas",area)
                    #cv2.drawContours(resized,[cnt],0,(0,255,0),1)
                    epsilon = 0.1*cv2.arcLength(cnt,True)
                    approxrect = cv2.approxPolyDP(cnt,epsilon,True)
                    approxrect = cv2.minAreaRect(cnt)
                    box = cv2.boxPoints(approxrect)
                    box = np.int0(box)
                    cv2.drawContours(rgb_image,[box],0,(0,0,255),2)
            
            cv2.waitKey(10) # For 30 fps
            #Show images
            #cv2.imshow("Raw Image", rgb_image)
            #cv2.imshow("Grayscale Image", grayscale_image)
            cv2.imshow("Blurred Image", blur_image)
            #cv2.imshow("Threshold", thresh_image)
            #cv2.imshow("Canny Edge Detection", edge_image)
            cv2.imshow("Morphology Image", canny)
            cv2.imshow("RGB Image w/ Overlay", rgb_image)
            #cv2.imshow("Depth Image w/ Overlay", depth_overlay * 16)
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

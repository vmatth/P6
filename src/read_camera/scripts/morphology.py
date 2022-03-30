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
            rgb_image = rgb_image[257:752, 626:1172]
            depth_image = depth_image[257:752, 626:1172]            

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
            depth_overlay = depth_image

            #Find contours
            _, contours, _= cv2.findContours(opening_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) #Image for contours can be opening_image or edge_image
            
            #Bounding box with rotated rect
            i = 0
            for cnt in contours:
                print("------------------")
                print("Parcel [",i,"] found")
                #Create a rotated box around the parcel https://theailearner.com/tag/cv2-minarearect/
                rect = cv2.minAreaRect(cnt)
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                #Find centerpoint using quik maffs
                centerpoint_x = ((box[3][0] - box[1][0])/2) + box[1][0]
                centerpoint_y = ((box[0][1] - box[2][1])/2) + box[2][1]
                centerpoint = (centerpoint_x, centerpoint_y)
                center = rect[0]
                #Get pixel value at centerpoint
                if(centerpoint_x >= depth_image.shape[0] or centerpoint_y >= depth_image.shape[1]): #Check if all 4 corners are available, or else it returns error
                    print("Centerpoint cannot be found in depth image")
                else:
                    #Calculate dimensions & angle
                    distance_to_parcel = (depth_image[centerpoint_x][centerpoint_y]-150) / 10 #Calculate distance in [cm] 
                    width = (rect[1][1])/self.pix_per_cm
                    height = (rect[1][0])/self.pix_per_cm
                    depth = self.cam_height - distance_to_parcel
                    angle = rect[2]
                    #Call parcel_pub function
                    self.parcel_pub((rect[1][1])/self.pix_per_cm, (rect[1][0])/self.pix_per_cm, self.cam_height - distance_to_parcel, angle, centerpoint_x, centerpoint_y)
                    #Overlay centerpoint and contours to rgb and depth images
                    cv2.drawContours(rgb_overlay,[box],0,(0,0,255),2)
                    rgb_overlay = cv2.circle(rgb_overlay, centerpoint, radius=3, color=(0, 255, 0), thickness=-1)
                    cv2.drawContours(depth_overlay,[box],0,(255,255,255),2)
                    depth_overlay = cv2.circle(depth_overlay, centerpoint, radius=3, color=(255, 255, 255), thickness=-1)

                i = i + 1


            cv2.waitKey(3)
            #Show images
            cv2.imshow("Raw Image", rgb_image)
            cv2.imshow("Grayscale Image", grayscale_image)
            cv2.imshow("Blurred Image", blur_image)
            cv2.imshow("Threshold", thresh_image)
            #cv2.imshow("Canny Edge Detection", edge_image)
            cv2.imshow("Morphology Image", opening_image)
            cv2.imshow("RGB Image w/ Overlay", rgb_overlay)
            cv2.imshow("Depth Image w/ Overlay", depth_overlay * 16)
            #Save image for testing purposes
            # cv2.imwrite('test.jpg', blur_image)
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

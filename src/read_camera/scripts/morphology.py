#!/usr/bin/env python
import cv2 
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import rospy
from matplotlib import pyplot as plt
import message_filters
from read_camera.msg import Parcel #Parcel msg

class image_converter:

    def __init__(self): 
        self.bridge = CvBridge()
        self.rgb_sub = message_filters.Subscriber("/kinect2/hd/image_color", Image)
        self.depth_sub = message_filters.Subscriber("/kinect2/hd/image_depth_rect", Image)
        self.ts = message_filters.TimeSynchronizer([self.rgb_sub, self.depth_sub], 10)
        self.ts.registerCallback(self.camera_callback)

        self.pub = rospy.Publisher('/vision/parcel', Parcel, queue_size=10)

        ###########################################################################
        ################################ SETUP ##################################
        ##########################################################################
        
        #Udregn hver gang setup aendres, ved at dividere bredden eller hoejden i pixels, over bredden eller hoejden af et kendt objekt i cm.
        self.pix_per_cm = 12.35 #Dynamisk

        #Skal kalibreres hver gang setup aendres.
        self.cam_height = 102.5


    def camera_callback(self, rgb_data, depth_data):
        rospy.loginfo("Receiving info from image topic!")
        try:
            #Get rgb & depth images for this frame
            rgb_image = self.bridge.imgmsg_to_cv2(rgb_data)
            depth_image = self.bridge.imgmsg_to_cv2(depth_data, "16UC1")

            #Crop image to only have the parcel in focus - removes unnecesarry items
            rgb_image = rgb_image[242:785, 638:1050] 
            depth_image = depth_image[263:785, 644:1058]            

            #Grayscale image
            grayscale_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2GRAY)

            #Blur
            #blur_image = cv2.medianBlur(grayscale_image, 5)
            blur_image = cv2.bilateralFilter(grayscale_image, 15, 45, 45)

            #Threshold
            #ret, thresh_image = cv2.threshold(blur_image,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
            thresh_image = cv2.adaptiveThreshold(blur_image, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 11, 2)
            #thresh_image = cv2.adaptiveThreshold(blur_image, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)
            
            #Morphology
            kernel_erosion = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(5,5)) #Can also be MORPH_ELLIPSE or MORPH_CROSS
            kernel_dilation = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(7,7)) #Can also be MORPH_ELLIPSE or MORPH_CROSS
            #opening_image = cv2.morphologyEx(thresh_image, cv2.MORPH_OPEN, kernel)
            #closing_image = cv2.morphologyEx(opening_image, cv2.MORPH_CLOSE, kernel)
            img_erosion = cv2.erode(thresh_image, kernel_erosion, iterations=1)
            img_dilation = cv2.dilate(img_erosion, kernel_dilation, iterations=1)
            
            #Canny edge detection
            edge_image = cv2.Canny(blur_image,100,200)
            
            # Overlay of images with edges
            rgb_overlay = rgb_image
            depth_overlay = depth_image

            #Find contours
            _, contours, _= cv2.findContours(img_dilation, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) #Image for contours can be opening_image or edge_image
            #_, contours_closing, _= cv2.findContours(closing_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) #Image for contours can be opening_image or edge_image
            #Bounding box with rotated rect
            i = 0

            cv2.circle(rgb_image, (200,200), 3, (10000, 10000, 10000), -1)


            for cnt in contours:
                print("------------------")
                print("Parcel [",i,"] found")

                area = cv2.contourArea(contours[i])
                #print("area: ", area)
                if area > 3000 and area < 130000:
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
                        distance_to_parcel = (depth_image[centerpoint_x][centerpoint_y]) / 10 #Calculate distance in [cm] 
                        #print("testtstst: ", depth_image[centerpoint_x][centerpoint_y])
                        print("distance to parcel: ", distance_to_parcel)
                        #print("width in pixels", rect[1][1])
                        #print("length in pixels", rect[1][0])
                        width = (rect[1][1])/self.pix_per_cm
                        length = (rect[1][0])/self.pix_per_cm
                        heigth = self.cam_height - distance_to_parcel
                        angle = rect[2]
                        #Call parcel_pub function
                        self.parcel_pub((rect[1][1])/self.pix_per_cm, (rect[1][0])/self.pix_per_cm, self.cam_height - distance_to_parcel, angle, centerpoint_x, centerpoint_y, distance_to_parcel)
                        #Overlay centerpoint and contours to rgb and depth images
                        cv2.drawContours(rgb_overlay,[box],0,(0,0,255),2)
                        rgb_overlay = cv2.circle(rgb_overlay, centerpoint, radius=3, color=(0, 255, 0), thickness=-1)
                        cv2.drawContours(depth_overlay,[box],0,(255,255,255),2)
                        depth_overlay = cv2.circle(depth_overlay, centerpoint, radius=3, color=(255, 255, 255), thickness=-1)

                i = i + 1


            cv2.waitKey(3)
            #Show images
            cv2.imshow("Raw Image", rgb_image)
            #cv2.imshow("Grayscale Image", grayscale_image)
            #cv2.imshow("Blurred Image", blur_image)
            cv2.imshow("Threshold", thresh_image)
            #cv2.imshow("Threshold2", thresh_image2)
            #cv2.imshow("Canny Edge Detection", edge_image)
            cv2.imshow("Morphology Image", img_dilation)
            cv2.imshow("RGB Image w/ Overlay", rgb_overlay)
            cv2.imshow("Depth Image w/ Overlay", depth_overlay * 16)
            #Save image for testing purposes
            # cv2.imwrite('test.jpg', blur_image)
        except CvBridgeError as e:
            print(e)

    

    
    def parcel_pub(self, width, length, height, angle, centerpoint_x, centerpoint_y, distance_to_parcel):
        print("----------")
        print("Publishing parcel to /vision/parcel_raw")
        print("Parcel width [cm]", width)
        print("Parcel length [cm]", length)
        print("Parcel height [cm]", height)
        print("Parcel angle ", angle)
        msg = Parcel()
        msg.size.x = width
        msg.size.y = length
        msg.size.z = height
        msg.angle = angle
        
        #msg.centerpoint_x = centerpoint_x
        #msg.centerpoint_y = centerpoint_y
        msg.centerpoint.z = distance_to_parcel
        msg.centerpoint.x = centerpoint_x / self.pix_per_cm
        msg.centerpoint.y = centerpoint_y / self.pix_per_cm

        
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

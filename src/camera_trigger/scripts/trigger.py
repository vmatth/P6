#!/usr/bin/env python
from pynput import keyboard
import rospy
from read_camera.msg import Parcel #Parcel msg
from matplotlib.pyplot import contour
import cv2 
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import math
import imutils
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt


## When triggered
    # Get data from camera : subscribe to kinect2/hd/depth_image_rect
    # threshold based on minimal depth + threshold_tolerance (2-3 cm)
    # opencv -> 


class depth_dectect:
    def __init__(self):
        self.bridge = CvBridge()
        self.depth_data = None
        self.threshold_depth = None #distances higher than this are removed
        self.cam_height = 104.1 #cm

        self.cm_per_pixel = None #pix / cm ### fix so its not static
        self.depth_list = []
        self.counter = 0

    def on_press(self, key):
        try:
            print('special key pressed: {0}'.format(key))
            #print("key: ", key)
            #if key er enter
            if key == key.enter:
                self.depth_data = rospy.wait_for_message("/kinect2/hd/image_depth_rect", Image, timeout=None)
                self.thresholding(self.depth_data)
                #print("saved data: ", self.depth_data)
                #publish here to another topic
            elif key == key.space:
                self.depth_data = rospy.wait_for_message("/kinect2/hd/image_depth_rect", Image, timeout=None)
                self.thresholding(self.depth_data)
                #print("saved data: ", self.depth_data)
        except AttributeError:
            print("error :((((")

    def on_release(self, key):
            return False


    def get_depth(self, depth_image):

        depth_data_list = []
        x_arr = []
        y_arr = []
        lowest_pixel_depth = []

        lowest_depth = depth_image[0][0]
        lowest_pix = (0, 0)
        list = []

        for y in range(0, depth_image.shape[0], 5):
            for x in range(0, depth_image.shape[1], 5):
                if depth_image[y][x] > 0:
                    depth_data_list.append(depth_image[y][x])
                    x_arr.append(x)
                    y_arr.append(y)
                    if depth_image[y][x] < lowest_depth:
                        lowest_depth = depth_image[y][x]
                        lowest_pix = (x,y)
                        print("new lowest depth found: ", lowest_depth)
                        print("at pixel: ", x, y)
                        list.append(lowest_pix)
                        lowest_pixel_depth.append(lowest_depth)
    
        # find the 4 lowest pixel coordinates
        new_lowest_pix = list[(len(list))-4:len(list)]
        #print("new pixel array: ", new_lowest_pix)
        # find the 4 lowest pixel depths
        lowest_pixel_depth = lowest_pixel_depth[(len(lowest_pixel_depth))-4:len(lowest_pixel_depth)]  
        #print("depths: ", lowest_pixel_depth)
 
        # calculate average of the 4 lowest pixel depths
        self.threshold_depth = ((sum(lowest_pixel_depth) / len(lowest_pixel_depth))+30)/10  
        print("thresholding depth: ", self.threshold_depth)
        #cv2.circle(depth_image, lowest_pix, 3, (10000, 10000, 10000), -1)

        # fig = plt.figure(figsize = (10, 7))
        # ax = plt.axes(projection ="3d")
        
        # # Creating plot
        # ax.scatter3D(x_arr, y_arr, depth_data_list, color = "green")
        # plt.title("simple 3D scatter plot")
        
        # # show plot
        # plt.show()
        
                


    def thresholding(self, depth_data):
        try:
            depth_image = self.bridge.imgmsg_to_cv2(depth_data, "16UC1")
            depth_image = depth_image[263:785, 644:1050]
            height = depth_image.shape[0]
            width = depth_image.shape[1]
            print("height width", height, width)

            self.get_depth(depth_image)


            threshold_image = depth_image.copy()
            #loop all x y pixels
            for y in range(0,height):
                for x in range(0, width):
                    if depth_image[y, x] >= self.threshold_depth * 10 or depth_image[y,x] == 0:
                        threshold_image[y, x] = 0
                    else:
                        threshold_image[y, x] = 10000 #10 meters (should be white)


            converted_image = threshold_image.copy()
            converted_image = (threshold_image/256).astype('uint8')

            _, contours, _ = cv2.findContours(converted_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            print("contours :", len(contours))
            


            for cnt in contours:
                area = cv2.contourArea(cnt)
                
                if area > 500:
                    print("area: ", area)

                    #Create a rotated box around the parcel https://theailearner.com/tag/cv2-minarearect/
                    rect = cv2.minAreaRect(cnt)
                    box = cv2.boxPoints(rect)
                    box = np.int0(box)
                    #Find centerpoint using quik maffs
                    centerpoint_x = ((box[3][0] - box[1][0])/2) + box[1][0]
                    centerpoint_y = ((box[0][1] - box[2][1])/2) + box[2][1]
                    centerpoint = (centerpoint_x, centerpoint_y)
                    center = rect[0]

                    print("centerpont: ", centerpoint)
                    print("dim ,", depth_image.shape)

                    #Get pixel value at centerpoint
                    if(centerpoint_x >= depth_image.shape[1] or centerpoint_y >= depth_image.shape[0]): #Check if all 4 corners are available, or else it returns error
                        print("Centerpoint cannot be found in depth image")
                    else:
                        #Calculate dimensions & angle
                        distance_to_parcel = (depth_image[centerpoint_y][centerpoint_x]) / 10 #Calculate distance in [cm] 
                        #print("testtstst: ", depth_image[centerpoint_x][centerpoint_y])
                        print("distance to parcel: ", distance_to_parcel)
                        print("WIDTH IN PIXELS", rect[1][1])
                        print("LENGHT IN PIXELS", rect[1][0])

                        #self.cm_per_pixel = 0.00099063 * ((depth_image[centerpoint_y][centerpoint_x]) / 10)
                        self.cm_per_pixel = 0.000945918 * ((depth_image[centerpoint_y][centerpoint_x]) / 10) - 0.001137555
                        # print("cm per pixel: ", self.cm_per_pixel)
                        width = (rect[1][1])*self.cm_per_pixel
                        length = (rect[1][0])*self.cm_per_pixel
                        heigth = self.cam_height - distance_to_parcel
                        angle = rect[2]
                        print("Parcel width [cm]", width)
                        print("Parcel length [cm]", length)
                        print("Parcel height [cm]", height)
                        print("Parcel angle ", angle) 
                        # #Call parcel_pub function
                        # self.parcel_pub((rect[1][1])/self.cm_per_pixel, (rect[1][0])/self.cm_per_pixel, self.cam_height - distance_to_parcel, angle, centerpoint_x, centerpoint_y, distance_to_parcel)
                        # #Overlay centerpoint and contours to rgb and depth images


                    # 3d_in_cm = conversion * 2d_line * depth

                    cv2.drawContours(converted_image,[box],0,(255,255,255),2)
                    cv2.circle(converted_image, centerpoint, 3, (10000, 10000, 10000), -1)
                    


            cv2.imshow("Raw Depth Image (*16)", depth_image * 16)
            cv2.imshow("Threshold Image (*16)", threshold_image * 16)
            cv2.imshow("8-bit Threshold Image", converted_image * 16)
            cv2.waitKey(0)
            #cv2.waitKey(1)
            cv2.destroyAllWindows()
            # cv2.imshow("Depth Image Threshold (*16)", threshold_image * 16)
            # cv2.imshow("8-bit Threshold Image", converted_image * 16)
            # cv2.imshow("Threshold", thresh)

        except CvBridgeError as e:
            print(e)

if __name__ == '__main__':
    dd = depth_dectect()  
    rospy.init_node('trigger_node', anonymous=True)
    #define pub her
    pub = rospy.Publisher('/vision/frame_acq/parcel_info', Parcel, queue_size=10)
    print("hi jepper")
    while not rospy.is_shutdown():
        with keyboard.Listener(
            on_press=dd.on_press,
            on_release=dd.on_release) as listener:
            listener.join()
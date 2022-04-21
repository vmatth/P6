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
        #self.depth_to_parcel = None

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
        pixels = []

        lowest_depth = depth_image[0][0]
        lowest_pix = (0, 0)


        #blur?
        #depth_image = cv2.medianBlur(depth_image, 5)

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
                        # x_arr.append(x)
                        # y_arr.append(y)
                        
                    #pixels.append((x,y))
                    #depth_data_list.append(depth_image[y][x])
        cv2.circle(depth_image, lowest_pix, 3, (10000, 10000, 10000), -1)
        #print("distance: ", depth_image[300][300])
        #depth_min = min(depth_data_list)
        #index_min = min(range(len(depth_data_list)), key=depth_data_list.__getitem__)
        #min_pixel = pixels[index_min]
        #pixels er i x,y
       # print("Distance", depth_min)
       # print("min pixel", min_pixel)
       # print("index min", index_min)
        #circle requires col, rows
        #cv2.circle(depth_image, lowest_pix, 3, (10000, 10000, 10000), -1)

        fig = plt.figure(figsize = (10, 7))
        ax = plt.axes(projection ="3d")
        
        # Creating plot
        ax.scatter3D(x_arr, y_arr, depth_data_list, color = "green")
        plt.title("simple 3D scatter plot")
        
        # show plot
        plt.show()
        
                


    def thresholding(self, depth_data):
        try:
            depth_image = self.bridge.imgmsg_to_cv2(depth_data, "16UC1")
            depth_image = depth_image[263:785, 635:1058]
            height = depth_image.shape[0]
            width = depth_image.shape[1]
            print("height width", height, width)

            
            self.get_depth(depth_image)





            cv2.imshow("Raw Depth Image (*16)", depth_image * 16)
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
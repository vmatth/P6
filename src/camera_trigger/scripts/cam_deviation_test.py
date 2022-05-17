#!/usr/bin/env python
from pynput import keyboard
import rospy
from read_camera.msg import Parcel #Parcel msg
from matplotlib.pyplot import contour, imshow
import cv2 
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import math
import imutils
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
from geometry_msgs.msg import Point
from numpy.linalg import inv


## When triggered
    # Get data from camera : subscribe to kinect2/hd/depth_image_rect
    # threshold based on minimal depth + threshold_tolerance (2-3 cm)
    # opencv -> 


class depth_dectect:
    def __init__(self):
        self.bridge = CvBridge()
        self.depth_data = None
        self.threshold_depth = None #distances higher than this are removed
        self.cam_height_principle_point = 1016
        self.camera_offset = 2.4 #cm. The height is subtracted by this value. (As there is a small offset in the kinect2 camera)
        self.focal_length_x = 1.0663355230063235 * 10**3 #px
        self.focal_length_y = 1.0676521964588569 * 10**3 #px
        self.pixel_size = 0.0031 #mm/px
        self.sensor_width_mm = 5.952 #mm
        self.sensor_length_mm = 3.348 #mm
        self.sensor_width_px = 1920 #px 
        self.sensor_length_px = 1080 #px

        self.depth_list = []
        self.counter = 0

        self.pub = rospy.Publisher('/vision/parcel', Parcel, queue_size=10)


    def on_press(self, key):
        try:
            print('special key pressed: {0}'.format(key))
            #print("key: ", key)
            #if key er enter
            if key == key.ctrl_r:
                self.depth_data = rospy.wait_for_message("/kinect2/hd/image_depth_rect", Image, timeout=None)
                self.thresholding(self.depth_data)
                #print("saved data: ", self.depth_data)
                #publish here to another topic
            # elif key == key.space:
            #     self.depth_data = rospy.wait_for_message("/kinect2/hd/image_depth_rect", Image, timeout=None)
            #     self.thresholding(self.depth_data)
            #     #print("saved data: ", self.depth_data)
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
        lowest_pixel_depth = lowest_pixel_depth[(len(lowest_pixel_depth))-4:len(lowest_pixel_depth)-1] # average
        #lowest_pixel_depth = lowest_pixel_depth[(len(lowest_pixel_depth))-4:len(lowest_pixel_depth)] # median
        print("depths: ", lowest_pixel_depth)
 
        # calculate average of the 4 lowest pixel depths
        self.threshold_depth = ((sum(lowest_pixel_depth) / len(lowest_pixel_depth))+40)/10  #average
        #self.threshold_depth = np.median(lowest_pixel_depth) / 10.0 #median. Divide by 10.0 to go from mm to m
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
            crop_min_y = 243
            crop_max_y = 785
            crop_min_x = 638
            crop_max_x = 1050
            depth_image = depth_image[crop_min_y:crop_max_y, crop_min_x:crop_max_x]
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


            #Calculate dimensions & angle
            for y in range(0,height):
                for x in range(0, width):
                    XYZ = self.calculate_XYZ(int_x + crop_min_x, depth_image[x] + crop_min_y, 1, True)
                    a = self.cam_height_principle_point 
                    b = XYZ[1] * 1000 #m to mm (X coordinate: [0], Y coordinate : [1])
                    #d = XYZ[0] * 1000 #m to mm (X coordinate: [0], Y coordinate : [1])

                    #calculate c

                    c = math.sqrt(a**2 + b**2)
                    #c = math.sqrt(a**2 + b**2 + d**2)

                    print("a", a)
                    print("b", b)
                    #print("d", d)
                    print("c", c)
                
                    height = (c - distance_to_parcel) / 10.0 #mm to cm                
                    XYZ = self.calculate_XYZ(centerpoint_x + crop_min_x, centerpoint_y + crop_min_y, 1, True)



            #cx = 890
            #cy = 535
            
            #Calculate parcel height based on distance from centerpoint
            # cam
            #   | \
            #   |  \ c
            # a |   \
            #   |    \
            #   ------ (XY)
            #      b



  

            #print("XYZ", XYZ[0])

            #todo: lav om til hand eye cal
            pos_x = XYZ[0][0] * 100
            pos_y = XYZ[1][0] * 100
            pos_z = height
            print("posx: ", pos_x, "posy: ", pos_y, "posz: ", pos_z)
            #print("Table",distance_to_table)
            
            #cv2.circle(uncropped_image, (890, 535), 3, (10000, 10000, 10000), -1)
            #cv2.circle(converted_image, (890 - 638, 535 - 242), 3, (10000, 10000, 1000), -1)

            #cv2.imshow("Uncropped Image (*16)", uncropped_image * 16)
            cv2.imshow("Raw Depth Image (*16)", depth_image * 16)
            cv2.imshow("Threshold Image (*16)", threshold_image * 16)
            cv2.imshow("8-bit Threshold Image", converted_image * 16)
            cv2.waitKey(0)
            cv2.destroyAllWindows()

        except CvBridgeError as e:
            print(e)
    
if __name__ == '__main__':
    dd = depth_dectect()  
    rospy.init_node('trigger_node', anonymous=True)
    #define pub her
    pub = rospy.Publisher('/vision/parcel', Parcel, queue_size=10)
    print("hi jepper")
    while not rospy.is_shutdown():
        with keyboard.Listener(
            on_press=dd.on_press,
            on_release=dd.on_release) as listener:
            listener.join()
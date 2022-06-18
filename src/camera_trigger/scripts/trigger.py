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
from geometry_msgs.msg import Point
from numpy.linalg import inv
import seaborn as sns

## When triggered
    # Get data from camera : subscribe to kinect2/hd/depth_image_rect
    # threshold based on minimal depth + threshold_tolerance (2-3 cm)
    # opencv -> 


class depth_dectect:
    def __init__(self):
        self.bridge = CvBridge()
        self.depth_data = None
        self.threshold_depth = None #distances higher than this are removed
        self.cam_height_principle_point = 1025
        self.camera_offset = 2.4 #cm. The height is subtracted by this value. (As there is a small offset in the kinect2 camera) not currently used : ]
        self.focal_length_x = 1.0685132562503038 * 10**3 #px
        self.focal_length_y = 1.0691031314129875 * 10**3 #px 
        self.pixel_size = 0.0031 #mm/px
        self.pixel_size_tof = 0.01
        self.sensor_width_mm = 5.952 #mm
        self.sensor_length_mm = 3.348 #mm
        self.sensor_width_px = 1920 #px 
        self.sensor_length_px = 1080 #px
        self.principle_point_x = 946.4624
        self.principle_point_y = 537.20

        self.depth_list = []
        self.counter = 0

        self.pub = rospy.Publisher('/vision/parcel', Parcel, queue_size=10)


    def on_press(self, key):
        try:
            print('special key pressed: {0}'.format(key))
            #print("key: ", key)
            #if key er enter
            if key == key.space:
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
                        # print("new lowest depth found: ", lowest_depth)
                        # print("at pixel: ", x, y)
                        list.append(lowest_pix)
                        lowest_pixel_depth.append(lowest_depth)
    
        # find the 4 lowest pixel coordinates
        new_lowest_pix = list[(len(list))-4:len(list)]
        #print("new pixel array: ", new_lowest_pix)
        # find the 4 lowest pixel depths
        lowest_pixel_depth = lowest_pixel_depth[(len(lowest_pixel_depth))-4:len(lowest_pixel_depth)-1] # average
        #lowest_pixel_depth = lowest_pixel_depth[(len(lowest_pixel_depth))-4:len(lowest_pixel_depth)] # median
        #print("depths: ", lowest_pixel_depth)
 
        # calculate average of the 4 lowest pixel depths
        self.threshold_depth = ((sum(lowest_pixel_depth) / len(lowest_pixel_depth))+40)/10  #average
        #self.threshold_depth = np.median(lowest_pixel_depth) / 10.0 #median. Divide by 10.0 to go from mm to m
        #print("thresholding depth: ", self.threshold_depth)
        #cv2.circle(depth_image, lowest_pix, 3, (10000, 10000, 10000), -1)

        # fig = plt.figure(figsize = (10, 7))
        # ax = plt.axes(projection ="3d")
        
        # # Creating plot
        # ax.scatter3D(x_arr, y_arr, depth_data_list, color = "green")
        # plt.title("simple 3D scatter plot")
        #ax = sns.heatmap(depth_image, vmin= lowest_depth, vmax= 1009, cmap="autumn")


        
        # show plot
       # plt.show()
        
                


    def thresholding(self, depth_data):
        try:
            depth_image = self.bridge.imgmsg_to_cv2(depth_data, "16UC1")
            uncropped_image = depth_image.copy()
            crop_min_y = 345
            crop_max_y = 825
            crop_min_x = 638
            crop_max_x = 1050
            depth_image = depth_image[crop_min_y:crop_max_y, crop_min_x:crop_max_x]
            height = depth_image.shape[0]
            width = depth_image.shape[1]
            #print("height width", height, width)

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

            counter = 0
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area > 500:
                    counter = counter + 1
            

           # print("Contours with area over 500: ", counter)

            if(counter == 1):
                for cnt in contours:
                    area = cv2.contourArea(cnt)
                    
                    if area > 500:
                        #print("area: ", area)

                        #Create a rotated box around the parcel https://theailearner.com/tag/cv2-minarearect/
                        rect = cv2.minAreaRect(cnt)
                        box = cv2.boxPoints(rect)
                        box = np.int0(box)
                        print("box box, box box: ", box)
                        #Find centerpoint using quik maffs
                        centerpoint_x = ((box[3][0] - box[1][0])/2) + box[1][0]
                        centerpoint_y = ((box[0][1] - box[2][1])/2) + box[2][1]
                        centerpoint = (centerpoint_x, centerpoint_y)
                        center = rect[0]

                        # print("centerpont: ", centerpoint)
                        # print("dim ,", depth_image.shape)

                        #Get pixel value at centerpoint
                        if(centerpoint_x >= depth_image.shape[1] or centerpoint_y >= depth_image.shape[0]): #Check if all 4 corners are available, or else it returns error
                            print("Centerpoint cannot be found in depth image")
                        else:
                            #Calculate dimensions & angle
                            distance_to_parcel = depth_image[centerpoint_y][centerpoint_x] #Calculate distance in [mm] 
                            #distance_to_table = depth_image[200][200]
                            cv2.circle(converted_image, (0,0), 3, (10000, 10000, 10000), -1)
                            #print("testtstst: ", depth_image[centerpoint_x][centerpoint_y])

                            print("distance to parcel: ", distance_to_parcel)

                            x1 = (distance_to_parcel / self.focal_length_x) * (box[0][0] + crop_min_x - self.principle_point_x)
                            print("test: X1", x1)
                            y1 = (distance_to_parcel / self.focal_length_y) * (box[0][1] + crop_min_y - self.principle_point_y)
                            print("test: Y1", y1)

                            x2 = (distance_to_parcel / self.focal_length_x) * (box[1][0] + crop_min_x - self.principle_point_x)
                            print("test: X2", x2)
                            y2 = (distance_to_parcel / self.focal_length_y) * (box[1][1] + crop_min_y - self.principle_point_y)
                            print("test: Y2", y2)

                            x3 = (distance_to_parcel / self.focal_length_x) * (box[3][0] + crop_min_x - self.principle_point_x)
                            print("test: X3", x3)
                            y3 = (distance_to_parcel / self.focal_length_y) * (box[3][1] + crop_min_y - self.principle_point_y)
                            print("test: Y3", y3)

                            width = math.sqrt((x2-x1)**2+(y2-y1)**2) / 10
                            length = math.sqrt((x3-x1)**2+(y3-y1)**2) / 10
                            # print("length: ", length)

                            # object_width_pixels = rect[1][1]
                            # object_length_pixels = rect[1][0]
                            
                            # print("WIDTH IN PIXELS", object_width_pixels)
                            # print("LENGTH IN PIXELS", object_length_pixels)

                            # object_width_on_sensor = self.sensor_width_mm * object_width_pixels / self.sensor_width_px
                            # object_length_on_sensor = self.sensor_length_mm * object_length_pixels / self.sensor_length_px

                            
                            # print("Bw: ", object_width_on_sensor)
                            # print("Bh: ", object_height_on_sensor)
                            # print("focal: ", self.focal_length*self.pixel_size)
                            # width = (distance_to_parcel * object_width_on_sensor / (self.focal_length_x*self.pixel_size)) / 10.0
                            # length = (distance_to_parcel * object_length_on_sensor / (self.focal_length_y*self.pixel_size)) / 10.0

                        

                            # small_bw = 1/((1/(self.focal_length_x*self.pixel_size))-(1/float(distance_to_parcel)))
                            # #print("small_bw: ", small_bw)
                            # small_bh = 1/((1/(self.focal_length_y*self.pixel_size))-(1/float(distance_to_parcel)))
                            # #print("small_bh: ", small_bh)

                            # width_b = (distance_to_parcel * object_width_on_sensor / (small_bw)) / 10.0
                            # length_b = (distance_to_parcel * object_length_on_sensor / (small_bh)) / 10.0


                            #Calculate parcel height based on distance from centerpoint
                            # cam
                            #   | \
                            #   |  \ c
                            # a |   \
                            #   |    \
                            #   ------ (XY)
                            #      b
                            #XYZ = self.calculate_XYZ(centerpoint_x + crop_min_x, centerpoint_y + crop_min_y, 1, False)
                            # a = self.cam_height_principle_point 
                            # b = XYZ[1] * 1000 #m to mm (X coordinate: [0], Y coordinate : [1])
                            # #calculate c

                            # c = math.sqrt(a**2 + b**2)
                            
                            # print("a", a)
                            # print("b", b)
                            # print("c", c)
                        
                            height = float((float(self.cam_height_principle_point) - float(distance_to_parcel)) / 10.0) #mm to cm
                            angle = rect[2]
                            # print("Parcel width [cm]", width)
                            # print("Parcel length [cm]", length)
                            # print("Parcel height [cm]", height)
                            # print("Parcel angle ", angle)

                            #print("distance to parcel: ", distance_to_parcel)

                            #Calculate centerpoint in world coordinates
                            centerpoint_x_world = (distance_to_parcel / self.focal_length_x) * (centerpoint_x + crop_min_x - self.principle_point_x)
                            centerpoint_y_world = (distance_to_parcel / self.focal_length_y) * (centerpoint_y + crop_min_y - self.principle_point_y)

                            #todo: lav om til hand eye cal
                            pos_x = centerpoint_x_world / 10 #XYZ[0][0] * 100
                            pos_y = centerpoint_y_world / 10 #XYZ[1][0] * 100
                            pos_z = height
                           # print("posx: ", pos_x, "posy: ", pos_y, "posz: ", pos_z)
                            #print("Table",distance_to_table)

                            # #Call parcel_pub function
                            self.parcel_pub(Point(width, length, height), angle, Point(pos_x, pos_y, pos_z))
                            # self.parcel_pub((rect[1][1])/self.cm_per_pixel, (rect[1][0])/self.cm_per_pixel, self.cam_height - distance_to_parcel, angle, centerpoint_x, centerpoint_y, distance_to_parcel)
                        # #Overlay centerpoint and contours to thresholded images
                        cv2.drawContours(converted_image,[box],0,(255,255,255),2)
                        cv2.circle(converted_image, centerpoint, 3, (10000, 10000, 10000), -1)

            #cv2.circle(uncropped_image, (890, 535), 3, (10000, 10000, 10000), -1)
            #cv2.circle(converted_image, (890 - 638, 535 - 242), 3, (10000, 10000, 1000), -1)

            # cv2.imshow("Uncropped Image (*16)", uncropped_image * 16)
            # cv2.imshow("Raw Depth Image (*16)", depth_image * 16)
            # cv2.imshow("Threshold Image (*16)", threshold_image * 16)
            cv2.imshow("8-bit Threshold Image", converted_image * 16)
            cv2.waitKey(0)
            cv2.destroyAllWindows()

        except CvBridgeError as e:
            print(e)

    def parcel_pub(self, size, angle, centerpoint):
        print("----------")
        print("Publishing parcel to /vision/parcel_raw")
        print("Parcel size [cm]", size)
        print("Parcel angle ", angle)
        print("Parcel centerpoint", centerpoint)
        msg = Parcel()
        #The bin packing algorithm expects the size to be in cm integers (no decimals)
        msg.size = size
        msg.angle = angle
        msg.centerpoint = centerpoint

        
        self.pub.publish(msg)

    #Calcualtes world coordinates from pixel u,v.
    #useCameraParameters = True calculates with respect to the camera frame
    #useCameraParameters = False calculates with respect to robot frame
    def calculate_XYZ(self, u, v, z):
        s = 1
        A = np.matrix([[1.0685132562503038e+03, 0, 9.4646247882698822e+02], [0, 1.0691031314129875e+03, 5.3720184420921953e+02], [0., 0., 1.]])
        R = np.matrix([[9.9986175081970585e-01, -1.7907271803387413e-03, -1.6530957138594669e-02],[1.7879233427828131e-03, 9.9999838466413493e-01, -1.8438883127595748e-04], [1.6531260625638637e-02, 1.5480725552454968e-04, 9.9986333739008593e-01]])
        t = np.array([[-3.5600027180614863e-02, 1.3910620566261074e-03, 9.7996622621221927e-03]])

        # print("u,v,z", u, v, z)
        # print("A: ", A)
        # print("R: ", R)
        # print("t: ", t)

        uv1 = np.array([[u,v,z]])

        #Transpose uv1
        uv1 = uv1.T
        #print("uv1 Transpose: ", uv1)

        #Times by scaling factor
        s_uv1 = s*uv1
        #print("s*uv1", s_uv1)

        #Invert A
        A_inv = inv(A)
        #print("A^-1: ", A_inv)

        #A^-1 * s_uv1
        xyz_c = A_inv.dot(s_uv1)
        #print("A^-1 * s_uv1: ", xyz_c)

        #Transpose t
        t = t.T
        #print("T Transpose: ", t)

        #Substract t
        xyz_c = xyz_c - t
        #print("Subtracted by T: ", xyz_c)

        #Invert R
        R_inv = inv(R)
        #print("R^-1: ", R_inv)

        XYZ = R_inv.dot(xyz_c)
        #print("Final XYZ", XYZ)

        return XYZ

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
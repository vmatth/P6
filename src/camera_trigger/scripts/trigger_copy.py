#!/usr/bin/env python
from copy import copy
import cv2 
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import rospy
from matplotlib import pyplot as plt
import message_filters
from read_camera.msg import Parcel
from mpl_toolkits import mplot3d

class image_converter:

    def __init__(self):
        self.bridge = CvBridge()
        self.rgb_sub = message_filters.Subscriber("/kinect2/hd/image_color", Image)
        self.depth_sub = message_filters.Subscriber("/kinect2/hd/image_depth_rect", Image)
        self.ts = message_filters.TimeSynchronizer([self.rgb_sub, self.depth_sub], 10)
        self.ts.registerCallback(self.camera_callback)

        ###########################################################################
        ################################ SETUP ##################################
        ##########################################################################
        
        #Udregn hver gang setup aendres, ved at dividere bredden eller hoejden i pixels, over bredden eller hoejden af et kendt objekt i cm.
        self.pix_per_cm = 10.2

        #Skal kalibreres hver gang setup aendres.
        self.cam_height = 111.5

        #self.counter 


    def camera_callback(self, rgb_data, depth_data):
        #rospy.loginfo("Receiving info from image topic!")
        try:
            #Get rgb & depth images for this frame
            rgb_image = self.bridge.imgmsg_to_cv2(rgb_data)
            depth_image = self.bridge.imgmsg_to_cv2(depth_data, "16UC1")

            #Crop image to only have the parcel in focus - removes unnecesarry items
            rgb_image = rgb_image[263:765, 658:1030]
            depth_image = depth_image[263:765, 658:1030]            

            distance_to_table = depth_image[200][200]

            depth_data_list = []
            x_arr = []
            y_arr = []

            # for y in range(3):
            #     for x in range(3):
            #         if depth_image[y][x]:
            #             depth_data_list.append(depth_image[y][x])


            # pis = depth_image[0][0]
            # pis2 = depth_image[1][0]
            # pis3 = depth_image[2][0]

            # print("Depth",depth_data_list)
            # print("Depth 1",pis)
            # print("Depth 4",pis2)
            # print("Depth 7",pis3)

            
            for y in range(0, depth_image.shape[0],5):
                for x in range(0, depth_image.shape[1],5):
                    if depth_image[y][x]:
                        depth_data_list.append(depth_image[y][x])    
                        x_arr.append(x)
                        y_arr.append(y)




            fig = plt.figure(figsize = (10, 7))
            ax = plt.axes(projection ="3d")
            
            # Creating plot
            ax.scatter3D(x_arr, y_arr, depth_data_list, cmap='viridis', linewidth = 0.5)
            plt.title("simple 3D scatter plot")
            
            # Creating plot
            #ax.contour3D(x_arr, y_arr, depth_data_list, 50, cmap='binary')
            #plt.title("simple 3D contour plot")
            

            # show plot
            plt.show()


            #Show images
            cv2.imshow("Raw Image", rgb_image)
            cv2.imshow("Depth Image", depth_image)
            #cv2.imshow("Grayscale Image", grayscale_image)
            #cv2.imshow("Blurred Image", blur_image)
            #cv2.imshow("Threshold", thresh_image)
            #cv2.imshow("Canny Edge Detection", edge_image)
            #cv2.imshow("Morphology Image", opening_image)
            #cv2.imshow("RGB Image w/ Overlay", rgb_overlay)
            #cv2.imshow("Depth Image w/ Overlay", depth_overlay * 16)
            #Save image for testing purposes
            # cv2.imwrite('test.jpg', blur_image)
            #cv2.imshow("Raw Image", rgb_image)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
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
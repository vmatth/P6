#!/usr/bin/env python
import rospy
import cv2 
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from read_camera.msg import Parcel #Parcel msg
from bin_packing.msg import Workspace #workspace msg
from geometry_msgs.msg import Point #point type
from bin_packing.convertTo2DArray import convertToMultiArray #convert function


class depth_detect:
    def __init__(self):
        self.bridge = CvBridge()
        #subscribe to depth image
        self.sub = rospy.Subscriber("/kinect2/hd/image_depth_rect", Image, self.callback)
        self.pub = rospy.Publisher('/vision/parcel_raw', Parcel, queue_size=10)
        self.ws_pub = rospy.Publisher("/workspace/update_height_map", Workspace, queue_size=100)

        self.cam_height = 128.5 #cm
        self.distance_threshold = 99 #distancer higher than this are removed
        self.threshold_tolerance = 0 #cm
        self.pix_per_cm = 11.4 #pix / cm


    def callback(self, depth_data):
        #rospy.loginfo(rospy.get_caller_id() + "I heard %s", depth_data.data)    
        try:    
            depth_image = self.bridge.imgmsg_to_cv2(depth_data, "16UC1")
            depth_image = depth_image[300:895, 635:1375]   #[y,x]
            height = depth_image.shape[0]
            width = depth_image.shape[1]
            print("crop width height", width, height)

            # #get distance to a pixel
            # distance_to_parcel = depth_image[300][300] #Calculate distance in [cm] 
            # print("Wall depth: ", distance_to_parcel)
            # #draw circle
            # depth_image = cv2.circle(depth_image, (300,300), radius=3, color=(255, 255, 255), thickness=-1)

            threshold_image = depth_image.copy()
            #loop all x y pixels
            for y in range(0,height):
                for x in range(0, width):
                    if depth_image[y, x] >= self.distance_threshold * 10: #to mm
                        threshold_image[y, x] = 0
                    else:
                        threshold_image[y, x] = 10000 #10 meters (should be white)

            #Convert to from 16-bit to 8-bit (for contours)
            converted_image = threshold_image.copy()
            converted_image = (threshold_image/256).astype('uint8')

            #Find contours
            _, contours, _= cv2.findContours(converted_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            #Creat empty arrays for finding the min and max (x,y) coordinates
            x_array = []
            y_array = []
        
            #Crop area
            for cntr in contours:
                x,y,w,h = cv2.boundingRect(cntr)
                #depth_image = cv2.rectangle(depth_image, (min_x, max_x), (min_y, max_y), (10000, 10000, 10000), 2)
                #cv2.rectangle(depth_image, (x,y), (w+x,y+h),(10000,10000,10000),2)
                print("x,y,w,h:",x,y,w,h)
                x_array.append(x)
                x_array.append(x+w)
                y_array.append(y)
                y_array.append(y+h)

            min_x = min(x_array)
            #print("min_x: ", min_x)
            max_x = max(x_array)
            #print("max_x: ", max_x)

            min_y = min(y_array)
            #print("min_y: ", min_y)
            max_y = max(y_array)
            #print("max_y: ", max_y)
            
            #Draw rectangle on roller cage
            cv2.rectangle(threshold_image, (min_x, min_y), (max_x, max_y), (10000, 10000, 10000), 2)

            # #Crop image to only have the parcel in focus - removes unnecesarry items
            cropped_image = converted_image.copy()
            cropped_image = converted_image[min_y:max_y, min_x:max_x]
            cropped_depth_image = depth_image[min_y:max_y, min_x:max_x]

            #Resize
            scale_percent = 20 # percent of original size
            resized_width = int(cropped_image.shape[1] * scale_percent / 100)
            resized_height = int(cropped_image.shape[0] * scale_percent / 100)
            dim = (resized_width, resized_height)
            resized_image = cv2.resize(cropped_image, dim, interpolation = cv2.INTER_AREA)
            resized_depth_image = cv2.resize(cropped_depth_image, dim, interpolation = cv2.INTER_AREA)
 

            #Find size

            
            # size_x = max_x - min_x
            # print("size_x: ", size_x)
            # size_y = max_y - min_y
            # print("size_y: ", size_y)

            # height = depth_image.shape[0]
            # width = depth_image.shape[1]




            # #for loop for depth image
            # for x in range(height):
            #     for y in range(width):
            #         #depth_image[x,y]  
            #         if depth_image[x,y] == 0:
                        
            #             cv2.circle(depth_image, (y,x), radius=3, color=(10000, 10000, 10000), thickness=2)
                    
            #             #print("depth: ", depth_image[x,y])          

            height_map = [[0.0 for i in range(resized_height)] for j in range(resized_width)] 
            for i in range(resized_height):
                for j in range(resized_width):

            #height_map = [[0.0 for i in range(7)] for j in range(5)] 
            #for i in range(7): #y
                #for j in range(5): #x
                    #if depth_image[i,j] > 0: 
                    #print("i: ", i, "j: ", j)
                    #print("distance at (", i, j, "): ", depth_image[i,j])
                    height_map[j][resized_height-i-1] = self.cam_height - (resized_depth_image[i,j]/10)
            #print("hm", height_map)

            #Resize height map :D
            #Convert 2D to np array
            #resizes

            #Publish height map to /workspace/update_height_map
            self.workspace_pub(height_map, Point(resized_width, resized_height, 0))



            cv2.waitKey(3)
            cv2.imshow("Raw Depth Image (*16)", depth_image * 16)
            cv2.imshow("Depth Image Threshold (*16)", threshold_image * 16)
            cv2.imshow("Cropped Image", cropped_image * 16)   
            cv2.imshow("Depth", depth_image * 16)
            cv2.imshow("Resized image", resized_image * 16)
            cv2.imshow("asdasdasd", cropped_depth_image * 16)
            cv2.imshow("lol", resized_depth_image * 16)
        except CvBridgeError as e:
            print(e)

        

    def workspace_pub(self, height_map, workspace_size):
        print("-------------")
        print("Publishing parcel to /workpsace/update_height_map")
        msg = Workspace()
        msg.size = Point(workspace_size.x, workspace_size.y, 16) #fix workspace size y
        msg.height_map = convertToMultiArray(height_map, workspace_size.x)

        self.ws_pub.publish(msg)

#  depth_map = cv2.normalize(depth_map, None, 0, 1, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_32F)


def main():
    dd = depth_detect()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    rospy.init_node('inbound_parcel_detection', anonymous=True)
    main()
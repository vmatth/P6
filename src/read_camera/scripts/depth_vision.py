#!/usr/bin/env python
import rospy
import cv2 
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

class depth_detect:
    def __init__(self):
        self.bridge = CvBridge()
        #subscribe to depth image
        self.sub = rospy.Subscriber("/kinect2/hd/image_depth_rect", Image, self.callback)
        self.distance_to_wall = 950
        self.pix_per_cm = 12


    def callback(self, depth_data):
        #rospy.loginfo(rospy.get_caller_id() + "I heard %s", depth_data.data)    
        try:    
            depth_image = self.bridge.imgmsg_to_cv2(depth_data, "16UC1")
            #depth_image = depth_image[263:785, 635:1058]   
            height = depth_image.shape[0]
            width = depth_image.shape[1]

            #get distance to a pixel
            distance_to_parcel = depth_image[300][300] #Calculate distance in [cm] 
            print("Wall depth: ", distance_to_parcel)
            #draw circle
            depth_image = cv2.circle(depth_image, (300,300), radius=3, color=(255, 255, 255), thickness=-1)

            cv2.imshow("Depth *16", depth_image * 16)
            threshold_image = depth_image.copy()
            #loop all x y pixels
            for y in range(0,height):
                for x in range(0, width):
                        threshold_image[y, x] = 0 if depth_image[y, x] >= self.distance_to_wall else 5000

            #Find contours
            _, contours, _= cv2.findContours(threshold_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            i = 0
            for cnt in contours:
                print("------------------")
                print("Parcel [",i,"] found")

                # area = cv2.contourArea(contours[i])
                # print("area: ", area)
                # if area > 3000 and area < 130000:
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
                    cv2.drawContours(threshold_image,[box],0,(255,255,255),2)
                    threshold_image = cv2.circle(threshold_image, centerpoint, radius=3, color=(0, 255, 0), thickness=-1)

            i = i + 1

            cv2.imshow("Depth (Threshold)", threshold_image * 16)
                
            #cv2.imshow("Depth", depth_image * 16)
            cv2.waitKey(33)
            #find distance to wall

            #custom "threshold" removing all points that are the wall.
        except CvBridgeError as e:
            print(e)


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
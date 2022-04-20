#!/usr/bin/env python
import rospy
import cv2 
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from read_camera.msg import Parcel #Parcel msg

class depth_detect:
    def __init__(self):
        self.bridge = CvBridge()
        #subscribe to depth image
        self.sub = rospy.Subscriber("/kinect2/hd/image_depth_rect", Image, self.callback)
        self.pub = rospy.Publisher('/vision/parcel_raw', Parcel, queue_size=10)

        self.cam_height = 104.1 #cm
        self.distance_threshold = 99 #distancer higher than this are removed
        self.threshold_tolerance = 0 #cm
        self.pix_per_cm = 11.4 #pix / cm


    def callback(self, depth_data):
        #rospy.loginfo(rospy.get_caller_id() + "I heard %s", depth_data.data)    
        try:    
            depth_image = self.bridge.imgmsg_to_cv2(depth_data, "16UC1")
            depth_image = depth_image[175:925, 635:1375]   #[y,x]
            height = depth_image.shape[0]
            width = depth_image.shape[1]
            print("hheight width", height, width)

            # #get distance to a pixel
            # distance_to_parcel = depth_image[300][300] #Calculate distance in [cm] 
            # print("Wall depth: ", distance_to_parcel)
            # #draw circle
            # depth_image = cv2.circle(depth_image, (300,300), radius=3, color=(255, 255, 255), thickness=-1)

            threshold_image = depth_image.copy()
            #loop all x y pixels
            for y in range(0,height):
                for x in range(0, width):
                    if depth_image[y, x] >= self.distance_threshold * 10:
                        threshold_image[y, x] = 0
                    else:
                        threshold_image[y, x] = 10000 #10 meters (should be white)

            #Convert to from 16-bit to 8-bit (for contours)
            converted_image = threshold_image.copy()
            converted_image = (threshold_image/256).astype('uint8')
        


            #Find contours
            _, contours, _= cv2.findContours(converted_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)


            i = 0
            for cnt in contours:
                area = cv2.contourArea(contours[i])
                if area > 3000: #and area < 130000:
                    print("------------------")
                    #print("Parcel [",i,"] found")
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
                    #Get pixel value at centerpoint
                    if(centerpoint_x >= depth_image.shape[0] or centerpoint_y >= depth_image.shape[1]): #Check if all 4 corners are available, or else it returns error
                        print("Centerpoint cannot be found in depth image")
                    else:
                        #Calculate dimensions & angle
                        distance_to_parcel = (depth_image[centerpoint_x][centerpoint_y]) / 10 #Calculate distance in [cm] 
                        #print("testtstst: ", depth_image[centerpoint_x][centerpoint_y])
                        print("distance to parcel: ", distance_to_parcel)
                        print("WIDTH IN PIXELS", rect[1][1])
                        print("HEIGHT IN PIXELS", rect[1][0])
                        #print("width in pixels", rect[1][1])
                        #print("length in pixels", rect[1][0])
                        width = (rect[1][1])/self.pix_per_cm  
                        length = (rect[1][0])/self.pix_per_cm
                        heigth = self.cam_height - distance_to_parcel
                        angle = rect[2]
                        #Call parcel_pub function
                        self.parcel_pub((rect[1][1])/self.pix_per_cm, (rect[1][0])/self.pix_per_cm, self.cam_height - distance_to_parcel, angle, centerpoint_x, centerpoint_y, distance_to_parcel)
                        #Overlay centerpoint and contours to rgb and depth images
                        cv2.drawContours(converted_image,[box],0,(255,255,255),2)
                        converted_image = cv2.circle(converted_image, centerpoint, radius=3, color=(255, 255, 255), thickness=-1)

                i = i + 1

            cv2.waitKey(3)
            cv2.imshow("Raw Depth Image (*16)", depth_image * 16)
            cv2.imshow("Depth Image Threshold (*16)", threshold_image * 16)
            cv2.imshow("8-bit Threshold Image", converted_image * 16)   
            #cv2.imshow("Depth", depth_image * 16)

            #find distance to wall

            #custom "threshold" removing all points that are the wall.
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
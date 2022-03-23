#!/usr/bin/env python
import cv2 
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import rospy


class image_converter:

    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/kinect2/qhd/image_depth_rect", Image, self.image_callback)


    def image_callback(self, data):
        rospy.loginfo("Receiving info from image topic!")
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
        except CvBridgeError as e:
            print(e)
        
        self.videoCapture(cv_image)
        cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)

    def videoCapture(self, data):
        cv2.VideoCapture(0) 


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
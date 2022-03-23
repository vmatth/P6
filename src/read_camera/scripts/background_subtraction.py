#!/usr/bin/env python
import cv2 
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import rospy


class image_converter:

    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/kinect2/qhd/image_depth_rect", Image, self.image_callback)
        self.cv_background = None
        self.save_background = True

    def image_callback(self, data):
        rospy.loginfo("Receiving info from image topic!")
        try:
            #Get image for this frame
            cv_image = self.bridge.imgmsg_to_cv2(data, '8UC1')
            #Normalize Image
            cv_image_array = np.array(cv_image, dtype = np.dtype('f8'))
            cv_image_norm = cv2.normalize(cv_image_array, cv_image_array, 0, 1, cv2.NORM_MINMAX)
            #Get background on first frame
            if self.save_background is True:
                self.cv_background = cv_image_norm
                self.save_background = False

            diff = cv2.absdiff(self.cv_background, cv_image_norm)

            cv2.imshow ("Background Subtraction", diff)
            cv2.imshow("Image window", cv_image_norm)
            cv2.imshow("Background", self.cv_background)
            cv2.waitKey(3)

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
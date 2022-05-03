#!/usr/bin/env python
import rospy
import cv2 
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from matplotlib import pyplot as plt
import message_filters
from read_camera.msg import Parcel #Parcel msg


class eye_hand_calibration:

    def __init__(self):
        self.bridge = CvBridge()
        self.rgb_sub = message_filters.Subscriber("/kinect2/hd/image_color", Image)
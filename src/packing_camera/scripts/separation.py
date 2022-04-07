#!/usr/bin/env python
from xml.dom import HierarchyRequestErr
import cv2
from cv2 import blur
from cv2 import contourArea
from cv2 import erode
from cv2 import Canny
from matplotlib.contour import ContourSet
import numpy as np
from matplotlib import image, pyplot as plt
import sys
import math
#from skimage.transform import (hough_line, hough_line_peaks)


#import image
rgb = cv2.imread('test2.jpg', cv2.IMREAD_UNCHANGED)
 
#################### resizing #################################
scale_percent = 30                                            #
width = int(rgb.shape[1] * scale_percent / 100)               #
height = int(rgb.shape[0] * scale_percent / 100)              #
dim = (width, height)                                         #
resized = cv2.resize(rgb, dim, interpolation = cv2.INTER_AREA)#
 ##############################################################

#image processing

#Grayscale
gray = cv2.cvtColor(resized,cv2.COLOR_BGR2GRAY)

#Gauss
blurred = cv2.GaussianBlur(gray,(7,7),0)
#_,thresh = cv2.threshold(blurred,0,255,cv2.THRESH_BINARY + cv2.THRESH_OTSU)


#Edge detection
canny = cv2.Canny(blurred,10,150,0)

#Dilation
#eroded = cv2.erode(thresh, (1,1), iterations = 1)
dilated = cv2.dilate(canny, (1,1), iterations = 1)

 
#Finding contours
#cnts = cv2.findContours(dilated.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#(cnts, hierarchy) = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
#cnts = cnts[0] if len(cnts) == 2 else cnts[1]


#Print and draw found contour areas with greater area than...
#min_area = 1000

# for cnt in cnts:
#     area = cv2.contourArea(cnt)
#     if area > min_area:
#         print("Contour areas",area)
#         cv2.drawContours(resized,[cnt],0,(0,255,0),2)
#         #print('Parcles in image: ', len(cnts))


#Drawing Contours
#cv2.drawContours(resized,cnts, -1, (0,255,0),2)


#cv2.imshow("Resized image", resized)
#cv2.imshow("grayscale", gray)
#cv2.imshow("Gauss", blurred)
#cv2.imshow("Edge", canny)
#cv2.imshow("Dilated", dilated)
#cv2.imshow("Eroded", eroded)
#cv2.imshow("Canny", canny)
#cv2.imshow("Contours", resized)
#cv2.waitKey(0)
#cv2.destroyAllWindows()    


#Finding contours
#cnts = cv2.findContours(cdstP, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
(cnts, hierarchy) = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
#cnts = cnts[0] if len(cnts) == 2 else cnts[1]


#Print and draw found contour areas with greater area than...
min_area = 10000

for cnt in cnts:
    area = cv2.contourArea(cnt)
    if area > min_area:
        print("Contour areas",area)
        cv2.drawContours(resized,[cnt],0,(0,255,0),1)
        epsilon = 0.1*cv2.arcLength(cnt,True)
        approxrect = cv2.approxPolyDP(cnt,epsilon,True)
        approxrect = cv2.minAreaRect(cnt)
        box = cv2.boxPoints(approxrect)
        box = np.int0(box)
        cv2.drawContours(resized,[box],0,(0,0,255),2)
        #print('Parcles in image: ', len(cnts))




#Drawing Contours
#cv2.drawContours(resized,cnts, -1, (0,255,0),1)
    
#cv2.imshow("Source", resized)
#cv2.imshow("Detected Lines (in red) - Standard Hough Line Transform", cdst)
cv2.imshow("Detected Lines (in red) - Probabilistic Line Transform", dilated)
cv2.imshow("Contours", resized)
    
cv2.waitKey()

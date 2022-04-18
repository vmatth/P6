from cv2 import imshow, waitKey
import numpy as np
import cv2
from matplotlib import pyplot as plt


img = cv2.imread('/home/yaro/P6/src/packing_camera/scripts/test2.jpg')

scale_percent = 20 # percent of original size
width = int(img.shape[1] * scale_percent / 100)
height = int(img.shape[0] * scale_percent / 100)
dim = (width, height)
  
#resize image
resized_img = cv2.resize(img, dim, interpolation = cv2.INTER_AREA)

gray = cv2.cvtColor(resized_img, cv2.COLOR_BGR2GRAY)


blur = cv2.GaussianBlur(gray,(5,5),0)

canny = cv2.Canny(blur,5,80)

kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(3,3))
dilated = cv2.dilate(canny, kernel)

ret, thresh = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
#noise removal

closing = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)
opening = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)


#sure background area
sure_bg = cv2.dilate(closing, kernel, iterations=3)

#finding sure foreground area
dist_transform = cv2.distanceTransform(closing, cv2.DIST_L2,5)
_, sure_fg = cv2.threshold(dist_transform,0.7*dist_transform.max(),255,0)

#findfing unknown region
#sure_fg = np.uint8(sure_fg)
#unknown = cv2.subtract(sure_bg,sure_fg)

contours,hierarchy = cv2.findContours(canny, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

cv2.drawContours(resized_img,contours,-1,(0,0,255), thickness = -1)

cv2.imshow("Closed",closing)
cv2.imshow("Open",opening)
cv2.imshow("canny",canny)
cv2.imshow("dilated",dilated)
cv2.imshow("hello", resized_img)
cv2.waitKey(0)
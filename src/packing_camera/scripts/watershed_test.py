#!/usr/bin/env python
import imp
from matplotlib import image
from skimage.feature import peak_local_max
from skimage.segmentation import watershed
from scipy import ndimage
import numpy as np
import argparse
import imutils
import cv2

#Import image - Change to YOUR location: ~/P6/src/packing_camera/scripts/test2edit.jpg
img = cv2.imread('/home/vinh/P6/src/packing_camera/scripts/test2edit.jpg') # Resize image if too big below

# # Resized image
# scale_percent = 20 # percent of original size
# width = int(img.shape[1] * scale_percent / 100)
# height = int(img.shape[0] * scale_percent / 100)
# dim = (width, height)
  
# resized_img = cv2.resize(img, dim, interpolation = cv2.INTER_AREA)

# Apply blur (pyramid mean shift filtering)
shifted = cv2.pyrMeanShiftFiltering(img, 21, 51) #Change img to resized_img if needed

# Grayscale and apply Otsu's thresholding
gray = cv2.cvtColor(shifted, cv2.COLOR_BGR2GRAY)

ret, thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
cv2.imshow("Thresh1", thresh)

distance = ndimage.distance_transform_edt(thresh)
localMax = peak_local_max(distance, indices=False, min_distance=20, labels=thresh)

markers = ndimage.label(localMax, structure=np.ones((3, 3)))[0]
labels = watershed(-distance, markers, mask=thresh)

contourImg = img.copy() #Change img to resized_img if needed

for label in np.unique(labels):
    if label == 0:
        continue

    mask = np.zeros(gray.shape, dtype="uint8")
    mask[labels == label] = 255

    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    c = max(cnts, key=cv2.contourArea)

    x,y,w,h = cv2.boundingRect(c)
    cv2.rectangle(contourImg,(x,y),(x+w,y+h),(0,255,0),2)
    cv2.putText(contourImg, "Parcel: {}".format(label), (int(x) - 10, int(y)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

#cv2.imshow("Raw image", img)
#cv2.imshow("Raw image", resized_img)
cv2.imshow("Blur", shifted)
cv2.imshow("Gray image", gray)
cv2.imshow("Threshold", thresh)
cv2.imshow("Contour", contourImg)
cv2.waitKey(0)
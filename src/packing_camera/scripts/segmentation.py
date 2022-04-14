#!/usr/bin/env python
import imp
from turtle import distance
from types import BuiltinFunctionType
from cv2 import imread, threshold
import numpy as np
import imageio
from skimage import morphology
from skimage.segmentation import watershed
from skimage.segmentation import random_walker
from skimage.feature import peak_local_max
from scipy import ndimage
import cv2
import tkinter as TK
#Get rgb & depth images for this frame
rgb_image = cv2.imread('test.jpg')

          
#Grayscale image
grayscale_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2GRAY)

#Blur
blur_image = cv2.medianBlur(grayscale_image, 5)
#Threshold
_,thresh_image = cv2.threshold(blur_image,0,255,cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU)

#computing the distance transfrom
#distance_t = ndimage.distance_transform_edt(thresh_image)
#computing local maxima from distance transform
#local_maxim = peak_local_max(distance_t, indices = False, footprint = np.ones((9,9)), labels = rgb_image)
#computes connected components, and label them with different values
#seeds = morphology.label(local_maxim)

#compute watershed segmentation
#segment = watershed(-distance_t, seeds, mask=rgb_image)


#Show images

#cv2.imshow("Segmetns", segment)
#cv2.imshow("Distance", distance_t)
cv2.imshow("Raw Image", thresh_image)
cv2.waitKey(0)
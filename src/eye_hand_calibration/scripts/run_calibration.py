#!/usr/bin/env python
import pickle
import numpy as np
import chessboard
import park_martin
import yaml
np.set_printoptions(linewidth=300, suppress=True)
from scipy.linalg import expm, inv
from numpy import dot, eye
import sys
import glob
from PIL import Image
import cv2
import io 
import os


#Load pictures
img_list = []
counter = 0
for filename in glob.glob('image_list.dump/*'):
    #Load image in opencv2
    filename = "image_list.dump/image" + str(counter) + ".jpg"
    print(filename)
    img = cv2.imread(filename)
    counter = counter + 1
    #convert her
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    img_list.append(gray)

print("Loaded " + str(len(img_list)) + " pictures")

rob_pose_list = []
counter = 0
for filename in glob.glob('pose_list.dump/*'):
    filename = "pose_list.dump/pose_list" + str(counter) + ".txt"
    print(filename)
    matrix = np.loadtxt(filename, usecols=range(4))
    counter = counter + 1
    rob_pose_list.append(matrix)
    
print("rob_pose_list ", rob_pose_list)


# for path in glob.glob(os.path.join('pose_list.dump', "pose_list*.txt")):
#     with io.open(path, mode="r", encoding="utf-8") as fd:
#         content = fd.read()

corner_list = []
obj_pose_list = []

#Calibrate camera using images
camera_matrix, dist_coeffs = chessboard.calibrate_lens(img_list)

def hat(v):
    return [[   0, -v[2],  v[1]],
            [v[2],     0, -v[0]],
            [-v[1],  v[0],    0]]

def tf_mat(r, t):
    res = eye(4)
    res[0:3, 0:3] = expm(hat(r))
    res[0:3, -1] = t
    return res

#Camera to checkerboard
for i, img in enumerate(img_list):
    found, corners = chessboard.find_corners(np.asarray(img))
    corner_list.append(corners)
    if not found:
        raise Exception("Failed to find corners in img # %d" % i)
    rvec, tvec = chessboard.get_object_pose(chessboard.pattern_points, corners, camera_matrix, dist_coeffs)
    object_pose = tf_mat(rvec, tvec)
    obj_pose_list.append(object_pose)


A, B = [], []
for i in range(1,len(img_list)):
    p = rob_pose_list[i-1], obj_pose_list[i-1]
    n = rob_pose_list[i], obj_pose_list[i]
    A.append(dot(inv(p[0]), n[0]))
    B.append(dot(inv(p[1]), n[1]))


# Transformation to chessboard in robot gripper
X = eye(4)
Rx, tx = park_martin.calibrate(A, B)
X[0:3, 0:3] = Rx
X[0:3, -1] = tx

print("X: ")
print(X)

print("For validation. Printing transformations from the robot base to the camera")
print("All the transformations should be quite similar")

for i in range(len(img_list)):
    rob = rob_pose_list[i]
    obj = obj_pose_list[i]
    tmp = dot(rob, dot(X, inv(obj)))
    print(tmp)

# Here I just pick one, but maybe some average can be used instead
rob = rob_pose_list[0]
obj = obj_pose_list[0]
cam_pose = dot(dot(rob, X), inv(obj))

cam = {'rotation' : cam_pose[0:3, 0:3].tolist(),
       'translation' : cam_pose[0:3, -1].tolist(),
       'camera_matrix' : camera_matrix.tolist(),
       'dist_coeffs' : dist_coeffs.tolist()}

fp = open('camera.yaml', 'w')
fp.write(yaml.dump(cam))
fp.close()

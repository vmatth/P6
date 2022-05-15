#!/usr/bin/env python
import pickle
from socketserver import BaseRequestHandler
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
import rospy
from scipy.spatial.transform import Rotation as R




#####################################
#Load pictures
#####################################
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

#####################################
#Load robot poses
#####################################
rob_pose_list = []
R_gripper2base_list = []
t_gripper2base_list = []
H_base2gripper_list = []
H_target2cam_list = []
counter = 0
for filename in glob.glob('pose_list.dump/*'):
    filename = "pose_list.dump/pose_list" + str(counter) + ".txt"
    print(filename)
    #Load transformation matrix base2gripper
    matrix = np.loadtxt(filename, usecols=range(4))
    H_base2gripper_list.append(matrix)
    matrix = inv(matrix)
    #Get Rotation matrix
    R_gripper2base = matrix[0:3, 0:3]
    #Get translation vector
    t_gripper2base = matrix[0:3, 3:4]
    #Append to list
    R_gripper2base_list.append(R_gripper2base)
    t_gripper2base_list.append(t_gripper2base)
    
    rob_pose_list.append(matrix)
    counter = counter + 1

#print("R Gripper 2 Base List", R_gripper2base_list)   
#print("T Gripper 2 Base List", t_gripper2base_list)

corner_list = []
obj_pose_list = []

#Calibrate camera using images
camera_matrix, dist_coeffs = chessboard.calibrate_lens(img_list)
print("-----------------------")
print("camera_matrix ", camera_matrix)
print("dist_coeffs ", dist_coeffs)
print("-----------------------")

# def hat(v):
#     return [[   0, -v[2],  v[1]],
#             [v[2],     0, -v[0]],
#             [-v[1],  v[0],    0]]

# def tf_mat(r, t):
#     res = eye(4)
#     res[0:3, 0:3] = expm(hat(r))
#     res[0:3, -1] = t
#     return res

###################################
#Camera to checkerboard
###################################
R_target2cam_list = []
t_target2cam_list = []
for i, img in enumerate(img_list):
    #print("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%")
    found, corners = chessboard.find_corners(np.asarray(img))
    corner_list.append(corners)
    if not found:
        print("Not found in", i)
        raise Exception("Failed to find corners in img # %d" % i)
    rvec, tvec = chessboard.get_object_pose(chessboard.pattern_points, corners, camera_matrix, dist_coeffs)
    ###################################
    #rvec is a vector containing roll, pitch and yaw
    #tvec is a vector containing x,y,z
    #Vectors cannot be inversed, therefore we combine the two vectors into a 4x4 transformation matrix, and thereafter inverse it.
    ###################################
    #print("RVEC", rvec)
    #tom3kryds3matrix = None

    H_cam2target = chessboard.matrix_from_rtvec(rvec, tvec)
    #R_cam2target = cv2.Rodrigues(rvec)
    # M = np.eye(4)
    #print("H_cam2target: ", H_cam2target)
    # M[0:3, 0:3] = R
    # M[0:3, 3] = tvec.squeeze() # 1-D vector, row vector, column vector, whatever
    # print("M2: ", M)
    # print("cam2target: ", R_cam2target)

    # tvec = np.array([[tvec[0], tvec[1], tvec[2]]])
    # tvec = tvec.T 
    # print("TVEC", tvec)
    # #Combine to a matrix
    # r = R.from_rotvec(rvec).as_dcm()
    # print("R matrix", r)
    # H_cam2target = np.hstack((r, tvec))
    # print("H_cam2target: ", H_cam2target)
    # scale_vector = [0,0,0,1]
    # H_cam2target = np.vstack((H_cam2target, scale_vector))
    H_target2cam = inv(H_cam2target)
    H_target2cam_list.append(H_target2cam)
    #print("inverted", H_target2cam)

    #Get Rotation matrix
    R_target2cam = matrix[0:3, 0:3]
    #Get translation vector
    t_target2cam = matrix[0:3, 3:4]
    #Append to list
    R_target2cam_list.append(R_target2cam)
    t_target2cam_list.append(t_target2cam)

#print("Rgripper2base list", R_gripper2base_list)

#open cv# 
for i in range(len(R_gripper2base_list)):
    print("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%")
    print("Calibrating Hand Eye")
    print("Input")
    print("R_gripper2base ", R_gripper2base_list[i])
    print("t_gripper2base ", t_gripper2base_list[i])
    print("R_target2cam " , R_target2cam_list[i])
    print("t_target2cam ", t_target2cam_list[i])

    #Output is cam2gripper
    R, t = cv2.calibrateHandEye(R_gripper2base_list, t_gripper2base_list, R_target2cam_list, t_target2cam_list, method=cv2.CALIB_HAND_EYE_TSAI)
    print("OUTPUT")
    print("R", R)
    print("t", t)


    #Calculate base2cam
    print("//////////////")
    print("Base2Cam: ", )
    print("//////////////")

    #base2gripper
    H_base2gripper_list[i]

    #target2cam
    H_target2cam_list[i]

    H_base2cam = np.matmul(H_base2gripper_list[i], H_target2cam_list[i])
    print("H_base2cam: ", H_base2cam)





# A, B = [], []
# for i in range(1,len(img_list)):
#     p = rob_pose_list[i-1], obj_pose_list[i-1]
#     n = rob_pose_list[i], obj_pose_list[i]
#     A.append(dot(inv(p[0]), n[0]))
#     B.append(dot(inv(p[1]), n[1]))


# # Transformation to chessboard in robot gripper
# X = eye(4)
# print("XXXXXXX", X)
# Rx, tx = park_martin.calibrate(A, B)
# X[0:3, 0:3] = Rx
# X[0:3, -1] = tx

# print("X: ")
# print(X)

# print("For validation. Printing transformations from the robot base to the camera")
# print("All the transformations should be quite similar")

# for i in range(len(img_list)):
#     rob = rob_pose_list[i]
#     obj = obj_pose_list[i]
#     tmp = dot(rob, dot(X, inv(obj)))
#     print(tmp)

# # Here I just pick one, but maybe some average can be used instead
# rob = rob_pose_list[0]
# obj = obj_pose_list[0]
# cam_pose = dot(dot(rob, X), inv(obj))

# cam = {'rotation' : cam_pose[0:3, 0:3].tolist(),
#        'translation' : cam_pose[0:3, -1].tolist(),
#        'camera_matrix' : camera_matrix.tolist(),
#        'dist_coeffs' : dist_coeffs.tolist()}

# fp = open('camera.yaml', 'w')
# fp.write(yaml.dump(cam))
# fp.close()

#!/usr/bin/env python
import numpy as np
import cv2

pattern_size = (7,5)
square_size = 0.03

pattern_points = np.zeros( (np.prod(pattern_size), 3), np.float32 )
pattern_points[:,:2] = np.indices(pattern_size).T.reshape(-1, 2)
pattern_points *= square_size

def find_corners(image):
    found, corners = cv2.findChessboardCorners(image, pattern_size)
    term = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT, 30, 0.1)
    if found:
        cv2.cornerSubPix(image, corners, (5, 5), (-1, -1), term)
    return found, corners

def draw_corners(image, corners):
    color_image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    cv2.drawChessboardCorners(color_image, pattern_size, corners, True)
    return color_image

def get_object_pose(object_points, image_points, camera_matrix, dist_coeffs):
    ret, rvec, tvec = cv2.solvePnP(object_points, image_points, camera_matrix, dist_coeffs)
    return rvec.flatten(), tvec.flatten()

def matrix_from_rtvec(rvec, tvec):
    (R, jac) = cv2.Rodrigues(rvec) # ignore the jacobian
    M = np.eye(4)
    M[0:3, 0:3] = R
    M[0:3, 3] = tvec.squeeze() # 1-D vector, row vector, column vector, whatever
    return M



def calibrate_lens(image_list):
    print("Calibrating lens")
    #print("list: ", image_list)
    img_points, obj_points = [], []
    h,w = 0, 0
    for img in image_list:
        #Process image
        h, w = img.shape[:2]
        found,corners = find_corners(np.asarray(img))
        if not found:
            raise Exception("chessboard calibrate_lens Failed to find corners in img")
        img_points.append(corners.reshape(-1, 2))
        obj_points.append(pattern_points)
    camera_matrix = np.zeros((3,3))
    dist_coeffs = np.zeros(5)
#    rms, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, (w,h))
    cv2.calibrateCamera(obj_points, img_points, (w,h), camera_matrix, dist_coeffs)
    print("calibration works")
    return camera_matrix, dist_coeffs

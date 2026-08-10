#!/usr/bin/env python3

import cv2
import numpy as np
import glob
import os
import yaml


# ==========================
# 标定参数
# ==========================

CHECKERBOARD = (11, 8)   # 内角点
SQUARE_SIZE = 0.02       # 米


IMAGE_DIR = "calib_data"

LEFT_DIR = IMAGE_DIR + "/left"
RIGHT_DIR = IMAGE_DIR + "/right"


OUTPUT_DIR = "calib_result"

os.makedirs(
    OUTPUT_DIR,
    exist_ok=True
)


# ==========================
# 世界坐标点
# ==========================

objp = np.zeros(
    (CHECKERBOARD[0]*CHECKERBOARD[1],3),
    np.float32
)

objp[:,:2] = np.mgrid[
    0:CHECKERBOARD[0],
    0:CHECKERBOARD[1]
].T.reshape(-1,2)

objp *= SQUARE_SIZE


objpoints = []
imgpoints_left = []
imgpoints_right = []


left_images = sorted(
    glob.glob(LEFT_DIR+"/*.png")
)

right_images = sorted(
    glob.glob(RIGHT_DIR+"/*.png")
)


print(
    "Images:",
    len(left_images),
    len(right_images)
)


img_size = None


for left_file,right_file in zip(
    left_images,
    right_images
):

    left = cv2.imread(
        left_file,
        cv2.IMREAD_GRAYSCALE
    )

    right = cv2.imread(
        right_file,
        cv2.IMREAD_GRAYSCALE
    )


    if img_size is None:
        img_size = (
            left.shape[1],
            left.shape[0]
        )

    ret_l, corners_l = cv2.findChessboardCornersSB(
        left,
        CHECKERBOARD
    )

    ret_r, corners_r = cv2.findChessboardCornersSB(
        right,
        CHECKERBOARD
    )

    if ret_l and ret_r:
        print(left_file)

        objpoints.append(objp.copy())

        imgpoints_left.append(corners_l)
        imgpoints_right.append(corners_r)


print(
    "Valid pairs:",
    len(objpoints)
)


# ==========================
# 单目标定
# ==========================

ret_l,K1,D1,R1,P1 = None,None,None,None,None

ret_l,K1,D1,rvecs_l,tvecs_l = cv2.calibrateCamera(
    objpoints,
    imgpoints_left,
    img_size,
    None,
    None
)

print("Left camera RMS:", ret_l)


print("\nLeft image reprojection errors:")

for i in range(len(objpoints)):

    imgpoints2, _ = cv2.projectPoints(
        objpoints[i],
        rvecs_l[i],
        tvecs_l[i],
        K1,
        D1
    )

    error = cv2.norm(
        imgpoints_left[i],
        imgpoints2,
        cv2.NORM_L2
    ) / len(imgpoints2)

    print(
        i,
        error
    )



ret_r,K2,D2,rvecs_r,tvecs_r = cv2.calibrateCamera(
    objpoints,
    imgpoints_right,
    img_size,
    None,
    None
)

print("Right camera RMS:", ret_r)


print("\nRight image reprojection errors:")

for i in range(len(objpoints)):

    imgpoints2, _ = cv2.projectPoints(
        objpoints[i],
        rvecs_r[i],
        tvecs_r[i],
        K2,
        D2
    )

    error = cv2.norm(
        imgpoints_right[i],
        imgpoints2,
        cv2.NORM_L2
    ) / len(imgpoints2)

    print(
        i,
        error
    )

# ==========================
# 双目标定
# ==========================

flags = 0


stereo_result = cv2.stereoCalibrate(
    objpoints,
    imgpoints_left,
    imgpoints_right,
    K1,
    D1,
    K2,
    D2,
    img_size,
    criteria=(
        cv2.TERM_CRITERIA_EPS +
        cv2.TERM_CRITERIA_MAX_ITER,
        100,
        1e-5
    ),
    flags=flags
)


ret = stereo_result[0]
K1 = stereo_result[1]
D1 = stereo_result[2]
K2 = stereo_result[3]
D2 = stereo_result[4]
R  = stereo_result[5]
T  = stereo_result[6]
E  = stereo_result[7]
F  = stereo_result[8]


print("Stereo RMS:", ret)

print("Translation:")
print(T)


# ==========================
# stereoRectify
# ==========================

R1,R2,P1,P2,Q,_,_=cv2.stereoRectify(
    K1,
    D1,
    K2,
    D2,
    img_size,
    R,
    T,
    alpha=0
)

# ROS stereo convention:
# right camera Tx should be negative
print("Before fix P2:")
print(P2)

if P2[0,3] > 0:
    P2[0,3] = -P2[0,3]

print("After fix P2:")
print(P2)

def save_yaml(
    filename,
    K,
    D,
    R,
    P,
    camera_name
):

    data={

        "image_width":img_size[0],
        "image_height":img_size[1],
        "camera_name": camera_name,

        "distortion_model": "plumb_bob",

        "camera_matrix":{
            "rows":3,
            "cols":3,
            "data":K.flatten().tolist()
        },

        "distortion_coefficients":{
            "rows":1,
            "cols":5,
            "data":D.flatten().tolist()
        },

        "rectification_matrix":{
            "rows":3,
            "cols":3,
            "data":R.flatten().tolist()
        },

        "projection_matrix":{
            "rows":3,
            "cols":4,
            "data":P.flatten().tolist()
        }
    }


    with open(filename,"w") as f:
        yaml.dump(
            data,
            f
        )


save_yaml(
    OUTPUT_DIR+"/left_camera.yaml",
    K1,D1,R1,P1,
    "left_camera"
)


save_yaml(
    OUTPUT_DIR+"/right_camera.yaml",
    K2,D2,R2,P2,
    "right_camera"
)


print("Calibration finished")

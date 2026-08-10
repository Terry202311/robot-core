#!/usr/bin/env python3

import cv2
import numpy as np
import glob
import os
import yaml


# ============================================================
# ORP Stereo Calibration Tool V2
# ============================================================

CHECKERBOARD = (11, 8)
SQUARE_SIZE = 0.02  # meter

IMAGE_DIR = "calib_data"
LEFT_DIR = os.path.join(IMAGE_DIR, "left")
RIGHT_DIR = os.path.join(IMAGE_DIR, "right")

OUTPUT_DIR = "calib_result"
PREVIEW_DIR = os.path.join(OUTPUT_DIR, "preview")

RUNTIME_SIZE = (640, 400)

os.makedirs(OUTPUT_DIR, exist_ok=True)
os.makedirs(PREVIEW_DIR, exist_ok=True)


def save_yaml(
    filename,
    K,
    D,
    R,
    P,
    camera_name,
    image_size
):
    data = {
        "image_width": int(image_size[0]),
        "image_height": int(image_size[1]),
        "camera_name": camera_name,

        "distortion_model": "plumb_bob",

        "camera_matrix": {
            "rows": 3,
            "cols": 3,
            "data": K.flatten().tolist()
        },

        "distortion_coefficients": {
            "rows": 1,
            "cols": len(D.flatten()),
            "data": D.flatten().tolist()
        },

        "rectification_matrix": {
            "rows": 3,
            "cols": 3,
            "data": R.flatten().tolist()
        },

        "projection_matrix": {
            "rows": 3,
            "cols": 4,
            "data": P.flatten().tolist()
        }
    }

    with open(filename, "w") as f:
        yaml.safe_dump(
            data,
            f,
            sort_keys=False
        )


def scale_camera_matrix(K, sx, sy):
    K_scaled = K.copy()

    K_scaled[0, 0] *= sx
    K_scaled[0, 2] *= sx

    K_scaled[1, 1] *= sy
    K_scaled[1, 2] *= sy

    return K_scaled


def reprojection_error(
    objpoints,
    imgpoints,
    rvecs,
    tvecs,
    K,
    D
):
    errors = []

    for i in range(len(objpoints)):
        projected, _ = cv2.projectPoints(
            objpoints[i],
            rvecs[i],
            tvecs[i],
            K,
            D
        )

        error = cv2.norm(
            imgpoints[i],
            projected,
            cv2.NORM_L2
        ) / len(projected)

        errors.append(error)

    return errors


# ============================================================
# Calibration board coordinates
# ============================================================

objp = np.zeros(
    (CHECKERBOARD[0] * CHECKERBOARD[1], 3),
    np.float32
)

objp[:, :2] = np.mgrid[
    0:CHECKERBOARD[0],
    0:CHECKERBOARD[1]
].T.reshape(-1, 2)

objp *= SQUARE_SIZE


objpoints = []
imgpoints_left = []
imgpoints_right = []


left_images = sorted(
    glob.glob(os.path.join(LEFT_DIR, "*.png"))
)

right_images = sorted(
    glob.glob(os.path.join(RIGHT_DIR, "*.png"))
)


print("Images:", len(left_images), len(right_images))

if len(left_images) != len(right_images):
    print("WARNING: left/right image counts are different!")


img_size = None
valid_files = []


for left_file, right_file in zip(
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

    if left is None or right is None:
        print("Failed to read:", left_file, right_file)
        continue

    if left.shape != right.shape:
        print("Image size mismatch:", left_file, right_file)
        continue

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

        valid_files.append(
            (left_file, right_file)
        )


print("Valid pairs:", len(objpoints))

if len(objpoints) < 10:
    raise RuntimeError(
        f"Not enough valid stereo calibration pairs: {len(objpoints)}"
    )


# ============================================================
# Monocular calibration
# ============================================================

ret_left, K_left, D_left, rvecs_left, tvecs_left = \
    cv2.calibrateCamera(
        objpoints,
        imgpoints_left,
        img_size,
        None,
        None
    )

ret_right, K_right, D_right, rvecs_right, tvecs_right = \
    cv2.calibrateCamera(
        objpoints,
        imgpoints_right,
        img_size,
        None,
        None
    )


print("\nLeft camera RMS:", ret_left)
print("Right camera RMS:", ret_right)


left_errors = reprojection_error(
    objpoints,
    imgpoints_left,
    rvecs_left,
    tvecs_left,
    K_left,
    D_left
)

right_errors = reprojection_error(
    objpoints,
    imgpoints_right,
    rvecs_right,
    tvecs_right,
    K_right,
    D_right
)

print(
    "Left mean reprojection error:",
    np.mean(left_errors)
)

print(
    "Right mean reprojection error:",
    np.mean(right_errors)
)


# ============================================================
# Stereo calibration
#
# IMPORTANT:
# Current calibration data appears to have physical left/right
# assignment reversed. Therefore:
#
# camera1 = original right
# camera2 = original left
#
# We test this direction explicitly.
# ============================================================

stereo_result = cv2.stereoCalibrate(
    objpoints,

    imgpoints_right,
    imgpoints_left,

    K_right,
    D_right,

    K_left,
    D_left,

    img_size,

    criteria=(
        cv2.TERM_CRITERIA_EPS +
        cv2.TERM_CRITERIA_MAX_ITER,
        100,
        1e-5
    ),

    flags=cv2.CALIB_FIX_INTRINSIC
)


stereo_rms = stereo_result[0]

K_cam1 = stereo_result[1]
D_cam1 = stereo_result[2]

K_cam2 = stereo_result[3]
D_cam2 = stereo_result[4]

R = stereo_result[5]
T = stereo_result[6]
E = stereo_result[7]
F = stereo_result[8]


print("\nStereo RMS:", stereo_rms)

print("Translation:")
print(T)


# ============================================================
# Runtime resolution
# ============================================================

calib_size = img_size
runtime_size = RUNTIME_SIZE

sx = runtime_size[0] / calib_size[0]
sy = runtime_size[1] / calib_size[1]

print("\nCalibration size:", calib_size)
print("Runtime size:", runtime_size)
print("Scale:", sx, sy)


K_cam1_runtime = scale_camera_matrix(
    K_cam1,
    sx,
    sy
)

K_cam2_runtime = scale_camera_matrix(
    K_cam2,
    sx,
    sy
)


# ============================================================
# Stereo rectification
# ============================================================

R1, R2, P1, P2, Q, roi1, roi2 = \
    cv2.stereoRectify(
        K_cam1_runtime,
        D_cam1,

        K_cam2_runtime,
        D_cam2,

        runtime_size,

        R,
        T,

        flags=cv2.CALIB_ZERO_DISPARITY,
        alpha=0
    )


baseline = -P2[0, 3] / P2[0, 0]


print("\n==============================")
print("Stereo Geometry")
print("==============================")

print("T:")
print(T)

print("P1:")
print(P1)

print("P2:")
print(P2)

print("Baseline:", baseline, "m")

print("ROI1:", roi1)
print("ROI2:", roi2)


if baseline <= 0:
    raise RuntimeError(
        "Stereo baseline <= 0. "
        "LEFT/RIGHT camera assignment is still wrong."
    )


# ============================================================
# IMPORTANT:
#
# camera1 is ORIGINAL RIGHT
# camera2 is ORIGINAL LEFT
#
# Therefore when saving final ROS camera names:
#
# ROS left  = camera1
# ROS right = camera2
#
# Only use these YAMLs after the runtime driver is made consistent
# with this physical assignment.
# ============================================================

left_yaml = os.path.join(
    OUTPUT_DIR,
    "left_camera.yaml"
)

right_yaml = os.path.join(
    OUTPUT_DIR,
    "right_camera.yaml"
)


save_yaml(
    left_yaml,

    K_cam1_runtime,
    D_cam1,

    R1,
    P1,

    "left_camera",
    runtime_size
)


save_yaml(
    right_yaml,

    K_cam2_runtime,
    D_cam2,

    R2,
    P2,

    "right_camera",
    runtime_size
)


# ============================================================
# Rectification preview
# ============================================================

preview_left_file, preview_right_file = valid_files[0]

preview_left_original = cv2.imread(
    preview_right_file,
    cv2.IMREAD_GRAYSCALE
)

preview_right_original = cv2.imread(
    preview_left_file,
    cv2.IMREAD_GRAYSCALE
)


preview_left_original = cv2.resize(
    preview_left_original,
    runtime_size
)

preview_right_original = cv2.resize(
    preview_right_original,
    runtime_size
)


map1_l, map2_l = cv2.initUndistortRectifyMap(
    K_cam1_runtime,
    D_cam1,
    R1,
    P1,
    runtime_size,
    cv2.CV_32FC1
)

map1_r, map2_r = cv2.initUndistortRectifyMap(
    K_cam2_runtime,
    D_cam2,
    R2,
    P2,
    runtime_size,
    cv2.CV_32FC1
)


rect_left = cv2.remap(
    preview_left_original,
    map1_l,
    map2_l,
    cv2.INTER_LINEAR
)

rect_right = cv2.remap(
    preview_right_original,
    map1_r,
    map2_r,
    cv2.INTER_LINEAR
)


preview = np.hstack(
    (
        rect_left,
        rect_right
    )
)


for y in range(
    20,
    runtime_size[1],
    40
):
    cv2.line(
        preview,
        (0, y),
        (runtime_size[0] * 2, y),
        255,
        1
    )


preview_file = os.path.join(
    PREVIEW_DIR,
    "stereo_rectified_preview.png"
)

cv2.imwrite(
    preview_file,
    preview
)


# ============================================================
# Summary
# ============================================================

print("\n==============================")
print("Calibration Summary")
print("==============================")

print("Valid pairs:", len(objpoints))

print("Left RMS:", ret_left)
print("Right RMS:", ret_right)
print("Stereo RMS:", stereo_rms)

print("Baseline:", baseline, "m")

print("Left YAML:", left_yaml)
print("Right YAML:", right_yaml)

print("Preview:", preview_file)

print("\nCalibration finished.")
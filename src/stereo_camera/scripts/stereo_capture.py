#!/usr/bin/env python3

import cv2
import os
import time


WIDTH = 2560
HEIGHT = 800

SAVE_DIR = "calib_data"

LEFT_DIR = os.path.join(SAVE_DIR, "left")
RIGHT_DIR = os.path.join(SAVE_DIR, "right")


os.makedirs(LEFT_DIR, exist_ok=True)
os.makedirs(RIGHT_DIR, exist_ok=True)


cap = cv2.VideoCapture("/dev/video0", cv2.CAP_V4L2)

cap.set(cv2.CAP_PROP_FOURCC,
        cv2.VideoWriter_fourcc(*"MJPG"))

cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
cap.set(cv2.CAP_PROP_FPS, 60)


count = 0


print("Press SPACE save pair")
print("Press ESC exit")


while True:

    ret, frame = cap.read()

    if not ret:
        continue


    left = frame[:, :1280]
    right = frame[:,1280:]


    show = cv2.hconcat(
        [left,right]
    )

    cv2.imshow(
        "stereo",
        show
    )


    key=cv2.waitKey(1)


    if key==27:
        break


    if key==32:

        name=f"{count:04d}.png"

        cv2.imwrite(
            os.path.join(LEFT_DIR,name),
            left
        )

        cv2.imwrite(
            os.path.join(RIGHT_DIR,name),
            right
        )

        print("saved",name)

        count+=1


cap.release()
cv2.destroyAllWindows()

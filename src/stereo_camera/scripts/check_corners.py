import cv2


img=cv2.imread(
    "calib_data/left/0000.png"
)

gray=cv2.cvtColor(
    img,
    cv2.COLOR_BGR2GRAY
)


pattern=(11,8)


ret,corners=cv2.findChessboardCornersSB(
    gray,
    pattern
)


print("result:",ret)


if ret:

    cv2.drawChessboardCorners(
        img,
        pattern,
        corners,
        ret
    )


cv2.imwrite(
    "/tmp/corners.png",
    img
)

print("saved /tmp/corners.png")

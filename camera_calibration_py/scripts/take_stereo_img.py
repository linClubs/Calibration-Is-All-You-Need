# coding:utf-8
import cv2
import os
from config import ws_path, img_height, img_width

w = img_width
h = img_height

left_camera = cv2.VideoCapture(0)
left_camera.set(cv2.CAP_PROP_FRAME_WIDTH, w)
left_camera.set(cv2.CAP_PROP_FRAME_HEIGHT, h)

right_camera = cv2.VideoCapture(1)
right_camera.set(cv2.CAP_PROP_FRAME_WIDTH, w)
right_camera.set(cv2.CAP_PROP_FRAME_HEIGHT, h)


cv2.namedWindow("left")
cv2.namedWindow("right")
cv2.moveWindow("left", 10, 10)
cv2.moveWindow("right", 20+w, 10)
cv2.resizeWindow("left",w,h)
cv2.resizeWindow("right",w,h)
counter = 0

# 创建image路径
folder = os.path.join(ws_path, "image")
if not os.path.exists(folder):
    os.mkdir(folder)

# 创建image/left路径
left_dir = os.path.join(folder, "left")  # 照片存储路径
if not os.path.exists(left_dir):
    os.mkdir(left_dir)

right_dir = os.path.join(folder, "right")  # 照片存储路径
if not os.path.exists(right_dir):
    os.mkdir(right_dir)

# shot保存图像
def shot(pos, frame):
    global counter                                   # 全局变量，计数
    path = folder+"/" + pos+"/" +pos+ "_" + str(counter) + ".jpg"
    cv2.imwrite(path, frame)
    print("snapshot saved into: " + path)
    print("\n--Press s to save or q exit...\n")

print("\n--Press s to save or q exit...\n")
while True:
    ret1, left_frame = left_camera.read()
    ret2, right_frame = right_camera.read()
    left_frame = cv2.resize(left_frame,(w,h))
    right_frame = cv2.resize(right_frame,(w,h))
    cv2.imshow("left", left_frame)
    cv2.imshow("right", right_frame)

    # 手动保存
    key = cv2.waitKey(10)
    if key == ord("q"):                            # 安q停止
        break
    elif key == ord("s"):                           # 安s保存
        shot("left", left_frame)                     # 保存左相机
        shot("right", right_frame)                   # 保存右相机
        counter += 1                                # 计数

left_camera.release()
right_camera.release()
cv2.destroyWindow("left")
cv2.destroyWindow("right")
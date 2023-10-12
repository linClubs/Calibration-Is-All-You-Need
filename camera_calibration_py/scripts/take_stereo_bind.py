# coding:utf-8
import cv2
import os
from config import ws_path, img_height, img_width

w = img_width
h = img_height

left_camera = cv2.VideoCapture(0)
left_camera.set(cv2.CAP_PROP_FRAME_WIDTH, w * 2)  # 2倍w
left_camera.set(cv2.CAP_PROP_FRAME_HEIGHT, h)


cv2.namedWindow("view")
cv2.moveWindow("view", 30, 30)

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
    ret1, frame = left_camera.read()
    cv2.imshow("view", frame)
    
    left_img = frame[:, 0:int(w), :]
    right_img = frame[:, int(w):int(w + w), :]
    # 手动保存
    key = cv2.waitKey(10)
    if key == ord("q"):                            # 安q停止
        break
    elif key == ord("s"):                           # 安s保存
        shot("left", left_img)                     # 保存左相机
        shot("right", right_img)                   # 保存右相机
        counter += 1                                # 计数

left_camera.release()
cv2.destroyWindow("view")
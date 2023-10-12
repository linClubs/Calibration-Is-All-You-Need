# coding:utf-8
import cv2
from config import ws_path, img_height, img_width
import os

w = img_width
h = img_height
left_camera = cv2.VideoCapture(0)
left_camera.set(cv2.CAP_PROP_FRAME_WIDTH, w)
left_camera.set(cv2.CAP_PROP_FRAME_HEIGHT, h)



cv2.namedWindow("left")
cv2.moveWindow("left", 30, 0)
# cv2.resizeWindow("left",w+10,10)

counter = 0 # 计数

# 创建image路径
image_dir = os.path.join(ws_path, "image")
if not os.path.exists(image_dir):
    os.mkdir(image_dir)

# 创建image/left路径
folder = os.path.join(image_dir, "left")  # 照片存储路径
if not os.path.exists(folder):
    os.mkdir(folder)

# shot保存图像
def shot(pos, frame):
    global counter   # 申明全局变量，计数
    path = folder+"/" + pos + "_" + str(counter) + ".jpg"
    cv2.imwrite(path, frame)
    print("snapshot saved into: " + path)
    print("\n--Press s to save or q exit...\n")

print("\n--Press s to save or q exit...\n")
while (True):
    ret, left_frame = left_camera.read()
    if(ret == 0):
        break
    left_frame = cv2.resize(left_frame,(w,h))
    cv2.imshow("left", left_frame)
    
    # 手动保存
    key = cv2.waitKey(10)
    if key == ord("q"):                            # 安q停止
        break
    elif key == ord("s"):                           # 安s保存
        shot("left", left_frame)                     # 保存左相机
        counter += 1                                # 计数

left_camera.release()
cv2.destroyWindow("left")
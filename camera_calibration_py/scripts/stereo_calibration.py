# coding:utf-8
import numpy as np
import cv2
import os
from config import Checkerboard, Checkerboard_length, ws_path

#-------------------------------
# 设置棋盘格大小  Checkerboard参数
#  scale这里scale就是边长。边长最终影响平移向量的大小
#-------------------------------
# Checkerboard = (8, 6)  #棋盘格内角点数
# Checkerboard_length = 25  #棋盘格边长，单位mm
# ws_path = "/home/lin/share_dir/catkin_ws/src/camera_calibration"


scale = Checkerboard_length = 25  #棋盘格边长，单位mm
# 设置迭代终止条件
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)                 # 单目的，亚像素函数使用
criteria_stereo = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)           # 双目的。亚像素函数使用


# 设置 object points, 形式为 (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((Checkerboard[0] * Checkerboard[1], 3), np.float32)  # 我用的是6×7的棋盘格，可根据自己棋盘格自行修改相关参数
# objp[:, :2] = np.mgrid[0:Checkerboard[0], 0:Checkerboard[1]].T.reshape(-1, 2)

objp[:, :2] = np.mgrid[0:(Checkerboard[0]-1)*scale:complex(0,Checkerboard[0]),
              0:(Checkerboard[1]-1)*scale:complex(0,Checkerboard[1])].T.reshape(-1, 2)
# obj_points = [] # 存储3D点
# print(objp)

# 用arrays存储所有图片的object points 和 image points
objpoints = []  # 3d points in real world space
imgpointsR = []  # 2d points in image plane
imgpointsL = []

imageL_dir = os.path.join(ws_path, "image/left")
imageR_dir = os.path.join(ws_path, "image/right")

imageL_names = os.listdir(imageL_dir)

print("/n--It's %s pair of images..." %len(imageL_names))

if(len(imageL_names) < 1):
    print("\n--It is recommended to take more than 10 images\n\n--Please add image..\n" )
    exit()

for i, imageL_name in enumerate(imageL_names):
    imageL_path = os.path.join(imageL_dir, imageL_name)              # 左视图
    imageR_path = imageL_path.replace("/left/left", "/right/right") # 右视图

    ImaL = cv2.imread(imageL_path, 1)  
    ImaR = cv2.imread(imageR_path, 1)  

    
    grayL = cv2.cvtColor(ImaL, cv2.COLOR_BGR2GRAY)
    grayR = cv2.cvtColor(ImaR, cv2.COLOR_BGR2GRAY)
    retR, cornersR = cv2.findChessboardCorners(grayR, Checkerboard, None)  # 提取右图每一张图片的角点
    retL, cornersL = cv2.findChessboardCorners(grayL, Checkerboard, None)  # # 提取左图每一张图片的角点
    if (True == retR) & (True == retL):
        print("\n--detecting the %s image..."%i)
        objpoints.append(objp)
        cv2.cornerSubPix(grayL, cornersL, (11, 11), (-1, -1), criteria)  # 亚像素精确化，对粗提取的角点进行精确化
        cv2.cornerSubPix(grayR, cornersR, (11, 11), (-1, -1), criteria)  # 亚像素精确化，对粗提取的角点进行精确化
        imgpointsR.append(cornersR)
        imgpointsL.append(cornersL)
        
        # 画角点
        cv2.drawChessboardCorners(ImaL, Checkerboard, cornersL, retL)
        cv2.drawChessboardCorners(ImaR, Checkerboard, cornersR, retR)

        # 把数据压到栈中,一起显示左右检测结果
        Image_merge = np.hstack((ImaL, ImaR))
        # 各参数依次是:图片，添加的文字，左上角坐标，字体，字体大小，颜色，字体粗细
        cv2.putText(Image_merge,"Left_img", (5, 40), 0, 1, (0, 0, 255), 2)
        cv2.putText(Image_merge, "Right_img", (5+ImaL.shape[1], 40), 0, 1, (0, 0, 255), 2)

        cv2.imshow('input img', Image_merge)
        # print("请敲击任意键，检测下一帧角点...")
        cv2.waitKey(200)
    else:
        print("--/Failed to detect corner points...\n--Please check the image path or checkerboard size..")
cv2.destroyAllWindows()

# 相机的单双目标定、及校正
#   右侧相机单独标定
retR, mtxR, distR, rvecsR, tvecsR = cv2.calibrateCamera(objpoints, imgpointsR, grayR.shape[::-1], None, None)
# print("右相机内参:\n",mtxR, "右相机畸变:\n",distR)
# print("\n","-"*30,"\n")
#   获取新的相机矩阵后续传递给initUndistortRectifyMap，以用remap生成映射关系
hR, wR = grayR.shape[:2]
OmtxR, roiR = cv2.getOptimalNewCameraMatrix(mtxR, distR, (wR, hR), 1, (wR, hR))

#   左侧相机单独标定
'''
ret表示的是重投影误差；
mtx是相机的内参矩阵；dist表述的相机畸变参数；
rvecs表示标定棋盘格世界坐标系到相机坐标系的旋转参数：
rotation vectors，需要进行罗德里格斯转换；
tvecs表示translation vectors，主要是平移参数。
'''

retL, mtxL, distL, rvecsL, tvecsL = cv2.calibrateCamera(objpoints, imgpointsL, grayL.shape[::-1], None, None)
# print("左相机内参:\n",mtxL, "左相机畸变:\n",distL)
# print("\n","-"*30,"\n")
#   获取新的相机矩阵后续传递给initUndistortRectifyMap，以用remap生成映射关系
hL, wL = grayL.shape[:2]
OmtxL, roiL = cv2.getOptimalNewCameraMatrix(mtxL, distL, (wL, hL), 1, (wL, hL))

# 双目相机的标定
# 设置标志位为cv2.CALIB_FIX_INTRINSIC，这样就会固定输入的cameraMatrix和distCoeffs不变，只求解𝑅,𝑇,𝐸,𝐹
# flags = 1
# flags = cv2.CALIB_FIX_INTRINSIC
flags = cv2.CALIB_USE_INTRINSIC_GUESS
# 存储标定角点在世界坐标系中的位,左边亚像素点，右边亚像素点，左内参+畸变，右内参+畸变,图像大小，
retS, MLS, dLS, MRS, dRS, R, T, E, F = cv2.stereoCalibrate(objpoints, imgpointsL, imgpointsR,
                                                            mtxL, distL,
                                                           mtxR, distR,
                                                           grayR.shape[::-1],
                                                           criteria_stereo, flags)


# 利用stereoRectify()计算立体校正的映射矩阵
rectify_scale = 1  # 设置为0的话，对图片进行剪裁，设置为1则保留所有原图像像素
RL, RR, PL, PR, Q, roiL, roiR = cv2.stereoRectify(MLS, dLS, MRS, dRS, grayR.shape[::-1], R, T, rectify_scale, (0, 0))

# 利用initUndistortRectifyMap函数计算畸变矫正和立体校正的映射变换，实现极线对齐。
Left_Stereo_Map = cv2.initUndistortRectifyMap(MLS, dLS, RL, PL, grayR.shape[::-1], cv2.CV_16SC2)
Right_Stereo_Map = cv2.initUndistortRectifyMap(MRS, dRS, RR, PR, grayR.shape[::-1], cv2.CV_16SC2)

# 立体校正效果显示

rect_save_path = imageL_dir.replace("left", "stereo_rect")
if not os.path.exists(rect_save_path):
    os.mkdir(rect_save_path)

for i, imageL_name in enumerate(imageL_names):
    imageL_path = os.path.join(imageL_dir, imageL_name)              # 左视图
    imageR_path = imageL_path.replace("/left/left", "/right/right") # 右视图

    frameL = cv2.imread(imageL_path, -1)  
    frameR = cv2.imread(imageR_path, -1)  



    Left_rectified = cv2.remap(frameL, Left_Stereo_Map[0], Left_Stereo_Map[1], cv2.INTER_LANCZOS4, cv2.BORDER_CONSTANT,
                               0)  # 使用remap函数完成映射
    # im_L = Image.fromarray(Left_rectified)  # numpy 转 image类

    Right_rectified = cv2.remap(frameR, Right_Stereo_Map[0], Right_Stereo_Map[1], cv2.INTER_LANCZOS4,
                                cv2.BORDER_CONSTANT, 0)
    # im_R = Image.fromarray(Right_rectified)  # numpy 转 image 类

    # 创建一个能同时并排放下两张图片的区域，后把两张图片依次粘贴进去
    # 建立输出图像
    height = max(Left_rectified.shape[0], Right_rectified.shape[0])
    width = Left_rectified.shape[1] + Right_rectified.shape[1]

    output = np.zeros((height, width, 3 ), dtype=np.uint8)

    output[0:Left_rectified.shape[0], 0:Left_rectified.shape[1]] = Left_rectified
    output[0:Right_rectified.shape[0], Right_rectified.shape[1]:] = Right_rectified

    line_interval = 40  # 直线间隔:40
    for k in range(height // line_interval):
        cv2.line(output, (0, line_interval * (k + 1)), (2 * width, line_interval * (k + 1)), (0, 255, 0), thickness=2,
                 lineType=cv2.LINE_AA)
    cv2.imshow("Stereo_rectified", output)
    cv2.imwrite(rect_save_path + str(i) + ".jpg", output)
    print("It's %s rectified images..." %i)
    cv2.waitKey(200)
cv2.destroyAllWindows()


# 打印标定结果
print("----------result of calibration-----------")
print("\n--cameraMatrixL:\n",mtxL, "\n--distCoeffL:\n",distL)
print("\n--cameraMatrixR:\n",mtxR, "\n--distCoeffR:\n",distR)
print("\n--rotation of external parameters:\n",R,"\n--transform of external parameters::\n",T)
print("-"*30,"\n")




#重投影误差计算
# mean_error = 0
# for i in range(len(objpoints)):
#     imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecsL[i], tvecsL[i], mtxL, distL)
#     error = cv2.norm(imgpointsL[i], imgpoints2, cv2.NORM_L2)/len(imgpoints2)
#     mean_error += error
# print("\n Left camera error: ", mean_error/len(objpoints))


# 左重投影误差计算
total_error = 0

for i in range(len(objpoints)):
    imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecsL[i], tvecsL[i], mtxL, distL)
    # L2范数是所有元素(绝对值)的平方和再开方
    error = cv2.norm(imgpointsL[i], imgpoints2, cv2.NORM_L2) # 二范数
    # 范数是开方了,这里又平方,还原
    total_error += error*error  # 求和
    # i 表示i张图，每张图都有len(imgpoints2)角点个误差,总共len(objpoints)张图
total_errorL = np.sqrt( total_error /(len(objpoints)*len(imgpoints2)) )
print("Opencv projection error: {}".format(retL))
print("Left projection error: {}".format(total_errorL) )



# 右重投影误差计算
total_error = 0
for i in range(len(objpoints)):
    imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecsR[i], tvecsR[i], mtxR, distR)
    # L2范数是所有元素(绝对值)的平方和再开方
    error = cv2.norm(imgpointsR[i], imgpoints2, cv2.NORM_L2) # 二范数
    # 范数是开方了,这里又平方,还原
    total_error += error*error  # 求和
total_errorR = np.sqrt( total_error / (len(objpoints)*len(imgpoints2)) )

print("Opencv projection error: {}".format(retR))
print( "Right projection error: {}".format(total_errorR))

print("Stereo projection error: {}".format(retS))





# 保存结果
result = "cameraMatrixL:\n"+str(mtxL)+"\ndistCoeffL:\n"+str(distL)+ \
         "\ncameraMatrixR:\n"+str(mtxR)+"\ndistCoeffR:\n"+str(distR)+ \
         "\nR:\n"+str(R)+"\nT:\n"+str(T) + \
         "\n\nLeft Camera projectionerror:\n"+str(total_errorL) + \
         "\nRight Camera projectionerror:\n"+str(total_errorR) + \
         "\nStereo Camera projectionerror:\n"+str(retS)  

save_dir = os.path.join(ws_path, "results")
if not os.path.exists(save_dir):
    os.mkdir(save_dir)

save_path = os.path.join(save_dir, "stereo_calib.txt")
with open(save_path, 'w') as f:
    f.write(result)
f.close()
print("save result successfully...")



cd Thirdparty
tar -xf ceres-solver-1.14.0.tar.xz
cd ceres-solver-1.14.0
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j8
sudo make install   # root用户下不加sudo
cd ../..

# --------------------

tar -xf nlopt.tar.xz
cd nlopt
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j8
sudo make install  # root用户下不加sudo
cd ../../..

# --------------------

cd ../..
# 使用catkin_make编译
# 但是出现编译kalibr时aslam_cv_pythonConfig.Cmake找不到,直接用catkin build编译
# 一、--------------catkin_make---------------------

# # 1.-----------camera_calibration_ros---------------
# catkin_make -DCATKIN_WHITELIST_PACKAGES="camera_calibration;usb_cam"

# # 2.--------imu--------------------
# catkin_make -DCATKIN_WHITELIST_PACKAGES="code_utils"
# catkin_make -DCATKIN_WHITELIST_PACKAGES="imu_utils"

# # 3. ----------lidar_align--------------
# catkin_make -DCATKIN_WHITELIST_PACKAGES="lidar_align"

# # 4. -------------lidar2cam_calibration-------------
# catkin_make -DCATKIN_WHITELIST_PACKAGES="lidar2cam_calibration"

# # 5. -----------kalibr--------------------
# # 编译kalibr时aslam_cv_pythonConfig.Cmake找不到，
# # 或者直接用catkin build 编译先编译aslam_cv_python在kalibr

# catkin_make -DCATKIN_WHITELIST_PACKAGES="kalibr" -j8

#--------------------------------

# 二、 ---------catkin build --------------
# 编译卡死就加上-j4参数

catkin build camera_calibration usb_cam

catkin build code_utils
catkin build imu_utils

catkin build lidar_align
catkin build lidar2cam_calibration

# catkin build aslam_cv_python
catkin build kalibr
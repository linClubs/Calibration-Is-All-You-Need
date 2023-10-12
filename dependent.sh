
# 0 ---base_settings---
sudo apt-get update
sudo apt-get install -y git vim wget autoconf automake

# 1---camera_calibration-pkg-dependent---
sudo apt-get install -y python3-dev python3-pip python3-catkin-tools

# pip install opencv-python==4.7.0.72 numpy==1.23.0
# 国内源
pip install opencv-python==4.7.0.72 numpy==1.23.0 -i https://pypi.mirrors.ustc.edu.cn/simple/ 
# --------------------

# 2---kalibr-dependent(include ceres)---
sudo apt-get install -y python3-dev python3-pip python3-scipy python3-matplotlib ipython3 python3-wxgtk4.0 python3-tk python3-igraph libv4l-dev

# ceres-dependent
sudo apt-get install -y liblapack-dev libsuitesparse-dev libcxsparse3 libgflags-dev libgoogle-glog-dev libgtest-dev

cd Thirdparty
tar -xf ceres-solver-1.14.0.tar.xz
cd ceres-solver-1.14.0
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j8
sudo make install
cd ../..

# --------------------

# 3---imu_calibration-dependent---
sudo apt-get install -y libdw-dev

# 4---lidar_align-dependent---
tar -xf nlopt.tar.xz
cd nlopt
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j8
sudo make install
cd ../../..
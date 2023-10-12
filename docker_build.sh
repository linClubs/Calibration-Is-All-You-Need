
# 1. ---拉取calibration---
# cd ~
# git clone https://gitee.com/linClubs/lidar2cam_calibration.git

# 2. 制作镜像
sudo docker build -f dockerfile -t calibrate:v1.0 .

# 3. 启动容器
# --network host 
sudo docker run -it -v /tmp/.X11-unix:/tmp/.X11-unix --env="DISPLAY=$DISPLAY" -v $(pwd):/calibration_ws/src/calibration --name=test01 calibrate:v1.0 /bin/bash

# 4. 编译ceres与nlopt
# 进入docker环境后执行./build.sh

# 1 雷达与相机联合标定

## 1 标定方法

1. 基于目标的联合标定
2. 无目标的联合标定

## 2 基于目标的联合标定

1. 基于棋盘格
2. 二维码

## 3 基于ArUco的标定方法

![](2.png)


$$
P\_c = Tcl\ P\_lidar
$$

1.  根据标定板尺寸,得到在标定板坐标系下三维坐标p_board
2.  根据二维码信息,得到标定板的像素坐标p_uv
3.  根据相机内参、p_board,p_uv计算得到,标记点在相机坐标下的坐标p_cam
4.  根据点云直通滤波、分割平面、投影平面,得到标定板的点云平面,即雷达坐标p_lidar
5.  根据p_lidar，p_cam计算得到Tcl;
6.  优化Tcl。



##  4 代码docker部署

### 4.1 安装docker

1. 安装依赖
~~~python
sudo apt-get update
sudo apt-get --no-install-recommends install -y apt-transport-https ca-certificates curl gnupg-agent software-properties-common
~~~

2. 添加GPG密钥 

+ GPG密钥**官方秘钥**与**中科大密钥**只需要**运行一个**就行
+ 国内建议用**中科大密钥**

~~~python
# GPG密钥官方秘钥
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -

#中科大GPG密钥 
curl -fsSL https://mirrors.ustc.edu.cn/docker-ce/linux/ubuntu/gpg | sudo apt-key add -
~~~


3. Docker APT 软件源添加到你的系统

+ Docker软件源**官方软件源**与**中科大软件源**只需要**运行一个**就行
+ 国内建议用**中科大软件源**

~~~python
# Docker APT 软件源添
sudo add-apt-repository "deb [arch=amd64] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable"

# 中科大软件源添
sudo add-apt-repository "deb [arch=amd64] https://mirrors.ustc.edu.cn/docker-ce/linux/ubuntu $(lsb_release -cs) stable" 
~~~

4. 更新软件源
~~~python
sudo apt-get update
~~~

5. 安装docker
~~~python
sudo apt-get --no-install-recommends install -y docker-ce docker-ce-cli containerd.io docker-compose-plugin
~~~


6.去除docker命令行前的sudo权限

+ docker without root permissions

~~~python
#添加用户组
sudo groupadd docker

#将当前用户添加至用户组
sudo usermod -aG docker $USER

# 更新
newgrp docker
~~~

**现在运行docker就不需要加sudo了**

### 4.2 env

1. 拉取ros镜像
~~~python
docker pull osrf/ros:noetic-desktop-full
~~~

+ 解决拉取慢

~~~python
sudo vim /etc/docker/daemon.json
~~~

将下面内容放入`/etc/docker/daemon.json`文件中
~~~python
{
    "registry-mirrors":[
        "https://9cpn8tt6.mirror.aliyuncs.com",
        "https://registry.docker-cn.com"
    ]
}
~~~

+ 生效
~~~python
sudo systemctl daemon-reload
sudo systemctl restart docker
~~~

2. 启动镜像
~~~python
sudo docker run -it -v /tmp/.X11-unix:/tmp/.X11-unix --env="DISPLAY=$DISPLAY" --network host --name=lidar2cam osrf/ros:noetic-desktop-full /bin/bash 
~~~

+ 与宿主机共享文件夹
~~~
sudo docker run -it -v /tmp/.X11-unix:/tmp/.X11-unix --env="DISPLAY=$DISPLAY" -v /home/lin/Documents/share:/share --network host --name=lidar2cam osrf/ros:noetic-desktop-full /bin/bash
~~~

+ ros环境变量
~~~python
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
~~~


3. 依赖配置
~~~python
sudo apt update
sudo apt install git liblapack-dev libsuitesparse-dev libcxsparse3 libgflags-dev libgoogle-glog-dev libgtest-dev
~~~

4. 拉取标定功能包
~~~
mkdir -p catkin_ws/src
cd catkin_ws/src
git clone https://gitee.com/linClubs/lidar2cam_calibration.git
~~~

5. 安装ceres
~~~python
cd lidar2cam_calibration/Thirdparty
tar -xf ceres-solver-1.14.0.tar.xz
cd ceres-solver-1.14.0
mkdir build 
cd build
cmake ..
make -j8
sudo make install
~~~

6. 编译功能包
~~~python
cd /catkin_ws
catkin_make
~~~

### 4.3 标定

+ 启动标定程序

1. ros环境生效
~~~python
source devel/setup.bash
~~~

2. 启动采集数据程序
~~~python
roslaunch lidar2cam_calibration data_frame_extraction_node.launch
~~~

3. 估计外参程序
~~~python
roslaunch lidar2cam_calibration estimation_tf_node.launch
~~~


## 5 docker脚本自动构建环境

1. 宿主机可视化
~~~
sudo apt-get --no-install-recommends install -y x11-xserver-utils
xhost +
~~~

2. 启动docker容器
~~~
docker run -it -v /tmp/.X11-unix:/tmp/.X11-unix --env="DISPLAY=$DISPLAY" --network host --name=lidar2cam ros:1.0 /bin/bash 
~~~

3. 编译代码
~~~python
cd /catkin_ws
catkin_make
source devel/setup.bash
~~~

4. 启动采集数据程序
~~~python
roslaunch lidar2cam_calibration data_frame_extraction_node.launch
~~~

5. 估计外参程序
~~~python
roslaunch lidar2cam_calibration estimation_tf_node.launch
~~~
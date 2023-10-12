#  build all env 

## 1 dependent install

+ 本机保证`ubuntu-20.04，ros-noetic-desktop-full`完整版就行
+ `ros-noetic-desktop-full`自带 **`eigen-3.3.7, opencv-4.2.0, pcl-1.10`** 

~~~
./dependent.sh
~~~


## 2 docker build all env 

+ 环境使用`Ubuntu20.04`版本 `+ docker + ros-noetic-desktop-full`

---

### 2.1 安装docker

1. 宿主机配置（非新系统可以跳过）

只演示ubuntu环境(新系统)

+ 换源 [清华源链接](https://mirrors.tuna.tsinghua.edu.cn/help/ubuntu/)

`把/etc/apt/sources.list`内容换成清华源

~~~python
# 首先备份源列表
sudo cp /etc/apt/sources.list /etc/apt/sources.list_backup
# 编辑
gedit /etc/apt/sources.list
~~~

+ `/etc/apt/sources.list`换成下面的内容

~~~python
deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ focal main restricted universe multiverse
deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ focal-updates main restricted universe multiverse
deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ focal-backports main restricted universe multiverse
deb http://security.ubuntu.com/ubuntu/ focal-security main restricted universe multiverse
~~~

+ 更新

~~~python
sudo apt update
~~~

2. 安装docker

+ 需要加sudo,不然权限不够,无法向`/etc/docker/daemon.json`写入文字

~~~python
sudo ./docker_install.sh
~~~

---

### 2.2 制作`docker`镜像并启动容器

+ 分为**一键制作**和**详细制作**,快速构建环境选**一键制作**

---
+ **一键制作**

1. **一键制作docker容器与创建镜像**
~~~python
./docker_build.sh
~~~

2. 进入镜像
~~~python
# 新开一个终端, 重启镜像
docker restart test01

# 进入test01镜像
docker exec -it test01 /bin/bash
~~~

3. test01镜像中编译
~~~
# 进入工作空间
cd /calibration_ws/src/calibration

# 编译
./build.sh
~~~

---
+ **详细制作**

1. 制作标定程序`docker`镜像

+ 脚本运行
~~~python
./run_dockerfile.sh
~~~

+ 代码创建镜像
~~~python
docker build -f dockerfile -t calibrate:v1.0 .
~~~

+ 镜像终端命令
~~~python
# 查看镜像列表
docker images
# 删除镜像
docker rmi ${image_id}
~~~

2. 启动一个标定程序docker容器

3. 在宿主机可视化
~~~python
sudo apt-get --no-install-recommends install -y x11-xserver-utils
~~~

+ 启动xhost
~~~python
xhost +
~~~

4. 新建一个容器

+ 代码创建
~~~python
sudo docker run -it -v /tmp/.X11-unix:/tmp/.X11-unix --env="DISPLAY=$DISPLAY" -v $(pwd):/calibration_ws/src/calibration --network host --name=test01 calibrate:v1.0 /bin/bash 
~~~

+ 参数说明
~~~python
# 宿主机可视化
-v /tmp/.X11-unix:/tmp/.X11-unix --env="DISPLAY=$DISPLAY"

# 共享文档 宿主机目录:从机目录
-v calibration:/calibration_ws/src/calibration
# $(pwd) 表示当前路径

# 共享rosmaster
--network host
~~~



+ 容器终端命令
~~~python
# 查看容器列表
docker ps -a(-as)

# 删除容器
docker rm ${container_name}
~~~

---

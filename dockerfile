FROM osrf/ros:noetic-desktop-full

# ---------1. setup environment-----------
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
ENV DEBIAN_FRONTEND noninteractive

# ---------2. update sources list-----------
RUN cp -r /etc/apt/sources.list /etc/apt/sources.list_back
RUN echo "deb http://mirrors.ustc.edu.cn/ubuntu/ focal main restricted universe multiverse" > /etc/apt/sources.list
RUN echo "deb http://mirrors.ustc.edu.cn/ubuntu/ focal-updates main restricted universe multiverse" >> /etc/apt/sources.list
RUN echo "deb http://mirrors.ustc.edu.cn/ubuntu/ focal-backports main restricted universe multiverse" >> /etc/apt/sources.list
RUN echo "deb http://mirrors.ustc.edu.cn/ubuntu/ focal-security main restricted universe multiverse" >> /etc/apt/sources.list

# update
RUN apt-get update

# ---------3. install dependent lib -----------
# --- 3.1 base install---
RUN DEBIAN_FRONTEND=noninteractive apt-get install -y vim wget autoconf automake

# --- 3.2 optional install---
# RUN DEBIAN_FRONTEND=noninteractive apt-get install -y mlocate net-tools openssh-server

# --- 3.3 camera_calibration camera_calibration_py dependent ---
RUN DEBIAN_FRONTEND=noninteractive apt-get install -y \
    python3-dev python3-pip python3-catkin-tools
   
RUN pip3 install opencv-python==4.7.0.72 \
        numpy==1.23.0 -i https://pypi.tuna.tsinghua.edu.cn/simple

# --- 3.4 kalibr dependent ---
# RUN DEBIAN_FRONTEND=noninteractive apt-get install -y \
#     python3-dev python3-pip python3-scipy python3-matplotlib \
#     ipython3 python3-wxgtk4.0 python3-tk python3-igraph libv4l-dev

RUN DEBIAN_FRONTEND=noninteractive apt-get install -y \
	python3-dev python3-pip python3-scipy python3-matplotlib \
	ipython3 python3-wxgtk4.0 python3-tk python3-igraph python3-pyx \
    libboost-all-dev libsuitesparse-dev \
	doxygen \
	libpoco-dev libtbb-dev libblas-dev liblapack-dev libv4l-dev \
	python3-catkin-tools python3-osrf-pycommon

# ---ceres dependent---
RUN DEBIAN_FRONTEND=noninteractive apt-get install -y \
    liblapack-dev libsuitesparse-dev libcxsparse3 \
    libgflags-dev libgoogle-glog-dev libgtest-dev

# --- 3.5 imu_calibration dependent ---
RUN DEBIAN_FRONTEND=noninteractive apt-get install -y \
    libdw-dev


# ---------4. source ros env -----------
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

# ENV WORKSPACE /catkin_ws
# -----git calibration code------------
# RUN mkdir -p $WORKSPACE/src && \
#     git clone https://gitee.com/linClubs/calibration.git -o $WORKSPACE/src

# ---------build ceres-----------
# RUN cd $WORKSPACE/src/calibration/Thirdparty && \
#     tar -xf ceres-solver-1.14.0.tar.xz && \ 
#     cd ceres-solver-1.14.0 && \
#     mkdir build; cd build && \
#     cmake .. && \
#     make -j8 && \
#     make install

CMD ["bash"]

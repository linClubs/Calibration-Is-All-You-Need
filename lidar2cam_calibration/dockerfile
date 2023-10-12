FROM osrf/ros:noetic-desktop-full

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
ENV DEBIAN_FRONTEND noninteractive

RUN cp -r /etc/apt/sources.list /etc/apt/sources.list_back
RUN echo "deb http://mirrors.ustc.edu.cn/ubuntu/ focal main restricted universe multiverse" > /etc/apt/sources.list
RUN echo "deb http://mirrors.ustc.edu.cn/ubuntu/ focal-updates main restricted universe multiverse" >> /etc/apt/sources.list
RUN echo "deb http://mirrors.ustc.edu.cn/ubuntu/ focal-backports main restricted universe multiverse" >> /etc/apt/sources.list
RUN echo "deb http://mirrors.ustc.edu.cn/ubuntu/ focal-security main restricted universe multiverse" >> /etc/apt/sources.list

RUN apt-get update

# ----- base set------------
# RUN apt-get install --assume-yes apt-utils
RUN apt-get install git -y

# --ros env--
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

# --git lidar2cam_calibration--
RUN mkdir -p catkin_ws/src
RUN cd catkin_ws/src
RUN git clone https://gitee.com/linClubs/lidar2cam_calibration.git

# --ceres--
RUN apt-get install -y liblapack-dev libsuitesparse-dev libcxsparse3 libgflags-dev libgoogle-glog-dev libgtest-dev
RUN cd lidar2cam_calibration/Thirdparty \
    && tar -xf ceres-solver-1.14.0.tar.xz \
    && cd ceres-solver-1.14.0 \
    && mkdir build \
    && cd build \
    && cmake .. \
    && make -j8 \
    && make install

CMD ["bash"]
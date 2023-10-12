+ 编译kalibr时aslam_cv_pythonConfig.Cmake找不到

解决办法

+ 先编译aslam_cv_python在编译kalibr
~~~
~~~

kalibr/aslam_cv/aslam_cv_python

---

+ numpy/arrayobject.h: 没有那个文件或目录
~~~python
In file included from /home/lin/fusion_ws/src/kalibr/Schweizer-Messer/numpy_eigen/src/autogen_module/import_1_1_uchar.cpp:4:
/home/lin/fusion_ws/src/kalibr/Schweizer-Messer/numpy_eigen/include/numpy_eigen/NumpyEigenConverter.hpp:21:10: fatal error: numpy/arrayobject.h: 没有那个文件或目录
   21 | #include <numpy/arrayobject.h>
      |          ^~~~~~~~~~~~~~~~~~~~~
compilation terminated.
~~~

修改如下：
~~~python
# numpy/arrayobject.h查看位置
locate numpy/arrayobject.h
# 我的在
/usr/lib/python3/dist-packages/numpy/core/include/numpy/arrayobject.h

# 就在下面文件修改：
./Schweizer-Messer/numpy_eigen/CMakeLists.txt
# 加入
include_directories("/usr/lib/python3/dist-packages/numpy/core/include/")
~~~




---

![Kalibr](https://raw.githubusercontent.com/wiki/ethz-asl/kalibr/images/kalibr_small.png)

<!--*Ubuntu 14.04+ROS indigo*: [![Build Status](https://jenkins.asl.ethz.ch/buildStatus/icon?job=kalibr_weekly/label=ubuntu-trusty)](https://jenkins.asl.ethz.ch/job/kalibr_weekly/label=ubuntu-trusty/) *Ubuntu 16.04+ROS kinetic*: [![Build Status](https://jenkins.asl.ethz.ch/buildStatus/icon?job=kalibr_weekly/label=ubuntu-trusty)](https://jenkins.asl.ethz.ch/job/kalibr_weekly/label=ubuntu-xenial/)-->

~~~python
sudo apt install python3-dev python3-pip python3-scipy python3-matplotlib ipython3 python3-wxgtk4.0 python3-tk python3-igraph python3-pyx libboost-all-dev libsuitesparse-dev doxygen libpoco-dev libtbb-dev libblas-dev liblapack-dev libv4l-dev python3-catkin-tools python3-osrf-pycommon
~~~

## Introduction
Kalibr is a toolbox that solves the following calibration problems:

1. **Multiple camera calibration**: 
    intrinsic and extrinsic calibration of a camera-systems with non-globally shared overlapping fields of view
1. **Visual-inertial calibration calibration (camera-IMU)**:
    spatial and temporal calibration of an IMU w.r.t a camera-system
1. **Rolling Shutter Camera calibration**:
    full intrinsic calibration (projection, distortion and shutter parameters) of rolling shutter cameras


**Please find more information on the [wiki pages](https://github.com/ethz-asl/kalibr/wiki) of this repository.**

**For questions or comments, please open an issue on Github.**

## Installation

### Ubuntu 20.04

We've upgraded and fixed kalibr at ORI for 20.04. Please use our fork: `git clone https://github.com/ori-drs/kalibr.git --branch noetic-devel`.

- Use `rosdep` to install almost all required dependencies: `rosdep install --from-paths ./ -iry`.
- Then install the two missing runtime dependencies: `sudo apt install python3-wxgtk4.0 python3-igraph`
- Unittests are currently failing on 20.04 and thus deactivated on the buildserver.

## Tutorial: IMU-camera calibration
A video tutorial for the IMU-camera calibration can be found here:

[![alt text](https://user-images.githubusercontent.com/5337083/44033014-50208b8a-9f09-11e8-8e9a-d7d6d3c69d97.png)](https://m.youtube.com/watch?v=puNXsnrYWTY "imu cam calib")

(Credits: @indigomega)

## Authors
* Paul Furgale ([email](paul.furgale@mavt.ethz.ch))
* Hannes Sommer ([email](hannes.sommer@mavt.ethz.ch))
* Jérôme Maye ([email](jerome.maye@mavt.ethz.ch))
* Jörn Rehder ([email](joern.rehder@mavt.ethz.ch))
* Thomas Schneider ([email](schneith@ethz.ch))
* Luc Oth

## References
The calibration approaches used in Kalibr are based on the following papers. Please cite the appropriate papers when using this toolbox or parts of it in an academic publication.

1. <a name="joern1"></a>Joern Rehder, Janosch Nikolic, Thomas Schneider, Timo Hinzmann, Roland Siegwart (2016). Extending kalibr: Calibrating the extrinsics of multiple IMUs and of individual axes. In Proceedings of the IEEE International Conference on Robotics and Automation (ICRA), pp. 4304-4311, Stockholm, Sweden.
1. <a name="paul1"></a>Paul Furgale, Joern Rehder, Roland Siegwart (2013). Unified Temporal and Spatial Calibration for Multi-Sensor Systems. In Proceedings of the IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), Tokyo, Japan.
1. <a name="paul2"></a>Paul Furgale, T D Barfoot, G Sibley (2012). Continuous-Time Batch Estimation Using Temporal Basis Functions. In Proceedings of the IEEE International Conference on Robotics and Automation (ICRA), pp. 2088–2095, St. Paul, MN.
1. <a name="jmaye"></a> J. Maye, P. Furgale, R. Siegwart (2013). Self-supervised Calibration for Robotic Systems, In Proc. of the IEEE Intelligent Vehicles Symposium (IVS)
1. <a name="othlu"></a>L. Oth, P. Furgale, L. Kneip, R. Siegwart (2013). Rolling Shutter Camera Calibration, In Proc. of the IEEE Computer Vision and Pattern Recognition (CVPR)

## Acknowledgments
This work is supported in part by the European Union's Seventh Framework Programme (FP7/2007-2013) under grants #269916 (V-Charge), and #610603 (EUROPA2).

## License (BSD)
Copyright (c) 2014, Paul Furgale, Jérôme Maye and Jörn Rehder, Autonomous Systems Lab, ETH Zurich, Switzerland<br>
Copyright (c) 2014, Thomas Schneider, Skybotix AG, Switzerland<br>
All rights reserved.<br>

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

1. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

1. All advertising materials mentioning features or use of this software must display the following acknowledgement: This product includes software developed by the Autonomous Systems Lab and Skybotix AG.

1. Neither the name of the Autonomous Systems Lab and Skybotix AG nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE AUTONOMOUS SYSTEMS LAB AND SKYBOTIX AG ''AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL the AUTONOMOUS SYSTEMS LAB OR SKYBOTIX AG BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

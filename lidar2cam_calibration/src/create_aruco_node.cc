#include "create_aruco.h"
#include "memory"

#include <ros/ros.h> // 用ros来获取参数

int main(int argc, char** argv)
{   
    // int dictId = 8;
    // double marker_length = 400;
    // int markerId;
    // std::string img_save_path = "/home/lin/ros_code/calibration_ws/src/lidar2cam_calibration/data";

    double marker_length;        // aruco二维码的边长
    int markerId;                // aruco二维码的id号
    int dictId;                  // 创建aruco二维码cv::aruco::getPredefinedDictionary函数的参数
    std::string img_save_path;   // aruco二维码图片保存路径

    ros::init(argc, argv, "create_aruco_node");
    ros::NodeHandle n;

    n.getParam("ArucoID", dictId);
    n.param<int>("ArucoID", dictId, 8);

    n.getParam("marker_size", marker_length);
    n.param<double>("marker_size", marker_length, 0.4);
    marker_length = marker_length * 1000;  // 扩大100倍, 函数中是像素值, 保证像素大小，不能太小
    
    n.getParam("pkg_path", img_save_path);
    n.param<std::string>("pkg_path",img_save_path, "/home/lin/ros_code/calibration_ws/src/lidar2cam_calibration");
    img_save_path = img_save_path + "/data/marker_img";

    std::cout << "\n-------------------" << std::endl;
    std::cout << "dictId: "<< dictId << std::endl;
    std::cout << "marker_length: "<< marker_length << std::endl;
    std::cout << "img_save_path: "<< img_save_path << std::endl;
    std::cout << "\n-------------------" << std::endl;

    std::shared_ptr<lidar2cam_calibration::CreateAruco> creator = 
        std::make_shared<lidar2cam_calibration::CreateAruco>(dictId, marker_length);
    
    for(int i = 1; i < 5; i++)
    {   
        markerId = i;
        creator->setMarkerId(markerId);
        creator->makeArucoImg();
        std::cout << "\n--Please press any key to display the next aruco image..." << std::endl;
        creator->showArucoImg();
        creator->saveArucoImg(img_save_path);
    }

    return 0;
}
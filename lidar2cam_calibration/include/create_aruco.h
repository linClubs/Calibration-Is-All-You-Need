#ifndef ceres_aruco_h
#define ceres-aruco_h

#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>

namespace lidar2cam_calibration
{

class CreateAruco
{
 private:
    cv::Mat img_;
    double marker_length_;
    int markerId_;
    int dictId_;
    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    cv::Ptr<cv::aruco::DetectorParameters> parameters_;
    
    std::string img_save_path_;
 public:
    CreateAruco(const int& dictId, const double& marker_length);
    void setMarkerId(const int& markerId);
    void makeArucoImg();
    void showArucoImg();
    void saveArucoImg(std::string img_save_path);
};

} //namespace lidar2cam_calibration

#endif
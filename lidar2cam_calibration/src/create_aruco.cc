#include "create_aruco.h"

lidar2cam_calibration::CreateAruco::CreateAruco(const int& dictId, const double& marker_length):
dictId_(dictId), marker_length_(marker_length)
{   
    dictionary_ = cv::aruco::getPredefinedDictionary(dictId_);
    parameters_ = cv::aruco::DetectorParameters::create(); 
    
}

void lidar2cam_calibration::CreateAruco::setMarkerId(const int& markerId)
{
    markerId_ = markerId;
}

void lidar2cam_calibration::CreateAruco::makeArucoImg()
{
    cv::aruco::drawMarker(dictionary_, markerId_, marker_length_, img_);
}

void lidar2cam_calibration::CreateAruco::showArucoImg()
{
    std::string winName = "maker" + std::to_string(markerId_);
    cv::imshow(winName, img_);
    cv::waitKey(0);
}

void lidar2cam_calibration::CreateAruco::saveArucoImg(std::string img_save_path)
{   
    img_save_path_ = img_save_path + "/maker" + std::to_string(markerId_) + ".png";
    cv::imwrite(img_save_path_, img_);
    std::cout << "Save " << marker_length_ << "*" << marker_length_ << " ArUco image to \"" << img_save_path_ << "\" successfully." << std::endl;
}



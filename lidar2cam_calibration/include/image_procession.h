#ifndef image_procession_h
#define image_procession_h

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <memory>
#include <ros/ros.h>

namespace lidar2cam_calibration
{

class ImageProcession
{
    public:

        bool flag_;
        ros::NodeHandle n_;
        int arucoId_;

        cv::Mat src_;
        cv::Mat img_;
        // cv::Mat a(cv::Size(5,5),CV_8UC1);  类中不能这个定义
        
        float fx, fy, cx, cy;
        float k1, k2, k3, p1, p2;
        
        cv::Mat cameraMatrix_;
        cv::Mat distCoeffs_;

        cv::Vec3d rvec_, tvec_;

        cv::Ptr<cv::aruco::Dictionary> dictionary;
        cv::Ptr<cv::aruco::DetectorParameters> parameters;

        std::vector<int> boardIds;  // IDs order as explained above
        // 创建了4个标志格子
        cv::Ptr<cv::aruco::Board> board;
            
        // Create vector of markers corners. 4 markers * 4 corners
        // Markers order:
        // 0-------1
        // |       |
        // |   C   |
        // |       |
        // 3-------2
        int min_detected_markers_;
        float marker_size_, delta_width_qr_center_, delta_height_qr_center_;
        float board_width;
        float board_height;
        
        std::vector<std::vector<cv::Point3f>> boardCorners; 
        std::vector<cv::Point3f> board_vertex_;
        std::vector<cv::Point2f> p_pixel_vector_;


    public:
        ImageProcession();
        ~ImageProcession(){};
        void getImage(cv::Mat& img);
        cv::Mat detectMarkers();

        // 相机坐标到像素坐标uv
        cv::Point2f projectPointDist(cv::Point3f& pt_cv, const cv::Mat& intrinsics, const cv::Mat& distCoeffs);
        // 得到图像中点在相机坐标系下坐标
        std::vector<cv::Point3f> getBoardCenter3d();//
        void getConfigFileParam();
};

} // namespace lidar2cam_calibration

#endif
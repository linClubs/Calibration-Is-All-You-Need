#include "image_procession.h"


namespace lidar2cam_calibration
{
#define TARGET_NUM_CIRCLES 4
ImageProcession::ImageProcession()
{   
    
    getConfigFileParam();
    
    flag_ = false;
    // img_ = src_.clone();
   
    // DICT_6X6_50 8
    dictionary = cv::aruco::getPredefinedDictionary(arucoId_);
    // Detect markers
    // 亚像素
    parameters = cv::aruco::DetectorParameters::create();
    #if (CV_MAJOR_VERSION == 3 && CV_MINOR_VERSION <= 2) || CV_MAJOR_VERSION < 3
    parameters->doCornerRefinement = true;
    #else
    parameters->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
    #endif
    
    // marker_size_ = 0.1;
    // min_detected_markers_ = 3;
    // delta_width_qr_center_ =  0.55;  // 左右二维码中心距
    // delta_height_qr_center_ = 0.35; // // 上下二维码中心距

    // board_width = 0.7;  // 板宽度
    // board_height = 0.5; // 板高度

    float width = delta_width_qr_center_ ;
    float height = delta_height_qr_center_;

    boardCorners.resize(4);
    // 以标定板为中心获得4个二维码的3d位姿
    for (int i = 0; i < 4; ++i) 
    {
        int x_qr_center = (i % 3) == 0 ? -1 : 1;  // （-1， 1， 1， -1）
        int y_qr_center = (i < 2) ? 1 : -1; //  （1， 1， -1， -1）   
        
        float x_center = x_qr_center * width;   // （-1， 1， 1， -1）
        float y_center = y_qr_center * height; 
        
        board_vertex_.emplace_back(
            cv::Point3f(board_width / 2.0 * x_qr_center, 
                        board_height / 2.0 * y_qr_center,
                        0));

        for(int j = 0; j < 4; ++j)
        {
            int x_qr = (j % 3) == 0 ? -1 : 1;
            int y_qr = (j < 2) ? 1 : -1; 

            cv::Point3f pt3d(x_center / 2.0 + x_qr * marker_size_ / 2.,
                       y_center / 2.0 + y_qr * marker_size_ / 2., 0); //  4个标定板左上角点坐标
           
            // std::cout << pt3d << std::endl;
            boardCorners[i].push_back(pt3d);
        }
        // std::cout << "-------------" << std::endl;
       
    }

    boardIds = std::vector<int> {1, 2, 4, 3};  // IDs order as explained above
  
  // 创建了4个标志格子 
    board = cv::aruco::Board::create(boardCorners, dictionary, boardIds);


}

void ImageProcession::getImage(cv::Mat& img)
{
    src_ = img.clone();
}

cv::Mat ImageProcession::detectMarkers()
{   
    flag_ = false;
    img_ = src_.clone();
    std::vector<int> ids;       
    std::vector<std::vector<cv::Point2f>> corners; 

    // 1 检测Marker标志，返回id和corner(cv::Point2f精度)的列表
    cv::aruco::detectMarkers(img_, dictionary, corners, ids, parameters);

     // Draw detections if at least one marker detected
    if (ids.size() >= min_detected_markers_ && ids.size() <= TARGET_NUM_CIRCLES ) 
    {   
        flag_ = true;
        cv::aruco::drawDetectedMarkers(img_, corners, ids); // 检测到了就画图

        // estimatePoseSingleMarkers 中rvecs，tvecs为cv::Vec3d精度
        std::vector<cv::Vec3d> rvecs, tvecs;
        cv::aruco::estimatePoseSingleMarkers(corners, marker_size_, cameraMatrix_,
                                            distCoeffs_, rvecs, tvecs);
    
        for (int i = 0; i < rvecs.size(); i++) 
        {
            cv::aruco::drawAxis(img_, cameraMatrix_, distCoeffs_, rvecs[i], tvecs[i], 0.1);
            // std::cout << "rvec = " << rvecs[i] << std::endl;
            // std::cout << "tvec = " << tvecs[i] << std::endl;     
        }
        
        cv::Vec3d rvec, tvec;
       
        // Estimate 3D position of the board using detected markers
        int valid = cv::aruco::estimatePoseBoard(corners, ids, board, cameraMatrix_, distCoeffs_, rvec, tvec);
  
        cv::aruco::drawAxis(img_, cameraMatrix_, distCoeffs_, rvec, tvec, 0.1);
        // std::cout << "rvec = " << rvec << std::endl;
        // std::cout << "tvec = " << tvec << std::endl;

        rvec_ = rvec;
        tvec_ = tvec;
        // std::cout << "------------" << std::endl;
        
    }
    // std::cout << "\t--The number of detection calibration aruco is " << ids.size()  << " ..." << std::endl;
    return img_;
}


// 相机坐标到像素坐标uv
cv::Point2f ImageProcession::projectPointDist(cv::Point3f& pt_cv, const cv::Mat& intrinsics, const cv::Mat& distCoeffs)
{
// Project a 3D point taking into account distortion
    std::vector<cv::Point3f> input{pt_cv};
    std::vector<cv::Point2f> projectedPoints;
    projectedPoints.resize(1);  // TODO: Do it batched? (cv::circle is not batched anyway)
    
    // 相机坐标系->像素坐标，根据内参，畸变
    cv::projectPoints(input, cv::Mat::zeros(3, 1, CV_64FC1), cv::Mat::zeros(3, 1, CV_64FC1),
                    intrinsics, distCoeffs, projectedPoints);
    
    return projectedPoints[0]; 
}

std::vector<cv::Point3f> ImageProcession::getBoardCenter3d()
{   
    cv::Mat R(3, 3, cv::DataType<float>::type);  // 旋转
    cv::Rodrigues(rvec_, R);

    cv::Mat t = cv::Mat::zeros(3, 1, CV_32F); // 平移
    t.at<float>(0) = tvec_[0];
    t.at<float>(1) = tvec_[1];
    t.at<float>(2) = tvec_[2];

    // std::cout << "R:\n" << R << std::endl;
    // std::cout << "t:\n" << t << std::endl;
    
    cv::Mat board_transform = cv::Mat::eye(3, 4, CV_32F);
    R.copyTo(board_transform.rowRange(0, 3).colRange(0, 3));
    t.copyTo(board_transform.rowRange(0, 3).col(3));

    std::vector<cv::Mat> board_coord_vector;
    std::vector<cv::Point3f> p_cam_vector;
    p_pixel_vector_.clear();

    cv::Point2f uv;
    cv::Point3f p_cam;
    cv::Mat p = cv::Mat::zeros(4, 1, CV_32F); // 齐次坐标, 中心点在board坐标系下为(0,0,0,1)
    
    cv::Mat p_center = cv::Mat::zeros(4, 1, CV_32F);
    p_center.at<float>(3, 0) = 1.0; 

    board_coord_vector.emplace_back(p_center);
    
    p_center = board_transform * p_center;  // 矩阵乘法后，维度变了p_center变3维度了
   
    p_cam.x = p_center.at<float>(0, 0);
    p_cam.y = p_center.at<float>(1, 0);
    p_cam.z = p_center.at<float>(2, 0);  

    p_cam_vector.emplace_back(p_cam); //相机坐标系下坐标

    uv = projectPointDist(p_cam, cameraMatrix_,distCoeffs_);
    cv::circle(img_, uv, 7, cv::Scalar(0, 0, 255), -1);
    p_pixel_vector_.emplace_back(uv); // 像素坐标系下坐标


    for(auto it : board_vertex_)   // 保存5个点,中心点,左上，右上，右下，左下
    {   
    //     // 更新标定板坐标系下坐标，左上，右上，右下，左下
        cv::Mat p = cv::Mat::ones(4, 1, CV_32F);
        p.at<float>(0, 0) = it.x;
        p.at<float>(1, 0) = it.y;
        p.at<float>(2, 0) = it.z;
        p.at<float>(3, 0) = 1.0; 

        // std::cout << "p:\n" <<  p << std::endl;
        board_coord_vector.emplace_back(p);    

        p = board_transform * p;
        p_cam.x = p.at<float>(0, 0);
        p_cam.y = p.at<float>(1, 0);
        p_cam.z = p.at<float>(2, 0);  

        p_cam_vector.emplace_back(p_cam); // 相机坐标系下坐标

        uv = projectPointDist(p_cam, cameraMatrix_,distCoeffs_);
        
        p_pixel_vector_.emplace_back(uv); // 像素坐标系下坐标
        cv::circle(img_, uv, 7, cv::Scalar(0, 0, 255), -1);
         
    }

    // std::cout << "--------coodr---------" << std::endl;
    // for(size_t i = 0; i < board_coord_vector.size(); ++i)
    // {
    //     std::cout << board_coord_vector[i] << std::endl;
    // }

    // std::cout << "p_cam(center,lt,rt,rd,ld):" << std::endl;

    // for(size_t i = 0; i < board_coord_vector.size(); ++i)
    // {
    //     std::cout << p_cam_vector[i] << std::endl;
    // }  

    // std::cout << std::endl;

    // for(size_t i = 0; i < board_coord_vector.size(); ++i)
    // {
    //     std::cout << p_pixel_vector[i] << std::endl;
    // }

    // std::cout << "------------------" << std::endl;
   
    
    return p_cam_vector;
}

void ImageProcession::getConfigFileParam()
{

    n_.getParam("marker_size", marker_size_);
    n_.param<float>("marker_size", marker_size_, 0.1);
    
    n_.getParam("min_detected_markers", min_detected_markers_);
    n_.param("min_detected_markers", min_detected_markers_, 3);

    n_.getParam("delta_width_qr_center", delta_width_qr_center_);
    n_.param<float>("delta_width_qr_center", delta_width_qr_center_, 0.55); // 左右二维码中心距

    n_.getParam("delta_height_qr_center", delta_height_qr_center_);
    n_.param<float>("delta_height_qr_center", delta_height_qr_center_, 0.35);// 上下二维码中心距
    
    n_.getParam("board_width", board_width);
    n_.param<float>("board_width", board_width, 0.7);// 板宽度
    
    n_.getParam("board_height", board_height);
    n_.param<float>("board_height", board_height, 0.5); // 板高度

    n_.getParam("ArucoID", arucoId_);
    n_.param<int>("ArucoID", arucoId_, 8); 

    // -------------内参-----------------------
    n_.getParam("fx", fx);
    n_.param<float>("fx", fx, 617.201); 
    n_.getParam("fy", fy);
    n_.param<float>("fy", fy, 617.362); 
    n_.getParam("cx", cx);
    n_.param<float>("cx", cx, 324.637); 
    n_.getParam("cy", cy);
    n_.param<float>("cy", cy, 242.462); 
    
    n_.getParam("k1", k1);
    n_.param<float>("k1", k1, 0.0); 
    n_.getParam("k2", k2);
    n_.param<float>("k2", k2, 0.0); 
    n_.getParam("k3", k3);
    n_.param<float>("k3", k3, 0.0); 
    
    n_.getParam("p1", p1);
    n_.param<float>("p1", p1, 0.0); 
    n_.getParam("p2", p2);
    n_.param<float>("p2", k2, 0.0); 

    cameraMatrix_ = (cv::Mat_<float>(3, 3) << fx, 0., cx,
                                               0, fy, cy,
                                               0, 0., 1);
    
    distCoeffs_ = (cv::Mat_<float>(5, 1) << k1, k2, p1, p2, k3);  // 初始化加（）
    
    std::cout << "\n-------------------" << std::endl;
    std::cout << "marker_size: "<< marker_size_ << std::endl;
    std::cout << "min_detected_markers: "<< min_detected_markers_ << std::endl;
    std::cout << "delta_width_qr_center: "<< delta_width_qr_center_ << std::endl;
    std::cout << "delta_height_qr_center: "<< delta_height_qr_center_ << std::endl;
    std::cout << "board_width: "<< board_width << std::endl;
    std::cout << "board_height: "<< board_height << std::endl;
    std::cout << "ArucoID: "<< arucoId_ << std::endl;

    std::cout << "\n---------------" << std::endl;
    std::cout << "cameraMatrix: \n"<< cameraMatrix_ << std::endl;
    std::cout << "distCoeffs: \n"<< distCoeffs_.at<float>(0,0) << " " <<
                                    distCoeffs_.at<float>(1,0) << " " <<
                                    distCoeffs_.at<float>(2,0) << " " <<
                                    distCoeffs_.at<float>(3,0) << " " <<
                                    distCoeffs_.at<float>(4,0) << " " << std::endl;
    
    std::cout << "\n---------------" << std::endl;
}

} // namespace lidar2cam_calibration
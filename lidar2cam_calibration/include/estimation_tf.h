#ifndef estimation_tf_h
#define estimation_tf_h

#include <fstream>
#include<string>     //包含getline()
#include <vector>
#include <chrono>
#include <ros/ros.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/SVD>

#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>


#include <ceres/ceres.h>
#include <ceres/rotation.h>

struct PointSet
{
   std::vector<Eigen::Vector2d> p_uv_vector_;
   std::vector<Eigen::Vector3d> p_cam_vector_;
   std::vector<Eigen::Vector3d> p_lidar_vector_;

   std::vector<cv::Point2f> pts_uv_cv_;
   std::vector<cv::Point3f> pts_lidar_cv_;
};

// 代价函数的计算模型
// 3d-3d
struct TfCostFunctor
{   
    Eigen::Vector3d p_lidar_;
    Eigen::Vector3d p_cam_;
    
    TfCostFunctor(Eigen::Vector3d p_lidar, Eigen::Vector3d p_cam): p_lidar_(p_lidar), p_cam_(p_cam){}
    template <typename T>
    bool operator()(const T* r, const T* t, T* residual) const
    {
        T p_Lidar[3];
        T p_Cam[3];  //相机坐标系下空间点的坐标

        p_Lidar[0] = T(p_lidar_[0]);
        p_Lidar[1] = T(p_lidar_[1]);
        p_Lidar[2] = T(p_lidar_[2]);

        ceres::AngleAxisRotatePoint(r,p_Lidar,p_Cam);

        p_Cam[0] = p_Cam[0] + t[0];
        p_Cam[1] = p_Cam[1] + t[1];
        p_Cam[2] = p_Cam[2] + t[2];

        // 残差
        residual[0] = T(p_cam_[0]) - p_Cam[0];
        residual[1] = T(p_cam_[1]) - p_Cam[1];
        residual[2] = T(p_cam_[2]) - p_Cam[2];

        return true;
    }
};

bool readPoint(const std::string &path, std::shared_ptr<PointSet> point_set);

// ---1.1 旋转矩阵->欧拉角RPY(x,y,z)-----
inline void rotationMatrix2eulerAngles(const Eigen::Matrix3d& R, Eigen::Vector3d& euler)
{
    euler = R.eulerAngles(0, 1, 2);
    euler = euler * (180 / M_PI);  // 角度制
}
// -

inline void eigen2cv_vector(const Eigen::Matrix3d& R, cv::Mat& R_vector)
{
    cv::Mat cv_mat;
    cv::eigen2cv(R, cv_mat);
    cv::Rodrigues(cv_mat, R_vector);
}

void estimation_3d3d_SVD(const std::vector<Eigen::Vector3d> &p_cam, 
                            const std::vector<Eigen::Vector3d> &p_lidar,
                            Eigen::Matrix3d& R, Eigen::Vector3d& t);

// ceres求解3d2d
void optimation_3d3d(const std::vector<Eigen::Vector3d> &p_cam, 
                     const std::vector<Eigen::Vector3d> &p_lidar, 
                     Eigen::Matrix3d& R, Eigen::Vector3d& t);  

// g2o求解2d3d
void optimation_2d3d(const std::vector<Eigen::Vector2d> &p_uv, 
                            const std::vector<Eigen::Vector3d> &p_lidar,
                            Eigen::Matrix3d& R, Eigen::Vector3d& t);  

// opencv-solvePNP求解2d3d
void solvePNP_cv(const std::vector<cv::Point2f>& p_uv,
                    const std::vector<cv::Point3f>& p_lidar,
                    cv::Mat& rvec, cv::Mat& tvec);

#endif
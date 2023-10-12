#ifndef data_frame_extraction_h
#define data_frame_extraction_h

#include <memory>
#include <ros/ros.h>
#include <fstream>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <sensor_msgs/Image.h>


#include <pcl/point_types.h>  //pcl库
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h> // ros中pcl_conversions带的fromROSMsg toROSMsg
#include <pcl/filters/passthrough.h> // 直通滤波
#include <pcl/io/pcd_io.h> //保存
#include <pcl/common/impl/common.hpp>


#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/subscriber.h>

#include <dynamic_reconfigure/server.h> //实现动态参数的头文件
#include <lidar2cam_calibration/filter_Config.h> // cfg生成的头文件

#include "image_procession.h"

#include <pcl/ModelCoefficients.h>
#include <pcl/PointIndices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/project_inliers.h>



namespace lidar2cam_calibration
{

typedef pcl::PointXYZI PointT;

struct FilterThre
{
    double x_max;
    double x_min;
    double y_max;
    double y_min;
    double z_max;
    double z_min;
};

class FrameExtraction
{
 public:
    ros::NodeHandle n_;
    std::string pkg_path_;

    std::string topic_img_, topic_cloud_, topic_cloud_filter_;
    cv::Mat src_;
    pcl::PointCloud<PointT>::Ptr cloud_ptr_;
    

    cv::Mat img_new_;
    std::string topic_img_new_, topic_cloud_new_;
    sensor_msgs::ImageConstPtr msg_img_;
    sensor_msgs::PointCloud2Ptr msg_cloud_, msg_cloud_filter_;
    ros::Publisher pub_img_, pub_cloud_, pub_cloud_filter_;
    std::string frame_id_;

    // 定义同步器
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_cloud_; // 消息1的订阅
    message_filters::Subscriber<sensor_msgs::Image> sub_img_; // 消息2的订阅
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> MySyncPolicy_;
    typedef message_filters::Synchronizer<MySyncPolicy_> Sync_;
    std::shared_ptr<Sync_> sync_;

    // //创建了一个参数动态配置的服务端
    dynamic_reconfigure::Server<lidar2cam_calibration::filter_Config> server_;
    dynamic_reconfigure::Server<lidar2cam_calibration::filter_Config>::CallbackType f_;

    std::shared_ptr<ImageProcession> image_process_;
    // cv::Point3f p_cam_boarc_center_;

    pcl::PointCloud<PointT>::Ptr cloud_filter_ptr_;
    FilterThre box;
    std::vector<cv::Point3f> p_cam_vector_;
    std::vector<cv::Point2f> p_pixel_vector_;
    
    std::fstream fs;
    std::string name_str_; 
    int optimization_mode_;
    
 public:
    FrameExtraction();
    void getConfigFileParam();  // 读取ros中的参数
    void callBack(const sensor_msgs::ImageConstPtr msg_img, const sensor_msgs::PointCloud2ConstPtr msg_cloud);
    
    // 点云直通滤波
    void passThroughFilter(const pcl::PointCloud<PointT>::Ptr& cloud_in, pcl::PointCloud<PointT>::Ptr& cloud_out);
    void getFilterParam(lidar2cam_calibration::filter_Config& config, uint32_t level);
    // 保存当前帧数据
    void saveCurrentFrame(const cv::Mat& img, const pcl::PointCloud<PointT>::Ptr& cloud_in, const cv::Point3d& lidar_center);

    // 图像坐标获取
    void imageOperation();
    bool extractBoardCenter(const pcl::PointCloud<PointT>::Ptr& cloud_in, pcl::PointCloud<PointT>::Ptr& cloud_out);
};


} // namespace lidar2cam_calibration

#endif
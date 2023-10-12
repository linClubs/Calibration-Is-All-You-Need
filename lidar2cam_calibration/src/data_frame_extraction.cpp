#include "data_frame_extraction.h"


namespace lidar2cam_calibration
{

FrameExtraction::FrameExtraction()
{   
    getConfigFileParam();
    // std::cout << pkg_path_ + "/data/data.txt" << std::endl;
    fs.open(pkg_path_ + "/data/data.txt", std::ios::out);//输入你想写入的内容 
    fs  << "# timeStamp, p_pixel_, p_cam_center, lidar_center. (size = 9)" << std::endl;

    cloud_ptr_.reset(new pcl::PointCloud<PointT>);
    cloud_filter_ptr_.reset(new pcl::PointCloud<PointT>);
    
    msg_cloud_.reset(new sensor_msgs::PointCloud2);
    msg_cloud_filter_.reset(new sensor_msgs::PointCloud2);
    
    image_process_.reset(new ImageProcession);

    pub_img_ = n_.advertise<sensor_msgs::Image>(topic_img_new_, 10);
    pub_cloud_ = n_.advertise<sensor_msgs::PointCloud2>(topic_cloud_new_, 10);
    pub_cloud_filter_ = n_.advertise<sensor_msgs::PointCloud2>(topic_cloud_filter_, 10);

    sub_img_.subscribe(n_, topic_img_, 10);
    sub_cloud_.subscribe(n_, topic_cloud_, 10);
    sync_.reset( new Sync_(MySyncPolicy_(10), sub_img_, sub_cloud_) );
    sync_->registerCallback(boost::bind(&FrameExtraction::callBack, this, _1, _2));

    // 绑定服务端的回调函数
    f_ = boost::bind(&FrameExtraction::getFilterParam, this, _1, _2);
    server_.setCallback(f_);
    
    ROS_INFO("Waiting to publish data of image and cloud ...\n");
}   

void FrameExtraction::getConfigFileParam()
{

    n_.getParam("pkg_path", pkg_path_);
    n_.param<std::string>("pkg_path", pkg_path_, "/home/lin/ros_code/calibration_ws/src/lidar2cam_calibration");
    
    n_.getParam("topic_img", topic_img_);
    n_.param<std::string>("topic_img", topic_img_, "/camera/color/image_raw");

    n_.getParam("topic_cloud", topic_cloud_);
    n_.param<std::string>("topic_cloud", topic_cloud_, "/velodyne_points");

    n_.getParam("topic_img_new", topic_img_new_);
    n_.param<std::string>("topic_img_new", topic_img_new_, "/image_raw");
    
    n_.getParam("topic_cloud_seg", topic_cloud_new_);
    n_.param<std::string>("topic_cloud_seg", topic_cloud_new_, "/points_seg");
    
    n_.getParam("topic_cloud_filter", topic_cloud_filter_);
    n_.param<std::string>("topic_cloud_filter", topic_cloud_filter_, "/points_filter");

    n_.getParam("frame_id", frame_id_);
    n_.param<std::string>("frame_id", frame_id_, "velodyne");

    n_.getParam("optimization_mode", optimization_mode_);
    n_.param<int>("optimization_mode", optimization_mode_, 0);
    
    std::cout << "\n-------------------" << std::endl;
    std::cout << "pkg_path: "<< pkg_path_ << std::endl;
    std::cout << "topic_img: "<< topic_img_ << std::endl;
    std::cout << "topic_cloud: "<< topic_cloud_ << std::endl;
    std::cout << "topic_img_new: "<< topic_img_new_ << std::endl;
    std::cout << "topic_cloud_new: "<< topic_cloud_new_ << std::endl;
    std::cout << "topic_cloud_filter: "<< topic_cloud_filter_ << std::endl;
    std::cout << "data_save_path: "<< pkg_path_ + "/data/data.txt" << std::endl;
    std::cout << "frame_id: "<< frame_id_ << std::endl;
    std::cout << "optimization_mode: "<< optimization_mode_ << std::endl;
}

void FrameExtraction::callBack(const sensor_msgs::ImageConstPtr msg_img, const sensor_msgs::PointCloud2ConstPtr msg_cloud)
{
    src_ = cv_bridge::toCvShare(msg_img, "bgr8")->image;
    img_new_ = src_.clone();
    
    // 图像操作函数获取板子中心在相机坐标系下的坐标，以及像素坐标,便于pnp和icp
    imageOperation();  // 并返回图像坐标系下，中心点,左上，右上，右下，左下,储存在类成员p_cam_vector_里面（注意更新,写在定义里面）

    // 打印p_cam_boarc_center中心点的相机坐标系下坐标
    // std::cout << "p_cam_board_center:\n" << p_cam_vector_[0] << std::endl;

    // 点云msg转pcl
    pcl::fromROSMsg(*msg_cloud, *cloud_ptr_);
    // 直通滤波,根据ros动态参数
    passThroughFilter(cloud_ptr_, cloud_filter_ptr_);

    // 定义投影点云变量
    pcl::PointCloud<PointT>::Ptr cloud_projected(new pcl::PointCloud<PointT>());
   
    // cv::Point3d lidar_normal;
    // lidar_normal = extractBoard(cloud_filter_ptr_, cloud_projected);
    // std::cout << "lidar_normal:\n"<< lidar_normal << std::endl;

    // 提出标定板子点云中心
    bool is_success = extractBoardCenter(cloud_filter_ptr_, cloud_projected);

    // std::cout << "p_cam_center:\n"<< p_cam_vector_[0] << std::endl;
    
    PointT cloud_min, cloud_max;
    pcl::getMinMax3D(*cloud_projected, cloud_min, cloud_max);  // 获取点云中最小最大xyz

    // std::cout << "min: " << cloud_min.x << "\t" << cloud_min.y << "\t" << cloud_min.z << std::endl;
    // std::cout << "max: " << cloud_max.x << "\t" << cloud_max.y << "\t" << cloud_max.z << std::endl;

    cv::Point3d lidar_center((cloud_max.x + cloud_min.x) / 2., (cloud_max.y + cloud_min.y) / 2., (cloud_max.z + cloud_min.z) / 2.);

    // Eigen::Vector4f centroid;  //质心 所有点合取平均
    // pcl::compute3DCentroid(*cloud_projected, centroid); // 计算质心
    // std::cout << "centroid: " << centroid << std::endl;

    pcl::toROSMsg(*cloud_projected, *msg_cloud_);
    msg_cloud_->header.stamp = msg_cloud->header.stamp;
	msg_cloud_->header.frame_id = frame_id_;
    pub_cloud_.publish(msg_cloud_);

    pcl::toROSMsg(*cloud_filter_ptr_, *msg_cloud_filter_);
    msg_cloud_filter_->header.stamp = msg_cloud->header.stamp;
	msg_cloud_filter_->header.frame_id = frame_id_;
    pub_cloud_filter_.publish(msg_cloud_filter_);

    msg_img_ = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_new_).toImageMsg();
    pub_img_.publish(msg_img_);
    
    cv::imshow("view", img_new_);
    char k = cv::waitKey(30);
    if( k == 's')   // 按q退出 保存数据
    {
        if(is_success && image_process_->flag_)
        {   
            // --------save data-----------------
            // name_str_ = std::to_string(ros::Time::now().toNSec());
            name_str_ = std::to_string(msg_img->header.stamp.toNSec());
            saveCurrentFrame(src_, cloud_filter_ptr_,lidar_center);
            ROS_WARN("Save image and cloud data successfully...\n");
        }
        else
        {
            ROS_WARN("The number of detection calibration aruco less than 3...\n");
        }
    }
    if(k == 'q')  // 按q退出27
    {   
        fs.close();
        cv::destroyAllWindows();
        ROS_INFO("The program exit...\n");
        exit(-1);
    }
}

void FrameExtraction::passThroughFilter(const pcl::PointCloud<PointT>::Ptr& cloud_in, pcl::PointCloud<PointT>::Ptr& cloud_out)
{   
    pcl::PointCloud<PointT> cloud_x, cloud_y;
    pcl::PassThrough<PointT> pass_x;  // 创建直通滤波器对象
    pass_x.setInputCloud(cloud_in);            // 设置输入对象,传地址
    pass_x.setFilterFieldName("x");             // 设置过滤时所需要点云类型的Z字段
    pass_x.setFilterLimits(box.x_min, box.x_max);   
    pass_x.filter(*cloud_in);     
    
    pcl::PassThrough<PointT> pass_y; 
    pass_y.setInputCloud(cloud_in);  
    pass_y.setFilterFieldName("y");             // 设置过滤时所需要点云类型的Z字段
    pass_y.setFilterLimits(box.y_min, box.y_max);   
    pass_y.filter(*cloud_in);

    pcl::PassThrough<PointT> pass_z; 
    pass_z.setInputCloud(cloud_in);  
    pass_z.setFilterFieldName("z");             // 设置过滤时所需要点云类型的Z字段
    pass_z.setFilterLimits(box.z_min, box.z_max);           // 设置过滤字段的范围
    pass_z.filter(*cloud_out); 
}

void FrameExtraction::getFilterParam(lidar2cam_calibration::filter_Config& config, uint32_t level)
{   
    box.x_max = config.x_max;
    box.x_min = config.x_min;
    box.y_max = config.y_max;
    box.y_min = config.y_min;
    box.z_max = config.z_max;
    box.z_min = config.z_min;

    ROS_INFO("Reconfigure Request: x_min=%f, x_max=%f, y_min=%f, y_max=%f, z_min=%f, z_max=%f",
        box.x_min,
        box.x_max,
        box.y_min,
        box.y_max,
        box.z_min,
        box.z_max);
}

void FrameExtraction::saveCurrentFrame(const cv::Mat& img, const pcl::PointCloud<PointT>::Ptr& cloud_in, const cv::Point3d& lidar_center)
{   
         // 保存数据
    fs  << std::to_string(ros::Time::now().toNSec()) << " "
        << p_pixel_vector_[0].x << " " << p_pixel_vector_[0].y<< " " 
        << p_cam_vector_[0].x << " " << p_cam_vector_[0].y << " " << p_cam_vector_[0].z << " "
        << lidar_center.x << " " << lidar_center.y << " " << lidar_center.z << std::endl;
        
    cv::imwrite(pkg_path_ + "/data/image/" + name_str_ + ".png", img);
    pcl::io::savePCDFileASCII(pkg_path_ + "/data/cloud/" + name_str_ + ".pcd", *cloud_in);
}

void FrameExtraction::imageOperation()
{
    image_process_->getImage(img_new_);
    image_process_->detectMarkers();
    if(image_process_->flag_)
    {
        p_cam_vector_ = image_process_->getBoardCenter3d(); // 计算标定板上的坐标系，并画点
        img_new_ = image_process_->img_;
        p_pixel_vector_ = image_process_->p_pixel_vector_;
    }
    // else
    //     img_new_ = src_;
}

bool FrameExtraction::extractBoardCenter(const pcl::PointCloud<PointT>::Ptr& cloud_in, pcl::PointCloud<PointT>::Ptr& cloud_out)
{
    // Fit a plane through the board point cloud
    // Inliers give the indices of the points that are within the RANSAC threshold
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);  // 
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(0.004);
    // pcl::ExtractIndices<PointT> extract;
    seg.setInputCloud(cloud_in);
    seg.segment(*inliers, *coefficients);  // 返回模型

    // Check that segmentation succeeded
    pcl::PointCloud<PointT>::Ptr cloud_projected(new pcl::PointCloud<PointT>);
    if (coefficients->values.size() < 3)
    {
        std::cout << "\t--Chessboard plane segmentation failed..." << std::endl;
        cv::Point3d null_normal;
        return false;
    }

    // Plane normal vector magnitude 平面的法向量
    cv::Point3d lidar_normal(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
    // 归一化
    lidar_normal /= -cv::norm(lidar_normal);  // Normalise and flip the direction

    // 根据平面的法向量，将点云投到平面上
    // Project the inliers on the fitted plane
    // When it freezes the chessboard after capture, what you see are the inlier points (filtered from the original)
    pcl::ProjectInliers<PointT> proj;
    proj.setModelType(pcl::SACMODEL_PLANE);
    proj.setInputCloud(cloud_in);
    proj.setModelCoefficients(coefficients);
    proj.filter(*cloud_out);

    return true;

}





} //namespace lidar2cam_calibration
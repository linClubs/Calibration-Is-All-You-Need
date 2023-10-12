#include "estimation_tf.h"
#include "ceres_qt.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "estimation_tf_node");
    ros::NodeHandle n;
    // std::ofstream outf;
    std::string s, pkg_path;
    n.getParam("pkg_path", pkg_path);
    n.param<std::string>("pkg_path", pkg_path, "/home/lin/ros_code/calibration_ws/src/lidar2cam_calibration");
    std::string data_path = pkg_path + "/data/data.txt";

    std::string save_path = pkg_path + "/data/result.txt";
    
    // 打开保存的文件,往文件中写入
    std::fstream fs(save_path.c_str(), std::ios::out);

    // 创建一个点集,用来保存坐标点集合
    std::shared_ptr<PointSet> point_set = std::make_shared<PointSet>();
    
    // 读取data.txt的点坐标
    bool is_read = readPoint(data_path.c_str(), point_set);

    if(!is_read)
    {
        // std::cout << point_set->p_cam_vector_.size() << std::endl;
        std::cout << "\n--Read point is failure..." << std::endl;
        return -1;
    }

    // 写入保存的文件的第一排说明
    fs << "# ----lidar2cam-T_cl(RPY)----\n\n" << std::endl;

    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    Eigen::Vector3d t = Eigen::Vector3d::Zero();
    Eigen::Vector3d euler = Eigen::Vector3d::Zero();
    Eigen::Quaterniond q = Eigen::Quaterniond::Identity();

    // SVD求解
    estimation_3d3d_SVD(point_set->p_cam_vector_, point_set->p_lidar_vector_, R, t);
    rotationMatrix2eulerAngles(R, euler);
    
    std::cout << "\n \033[1;32m ----->SVD 3d3d: \033[0m";
    std::cout << "\neulerAngles(RPY): " << euler.transpose();
    std::cout << "\ttranslation: " << t.transpose() << std::endl;
    // std::cout << "\033[1;32m -------------------- \n \033[0m" << std::endl;
    fs << "# ------SVD-3d3d------\n# eulerAngles(RPY):\n" << euler.transpose() << "\n# translation:\n" << t << std::endl;
    
    // solvePNP_cv需要修改该函数的内参变量的值
    cv::Mat rvec, tvec, rvec_mat;
    R = Eigen::Matrix3d::Identity();
    euler = Eigen::Vector3d::Zero(); 
    solvePNP_cv(point_set->pts_uv_cv_, point_set->pts_lidar_cv_, rvec, tvec);
    
    cv::Rodrigues(rvec, rvec_mat);
    cv::cv2eigen(rvec_mat, R);
    rotationMatrix2eulerAngles(R, euler);

    std::cout << "\n \033[1;32m -----> Opencv-PNP 3d2d: \033[0m";
    std::cout << "\n eulerAngles(RPY): " << euler.transpose();
    std::cout << "\t translation: \n" << tvec << std::endl;
    // std::cout << "\033[1;32m -------------------- \n \033[0m" << std::endl;
    fs << "\n#------Opencv-PNP-3d2d------\n# eulerAngles(RPY):\n" << euler.transpose() << "\n# translation:\n" << tvec << std::endl;

    // 3d3d的ceres优化求解
    R = Eigen::Matrix3d::Identity();
    t = Eigen::Vector3d::Zero();
    euler = Eigen::Vector3d::Zero();

    optimation_3d3d(point_set->p_cam_vector_, point_set->p_lidar_vector_, R, t);
    rotationMatrix2eulerAngles(R, euler);

    std::cout << "\033[1;32m -----> Ceres-() 3d3d: \033[0m";
    std::cout << "\neulerAngles(RPY): " << euler.transpose();
    std::cout << "\ttranslation: " << t.transpose() << std::endl;
    // std::cout << "\033[1;32m -------------------- \n \033[0m" << std::endl;
    fs << "\n#------Ceres-3d3d------\n# eulerAngles(RPY):\n" << euler.transpose() << "\n# translation:\n" << t.transpose() << std::endl;

    // ceres四元素求解3d-3d
    t = Eigen::Vector3d::Zero();
    q = Eigen::Quaterniond::Identity();
    euler = Eigen::Vector3d::Zero();
    
    lidar2camICP(point_set->p_lidar_vector_, point_set->p_cam_vector_, q, t );
    Eigen::Matrix3d R_cl(q);
    euler = q.matrix().eulerAngles(0,1,2);
    euler = euler * 180 / M_PI;

    std::cout << "\n \033[1;32m -----> Ceres-q_t-3d3d: \033[0m";
    std::cout<< "\n eulerAngles(RPY): " << euler.transpose();
    std::cout << "\ttranslation:" <<  t.transpose() << std::endl;
    // std::cout << "\033[1;32m -------------------- \n \033[0m" << std::endl;
    fs << "\n#------ceres-q_t-3d3d:------\n# eulerAngles(RPY):\n" << euler.transpose() 
        << "\n# translation:\n" << t.transpose()  << std::endl;

    
    // ceres四元素求解3d-2d
    q = Eigen::Quaterniond::Identity();
    t = Eigen::Vector3d::Zero();
    euler = Eigen::Vector3d::Zero();
    Eigen::Matrix3d K;
    K << 617.1, 0, 324.1, 0, 617.1, 242.4, 0, 0, 1;

    lidar2cam_2d3d(point_set->p_lidar_vector_, point_set->p_uv_vector_,K, q, t);
    euler = q.matrix().eulerAngles(0,1,2);
    euler = euler * 180 / M_PI;

    std::cout << "\n \033[1;32m -----> Ceres-q_t-3d2d: \033[0m";
    std::cout<< "\n eulerAngles(RPY): " << euler.transpose();
    std::cout << "\ttranslation: " <<  t.transpose() << std::endl;
    // std::cout << "\033[1;32m -------------------- \n \033[0m" << std::endl;    
    fs << "\n#------ceres-q_t-3d2d:------\n# eulerAngles(RPY):\n" << euler.transpose() 
        << "\n# translation:\n" << t.transpose() << std::endl;

    fs.close();
    std::cout << "\033[1;32m----->Save result in " + save_path << "\033[0m \n" << std::endl;
    return 0;     

}
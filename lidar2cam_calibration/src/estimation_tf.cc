#include "estimation_tf.h"


bool readPoint(const std::string &path, std::shared_ptr<PointSet> point_set) 
{
    std::ifstream fin(path);
    std::string str;
    
    if (!fin.is_open()) {
        std::cerr << " --data.txt " << path << " not found." << std::endl;
        return false;
    }

    while (!fin.eof()) {
        std::getline(fin, str); 
        if(str[0] == '#')
        {
            continue;
        } 
        double time, u, v, x_c, y_c, z_c, x_l, y_l, z_l;
        
        fin >> time >> u >> v >> x_c >> y_c >> z_c >> x_l >> y_l >> z_l;
        
        Eigen::Vector2d p_uv( Eigen::Vector2d(u, v) );
        Eigen::Vector3d p_cam( Eigen::Vector3d(x_c, y_c, z_c) );
        Eigen::Vector3d p_lidar( Eigen::Vector3d(x_l, y_l, z_l) );
        
        cv::Point2f p_uv_cv(u,v);
        cv::Point3f p_lidar_cv(x_l, y_l, z_l);
        
        // Sophus::SE3d p1(Eigen::Quaterniond(qw, qx, qy, qz), Eigen::Vector3d(tx, ty, tz));
        
        // std::cout << "----------\n";
        // std::cout << p_uv << std::endl;
        // std::cout << p_cam << std::endl;
        // std::cout << p_lidar << std::endl;

        // std::cout << p_uv_cv << std::endl;

        // std::cout << p_lidar << std::endl;
        // std::cout << p_lidar_cv << std::endl;
        // std::cout << "----------\n";

        // eigen存储点
        point_set->p_uv_vector_.emplace_back(p_uv);
        point_set->p_cam_vector_.emplace_back(p_cam);
        point_set->p_lidar_vector_.emplace_back(p_lidar);

        // opencv存储点,使用solvePNP求解
        point_set->pts_uv_cv_.emplace_back(p_uv_cv);
        point_set->pts_lidar_cv_.emplace_back(p_lidar_cv);
    }
    return true;
}

void estimation_3d3d_SVD(const std::vector<Eigen::Vector3d> &pt_cam, 
                            const std::vector<Eigen::Vector3d> &pt_lidar,
                            Eigen::Matrix3d& R, Eigen::Vector3d& t)
{
    Eigen::Vector3d p1(0,0,0), p2(0,0,0);  // 使用前记得初始化

    int N = pt_cam.size();
    for (int i = 0; i < N; ++i)
    {
        p1 += pt_cam[i];
        p2 += pt_lidar[i];
    }
    
    p1 = Eigen::Vector3d(p1 / N);  // 均值
    p2 = Eigen::Vector3d(p2 / N);  // 均值
    
    std::vector<Eigen::Vector3d> q1(N), q2(N);

    for (int i = 0; i < N; ++i)
    {
        q1[i] = pt_cam[i] - p1;
        q2[i] = pt_lidar[i] - p2;     
    }

    // compute q1*q2^T
    Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
    for(int i = 0; i < N; ++i)
    {
        W += Eigen::Vector3d(q1[i][0], q1[i][1], q1[i][2]) * Eigen::Vector3d(q2[i][0],q2[i][1], q2[i][2]).transpose();
    }

    // std::cout << "W = " << std::endl;
    // SVD on W
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();

    // std::cout << "U=" << U << std::endl;
    // std::cout << "V=" << V << std::endl;

    R = U * (V.transpose());
    if (R.determinant() < 0) {
        R = -R;
    }

    t = Eigen::Vector3d(p1[0], p1[1], p1[2]) - R * Eigen::Vector3d(p2[0], p2[1], p2[2]);
    // // convert to cv::Mat
    // R = (cv::Mat_<double>(3, 3) <<
    //     R_(0, 0), R_(0, 1), R_(0, 2),
    //     R_(1, 0), R_(1, 1), R_(1, 2),
    //     R_(2, 0), R_(2, 1), R_(2, 2)
    // );
    // t = (cv::Mat_<double>(3, 1) << t_(0, 0), t_(1, 0), t_(2, 0));

    // std::cout << "R:\n" << R << std::endl;
    // std::cout << "t:\n" << t << std::endl;
}

void optimation_3d3d(const std::vector<Eigen::Vector3d> &p_cam, 
                            const std::vector<Eigen::Vector3d> &p_lidar,
                            Eigen::Matrix3d& R, Eigen::Vector3d& t)
{   
    double rotation_vector[3],tranf[3];//旋转向量r，平移t
    
    cv::Mat R_vector; // 优化时采用的旋转向量更新方向
    eigen2cv_vector(R, R_vector);
   
    rotation_vector[0] = R_vector.at<double>(0, 0);
    rotation_vector[1] = R_vector.at<double>(1, 0);
    rotation_vector[2] = R_vector.at<double>(2, 0);
    
    tranf[0] = t[0];
    tranf[1] = t[1];
    tranf[2] = t[2];

    ceres::Problem problem;
    ceres::LossFunction *loss_function = new ceres::HuberLoss(1.0);

    for(int i = 0; i < p_lidar.size(); ++i )
    {
        ceres::CostFunction* costfusion = 
            new ceres::AutoDiffCostFunction<TfCostFunctor, 3, 3, 3>(new TfCostFunctor(p_lidar[i], p_cam[i]));
            // problem.AddResidualBlock(costfusion, nullptr, rotation_vector, tranf);
            problem.AddResidualBlock(costfusion, loss_function, rotation_vector, tranf);
    }

    //配置求解器
    ceres::Solver::Options option;
    option.linear_solver_type=ceres::DENSE_QR;//DENSE_SCHUR
    
    //true:迭代信息输出到屏幕.false:不输出
    option.minimizer_progress_to_stdout= false; 

    ceres::Solver::Summary summary;//优化信息
    
    //开始优化
    ceres::Solve(option,&problem,&summary);
    
    // 输出结果
    // std::cout << summary.BriefReport() << "\n";
    
    cv::Mat cv_R_vector = (cv::Mat_<float>(3, 1) << rotation_vector[0], rotation_vector[1], rotation_vector[2]);
    
    cv::Mat R_mat;
    cv::Rodrigues(cv_R_vector, R_mat);
    cv::cv2eigen(R_mat, R);

    t[0] = tranf[0];
    t[1] = tranf[1];
    t[2] = tranf[2];
}

void optimation_2d3d(const std::vector<Eigen::Vector2d> &p_uv, 
                            const std::vector<Eigen::Vector3d> &p_lidar,
                            Eigen::Matrix3d& R, Eigen::Vector3d& t)
{

}

void solvePNP_cv(const std::vector<cv::Point2f>& p_uv,
                    const std::vector<cv::Point3f>& p_lidar,
                    cv::Mat& rvec, cv::Mat& tvec)
{   
    cv::Mat cameraMatrix = (cv::Mat_<float>(3, 3) << 617.201, 0., 324.637,
                                               0, 617.362, 242.462,
                                               0, 0., 1);
    cv::Mat distCoeffs = (cv::Mat_<float>(5, 1) << 0, 0, 0, 0, 0);  // 初始化加（）

    // cv::solvePnP(p_lidar, p_uv, cameraMatrix, distCoeffs, rvec, tvec);
    cv::solvePnPRansac(p_lidar, p_uv, cameraMatrix, distCoeffs, rvec, tvec);
}

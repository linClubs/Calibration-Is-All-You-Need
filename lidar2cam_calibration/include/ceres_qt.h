/*
构建代价函数(cost function)或残差(residual)
构建优化问题(ceres::Problem)：通过 AddResidualBlock 添加代价函数(cost function)、
                            损失函数(loss function核函数) 和 待优化状态量
配置求解器(ceres::Solver::Options)
运行求解器(ceres::Solve(options, &problem, &summary))
*/

#ifndef ceres_qt_h
#define ceres_qt_h

#include <Eigen/Dense>
#include <ceres/ceres.h>
#include <ceres/rotation.h>

struct EdgeFoctor
{   
    // 定义2个变量接收雷达点和相机的点
    const Eigen::Vector3d p_lidar_, p_cam_;

    // 初始化函数,给点赋值
    EdgeFoctor(Eigen::Vector3d p_lidar, Eigen::Vector3d p_cam): p_lidar_(p_lidar) ,p_cam_(p_cam){}
    
    // 重载()运算符，构建残差
    template <typename T> 
    bool operator()(const T* q, const T* t, T* residual) const
    {    
        // 1 定义2个点坐标
        // Eigen::Matrix<T, 3, 1> pl{T(p_lidar_(0)), T(p_lidar_(1)), T(p_lidar_(2))};
        // Eigen::Matrix<T, 3, 1> pc{T(p_cam_(0)), T(p_cam_(1)), T(p_cam_(2))};
        
        // 取分量转换Vector3d可以用(0), [0], .x();
        Eigen::Matrix<T, 3, 1> pl{T(p_lidar_(0)), T(p_lidar_.y()), T(p_lidar_[2])};
        // 强制转换
        Eigen::Matrix<T, 3, 1> pc(p_cam_.cast<T>());

        // 2 模板类定义旋转和平移，并赋值problem.AddResidualBlock传进来的优化变量值
        Eigen::Quaternion<T> q_{q[3], q[0], q[1], q[2]};
        Eigen::Matrix<T, 3, 1> t_{t[0], t[1], t[2]};

        // 3 定义一个中间量，点坐标，将雷达点用qt变换得到新的坐标点
        Eigen::Matrix<T, 3, 1> pl_;
        pl_ = q_ * pl + t_;  // 变换雷达点

        // 4 误差 = 新点-相机点，residual是个模版，可以将误差写一起，也可以分开
        residual[0] = pl_(0) - pc(0);
        residual[1] = pl_(1) - pc(1);
        residual[2] = pl_(2) - pc(2);

        // 向量v的平方2范数为v.dot(v)或v.squaredNorm()，可以用来表示2点的距离
        // 只要要一个残差需要修改problem.AddResidualBlock中残差的维度
        // residual[0] = (pl_-pc).squaredNorm();
        // residual[0] = (pl_-pc).dot(pl_-pc); 

        // 5 必须有true返回
        return true;  
    }
};


void lidar2camICP(const std::vector<Eigen::Vector3d> &p_lidar_vector, 
                const std::vector<Eigen::Vector3d> &p_cam_vector,
                Eigen::Quaterniond &q, Eigen::Vector3d &t)
{

    // 0 优化变量初始 定义成doble类型，四元素和平移的初值,四元素用eigen初始化时(w,x,y,z)
    double para_q[4]{0, 0, 0, 1};
    double para_t[3]{0 ,0, 0};

    // 1 优化问题构建
    // loss_function鲁棒核函数
    ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1); 
    
    // 重构LocalParameterization类，ceres提供了四元素类型，否则需要重写
    ceres::LocalParameterization *q_parameterization =
    new ceres::EigenQuaternionParameterization();
    
    // 定义优化问题
    ceres::Problem::Options problem_options;
    ceres::Problem problem(problem_options);

    // 显式添加优化变量， 一般情况可以隐式添加（省略不写）
    problem.AddParameterBlock(para_q, 4, q_parameterization);
    problem.AddParameterBlock(para_t, 3);


    // 2 构建残差函数
    for (auto i = 0; i < p_lidar_vector.size(); ++i)
    {
        problem.AddResidualBlock(new ceres::AutoDiffCostFunction<EdgeFoctor,3,4,3>
        (new EdgeFoctor(p_lidar_vector[i],p_cam_vector[i])),loss_function, para_q, para_t);
    }
    

    // 3 定义优化器
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    //迭代数
    options.max_num_iterations = 100;
    
    //进度是否发到STDOUT，是否打印
    options.minimizer_progress_to_stdout = false;
    
    ceres::Solver::Summary summary;
    
    // 4 求解优化器
    ceres::Solve(options, &problem, &summary);

     // 输出结果
    // std::cout << summary.BriefReport() << "\n";

    // para_q与para_t会自动更新
    q = Eigen::Quaterniond(para_q[3], para_q[0],para_q[1], para_q[2]);
    t = Eigen::Vector3d(para_t[0], para_t[1], para_t[2]);

}


struct EdgeFoctor2d3d
{   
    // 定义2个变量接收雷达点和相机的点
    const Eigen::Vector3d p_lidar_;
    const Eigen::Vector2d uv_cam_;
    const Eigen::Matrix3d cam_intrinsics_;
    // 初始化函数,给点赋值
    EdgeFoctor2d3d(Eigen::Vector3d p_lidar, Eigen::Vector2d uv_cam, const Eigen::Matrix3d K)
        : p_lidar_(p_lidar) ,uv_cam_(uv_cam),cam_intrinsics_(K) {}
    
    // 重载()运算符，构建残差
    template <typename T> 
    bool operator()(const T* q, const T* t, T* residual) const
    {    
       
        // 取分量转换Vector3d可以用(0), [0], .x();
        Eigen::Matrix<T, 3, 1> pl{T(p_lidar_(0)), T(p_lidar_.y()), T(p_lidar_[2])};
        // 强制转换
        Eigen::Matrix<T, 2, 1> pc(uv_cam_.cast<T>());
        Eigen::Matrix<T, 3, 3> K_(cam_intrinsics_.cast<T>());


        // 2 模板类定义旋转和平移，并赋值problem.AddResidualBlock传进来的优化变量值
        Eigen::Quaternion<T> q_{q[3], q[0], q[1], q[2]};
        Eigen::Matrix<T, 3, 1> t_{t[0], t[1], t[2]};
        
        const T& fx = K_(0, 0);
        const T& fy = K_(1, 1);
        const T& cx = K_(0, 2);
        const T& cy = K_(1, 2);


        // 3 定义一个中间量，点坐标，将雷达点用qt变换得到新的坐标点
        Eigen::Matrix<T, 3, 1> pl_;
        pl_ = q_ * pl + t_;  // 变换雷达点到相机坐标系
        pl_ = pl_ / pl_(2);  // 归一化坐标到


        // 4 误差 = 新点-相机点，residual是个模版，可以将误差写一起，也可以分开
        residual[0] = pl_(0) - (pc(0)-cx) / fx;
        residual[1] = pl_(1) - (pc(1)-cy) / fy;


        // 5 必须有true返回
        return true;  
    }
};


void lidar2cam_2d3d(const std::vector<Eigen::Vector3d> &p_lidar_vector, 
                const std::vector<Eigen::Vector2d> &uv_cam_vector,
                const Eigen::Matrix3d &K,
                Eigen::Quaterniond &q, Eigen::Vector3d &t)
{

    // 0 优化变量初始 定义成doble类型，四元素和平移的初值,四元素用eigen初始化时(w,x,y,z)
    double para_q[4]{0, 0, 0, 1};
    double para_t[3]{0 ,0, 0};

    // 1 优化问题构建
    // loss_function鲁棒核函数
    ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1); 
    
    // 重构LocalParameterization类，ceres提供了四元素类型，否则需要重写
    ceres::LocalParameterization *q_parameterization =
    new ceres::EigenQuaternionParameterization();
    
    // 定义优化问题
    ceres::Problem::Options problem_options;
    ceres::Problem problem(problem_options);

    // 显式添加优化变量， 一般情况可以隐式添加（省略不写）
    problem.AddParameterBlock(para_q, 4, q_parameterization);
    problem.AddParameterBlock(para_t, 3);
    // problem.AddParameterBlock(para_cam_intrinsics, 3);

    // 2 构建残差函数
    for (auto i = 0; i < p_lidar_vector.size(); ++i)
    {
        problem.AddResidualBlock(new ceres::AutoDiffCostFunction<EdgeFoctor2d3d, 2, 4, 3>
        (new EdgeFoctor2d3d(p_lidar_vector[i], uv_cam_vector[i], K)), loss_function, para_q, para_t);
    }
    

    // 3 定义优化器
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    //迭代数
    options.max_num_iterations = 1000;
    
    //进度是否发到STDOUT，是否打印
    options.minimizer_progress_to_stdout = false;
    
    ceres::Solver::Summary summary;
    
    // 4 求解优化器
    ceres::Solve(options, &problem, &summary);

     // 输出结果
    // std::cout << summary.BriefReport() << "\n";  

    // para_q与para_t会自动更新
    q = Eigen::Quaterniond(para_q[3], para_q[0],para_q[1], para_q[2]);
    t = Eigen::Vector3d(para_t[0], para_t[1], para_t[2]);
    
    // std::cout << q.coeffs().transpose() << std::endl;
    // std::cout << t.transpose() << std::endl;
    // std::cout << para_cam_intrinsics[0] << " " << para_cam_intrinsics[1] << " " << para_cam_intrinsics[2] << std::endl;

}


#endif
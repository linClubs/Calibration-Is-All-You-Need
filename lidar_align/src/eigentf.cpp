#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>

int main()
{   


    Eigen::Vector3d A = Eigen::Vector3d(2, 0, 0);
    Eigen::Vector3d A_norm = A.normalized();
    // A.norm();
    
    std::cout  << "A.norm(): " << A.norm() << std::endl;
    std::cout  << "A.normalized(): " << A_norm.transpose() << std::endl;
    
    Eigen::Vector3d euler(114, 0, 0);
    euler = euler * M_PI / 180;

    Eigen::Matrix3d rotation_matrix;
    rotation_matrix = Eigen::AngleAxisd(euler[0], Eigen::Vector3d::UnitZ()) * 
                       Eigen::AngleAxisd(euler[1], Eigen::Vector3d::UnitY()) * 
                       Eigen::AngleAxisd(euler[2], Eigen::Vector3d::UnitX());

    
    Eigen::Vector3d euler2 = rotation_matrix.eulerAngles(2,1,0); 
    euler2 = euler2 * 180 / M_PI;

    std::cout << rotation_matrix << std::endl;
    std::cout << euler2.transpose() << std::endl;
    
    return 0;

}
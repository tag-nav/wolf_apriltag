
//std
#include <iostream>

//Eigen
#include <eigen3/Eigen/Geometry>

//Wolf
#include "core/common/wolf.h"

int main()
{
    using namespace wolf;

    std::cout << std::endl << "Eigen Quatenrnion test" << std::endl;
    
    Scalar q1[4]; 
    Eigen::Map<Eigen::Quaternions> q1_map(q1);
    
    //try to find out how eigen sorts storage (real part tail or head ? )
    q1_map.w() = 1; 
    q1_map.x() = 2; 
    q1_map.y() = 3; 
    q1_map.z() = 4; 
    std::cout << "q1[0]=" << q1[0] << "; q1_map.x()=" << q1_map.x() << std::endl;
    std::cout << "q1[1]=" << q1[1] << "; q1_map.y()=" << q1_map.y() << std::endl;
    std::cout << "q1[2]=" << q1[2] << "; q1_map.z()=" << q1_map.z() << std::endl;
    std::cout << "q1[3]=" << q1[3] << "; q1_map.w()=" << q1_map.w() << std::endl;
    std::cout << std::endl << "RESULT: Eigen stores REAL part in the LAST memory position of the quaternion." << std::endl;
    
    std::cout << std::endl << "End of Eigen Quatenrnion test" << std::endl;
    return 0;
}


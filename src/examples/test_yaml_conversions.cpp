/**
 * \file test_yaml_conversions.cpp
 *
 *  Created on: May 15, 2016
 *      \author: jsola
 */

#include "core/yaml/yaml_conversion.h"

#include <yaml-cpp/yaml.h>

#include <eigen3/Eigen/Dense>

#include <iostream>
//#include <fstream>

int main()
{

    using namespace Eigen;

    std::cout << "\nTrying different yaml specs for matrix..." << std::endl;

    YAML::Node mat_sized_23, mat_sized_33, mat_sized_bad, mat_23, mat_33, mat_bad;

    mat_sized_23    = YAML::Load("[[2, 3] ,[1, 2, 3, 4, 5, 6] ]"); // insensitive to spacing
    mat_sized_33    = YAML::Load("[[3, 3] ,[1, 2, 3, 4, 5, 6, 7, 8, 9]]"); // insensitive to spacing

    mat_23      = YAML::Load("[1, 2, 3, 4, 5, 6]"); // insensitive to spacing
    mat_33      = YAML::Load("[1, 2, 3, 4, 5, 6, 7, 8, 9]"); // insensitive to spacing

    MatrixXd Mx = mat_sized_23.as<MatrixXd>();
    std::cout << "Dyn-Dyn [[2,3] ,[1, 2, 3, 4, 5, 6] ] = \n" << Mx << std::endl;

    Matrix<double, 2, Dynamic> M2D = mat_sized_23.as<Matrix<double, 2, Dynamic>>();
    std::cout << "2-Dyn [[2,3] ,[1, 2, 3, 4, 5, 6] ] = \n" << M2D << std::endl;

    Matrix<double, Dynamic, 3> MD3 = mat_sized_23.as<Matrix<double, Dynamic, 3>>();
    std::cout << "Dyn-3 [[2,3] ,[1, 2, 3, 4, 5, 6] ] = \n" << MD3 << std::endl;

    Matrix3d M3 = mat_sized_33.as<Matrix3d>();
    std::cout << "3-3   [[3,3] ,[1, 2, 3, 4, 5, 6, 7, 8, 9] ] = \n" << M3 << std::endl;

    M2D = mat_23.as<Matrix<double, 2, Dynamic>>();
    std::cout << "2-Dyn [1, 2, 3, 4, 5, 6] = \n" << M2D << std::endl;

    MD3 = mat_23.as<Matrix<double, Dynamic, 3>>();
    std::cout << "Dyn-3 [1, 2, 3, 4, 5, 6] = \n" << MD3 << std::endl;

    M3 = mat_33.as<Matrix3d>();
    std::cout << "3-3   [1, 2, 3, 4, 5, 6, 7, 8, 9] = \n" << M3 << std::endl;

    return 0;
}

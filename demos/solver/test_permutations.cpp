/*
 * test_permutations.cpp
 *
 *  Created on: Jun 15, 2015
 *      Author: jvallve
 */

//std includes
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <memory>
#include <random>
#include <typeinfo>
#include <ctime>
#include <queue>

// eigen includes
#include <eigen3/Eigen/OrderingMethods>

using namespace Eigen;

//main
int main(int argc, char *argv[])
{
    PermutationMatrix<Dynamic, Dynamic, int> P1(5), P2(5), P3(5), P4(5);
    P1.setIdentity();
    P2.setIdentity();
    P3.setIdentity();

    VectorXd a = VectorXd::LinSpaced(5,1,5);
    MatrixXd A= a.asDiagonal();
    SparseMatrix<double> B = A.sparseView();
    B.makeCompressed();

    std::cout << "A (dense)" << std::endl << A << std::endl << std::endl;
    std::cout << "B (sparse)" << std::endl << B << std::endl << std::endl;

    P1.indices()(3) = 4;
    P1.indices()(4) = 3;

    std::cout << "Permutation 1" << std::endl << P1.indices().transpose() << std::endl << std::endl;

    P2.indices()(0) = 4;
    P2.indices()(4) = 0;

    std::cout << "Permutation 2" << std::endl << P2.indices().transpose() << std::endl << std::endl;

    std::cout << "Pre-multiplying: Permutating rows" << std::endl;
    std::cout << "P1 * A" << std::endl << P1 * A << std::endl << std::endl;
    std::cout << "P1 * B" << std::endl << P1 * B << std::endl << std::endl;
    SparseMatrix<double> C = (P1 * B).sparseView();
    std::cout << "(P1 * B).bottomRows(1)" << std::endl << C.bottomRows(1) << std::endl << std::endl;

    std::cout << "Post-multiplying: Permutating cols" << std::endl;
    std::cout << "A * P1.transpose()" << std::endl << A  * P1.transpose()<< std::endl << std::endl;
    std::cout << "B * P1.transpose()" << std::endl << B  * P1.transpose()<< std::endl << std::endl;

    std::cout << "Pre&post-multiplying:" << std::endl;
    std::cout << "P1 * A * P1.transpose()" << std::endl << P1 * A * P1.transpose() << std::endl << std::endl;
    std::cout << "P2 * P1 * A * P1.transpose() * P2.transpose()" << std::endl << P2 * P1 * A * P1.transpose() * P2.transpose() << std::endl << std::endl;
    std::cout << "P1 * P2 * A * P2.transpose() * P1.transpose()" << std::endl << P1 * P2 * A * P2.transpose() * P1.transpose() << std::endl << std::endl;

    P3 = P1 * P2;

    std::cout << "Permutation P3 = P1 * P2" << std::endl << P3.indices().transpose() << std::endl << std::endl;
    std::cout << "P3 * A * P3.transpose()" << std::endl << P3 * A  * P3.transpose() << std::endl << std::endl;

    std::cout << "PERMUTATING INDICES" << std::endl;
    ArrayXi acc_permutations(5);
    acc_permutations << 0,1,2,3,4;

    std::cout << "acc_permutations: " << acc_permutations.transpose() << std::endl;

    std::cout << "P1: " << P1.indices().transpose() << std::endl;
    std::cout << "P1 * acc_permutations: " << (P1 * acc_permutations.matrix()).transpose() << std::endl;
    std::cout << "P1.inverse() * acc_permutations: " << (P1.inverse() * acc_permutations.matrix()).transpose() << std::endl;

    std::cout << "P2: " << P2.indices().transpose() << std::endl;
    std::cout << "P2 * (P1 * acc_permutations): " << (P2 * (P1 * acc_permutations.matrix())).transpose() << std::endl;
    std::cout << "(P2 * P1).inverse() * acc_permutations): " << ((P2 * P1).inverse() * acc_permutations.matrix()).transpose() << std::endl;
    P4 = P1 * P2 * P3;
    std::cout << "Permutation P4 = P1 * P2 * P3:   " << P4.indices().transpose() << std::endl;
    std::cout << "P3 * (P2 * (P1 * acc_permutations)): " << (P3 * (P2 * (P1 * acc_permutations.matrix()))).transpose() << std::endl;
    std::cout << "accumulated permutations can not be stored in vectors..." << std::endl;

    std::cout << std::endl << "PARTIALL PERMUTATIONS" << std::endl;
    PermutationMatrix<Dynamic, Dynamic, int> P5(2);
    P5.indices()(0) = 1;
    P5.indices()(1) = 0;
    std::cout << "P5 (equivalent to P1 but partiall): " << std::endl << P5.indices().transpose() << std::endl;
    std::cout << "P2: " << P2.indices().transpose() << std::endl << std::endl;
    std::cout << "A * P2.transpose(): " << std::endl << A * P2.transpose() << std::endl << std::endl;
    std::cout << "(A * P2.transpose()).rightCols(2) * P5.transpose(): " << std::endl << (A * P2.transpose()).rightCols(2) * P5.transpose() << std::endl << std::endl;
    PermutationMatrix<Dynamic, Dynamic, int> P6 = P2;
    P6.indices().tail(2) = P5 * P6.indices().tail(2);
    std::cout << "P6 = P2, P6.indices().tail(2) = P5 * P6.indices().tail(2): " << P6.indices().transpose() << std::endl << std::endl;
    std::cout << "A * P6.transpose(): " << std::endl << A * P6.transpose() << std::endl << std::endl;
    std::cout << "(P1 * P2): " << std::endl << (P1 * P2).indices().transpose() << std::endl << std::endl;
    std::cout << "A * (P1 * P2).transpose(): " << std::endl << A * (P1 * P2).transpose() << std::endl << std::endl;
    std::cout << "Partiall permutations can not be accumulated, for doing that they should be full size" << std::endl;

    std::cout << std::endl << "PERMUTATION OF MAPPED VECTORS" << std::endl;
    std::cout << "a = " << a.transpose() << std::endl << std::endl;
    Map<VectorXd> mapped_a(a.data(), 1);
    std::cout << "mapped_a1 = " << mapped_a << std::endl << std::endl;
    a = P2 * a;
    std::cout << "a = P2 * a: " << std::endl << a.transpose() << std::endl << std::endl;
    std::cout << "mapped_a = " << mapped_a.transpose() << std::endl << std::endl;
    std::cout << "maps are affected of the reorderings in mapped vectors" << std::endl;

//    Map<>
}

/*
 * test_ccolamd.cpp
 *
 *  Created on: Jun 11, 2015
 *      Author: jvallve
 */

// Wolf includes
#include "base/common/wolf.h"

//std includes
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <memory>
#include <random>
#include <typeinfo>
#include <ctime>
#include <queue>

// ccolamd
#include "solver/ccolamd_ordering.h"

// eigen includes
#include <eigen3/Eigen/OrderingMethods>
#include <eigen3/Eigen/CholmodSupport>
#include <eigen3/Eigen/SparseLU>

using namespace Eigen;
using namespace wolf;

//main
int main(int argc, char *argv[])
{
    if (argc != 2 || atoi(argv[1]) < 1)
    {
        std::cout << "Please call me with: [./test_ccolamd SIZE], where:" << std::endl;
        std::cout << "     - SIZE: integer size of the problem" << std::endl;
        std::cout << "EXIT due to bad user input" << std::endl << std::endl;
        return -1;
    }
    SizeEigen size = atoi(argv[1]);

    SparseMatrix<double, ColMajor, SizeEigen> A(size, size), Aordered(size, size);
    CholmodSupernodalLLT < SparseMatrix<double, ColMajor, SizeEigen> > solver;
    PermutationMatrix<Dynamic, Dynamic, SizeEigen> perm(size);
    CCOLAMDOrdering<SizeEigen> ordering;
    Matrix<SizeEigen, Dynamic, 1> ordering_factors = Matrix<SizeEigen, Dynamic, 1>::Ones(size);
    VectorXd b(size), bordered(size), xordered(size), x(size);
    clock_t t1, t2, t3;
    double time1, time2, time3;

    // BUILD THE PROBLEM ----------------------------
    //Fill A & b
    A.insert(0, 0) = 5;
    b(0) = 1;
    for (int i = 1; i < size; i++)
    {
        A.insert(i, i) = 5;
        A.insert(i, i - 1) = 1;
        A.insert(i - 1, i) = 1;
        b(i) = i + 1;
    }
    A.insert(size - 1, 0) = 2;
    A.insert(0, size - 1) = 2;

    std::cout << "Solving Ax = b:" << std::endl << "A = " << std::endl << A << std::endl << std::endl;
    std::cout << "b = " << std::endl << b.transpose() << std::endl << std::endl;

    // SOLVING WITHOUT REORDERING ------------------------------------
    // solve Ax = b
    t1 = clock();
    solver.compute(A);
    if (solver.info() != Success)
    {
        std::cout << "decomposition failed" << std::endl;
        return 0;
    }
    x = solver.solve(b);
    time1 = ((double) clock() - t1) / CLOCKS_PER_SEC;
    std::cout << "solved in " << time1 << "seconds" << std::endl;
    std::cout << "x = " << x.transpose() << std::endl;

    // SOLVING AFTER REORDERING ------------------------------------
    // ordering factors
    ordering_factors(size-1) = 2;
    ordering_factors(0) = 2;

    // ordering
    t2 = clock();
    A.makeCompressed();

    std::cout << "Reordering using CCOLAMD:" << std::endl;
    std::cout << "ordering_factors = " << std::endl << ordering_factors.transpose() << std::endl << std::endl;
    ordering(A, perm, ordering_factors.data());
    std::cout << "perm = " << std::endl << perm.indices().transpose() << std::endl << std::endl;

    bordered = perm * b;
    Aordered = A.twistedBy(perm);
    std::cout << "reordered A = " << std::endl << Aordered * MatrixXd::Identity(size, size) << std::endl << std::endl;
    std::cout << "reordered b = " << std::endl << bordered.transpose() << std::endl << std::endl;

    // solve Ax = b
    solver.compute(Aordered);
    if (solver.info() != Success)
    {
        std::cout << "decomposition failed" << std::endl;
        return 0;
    }
    xordered = solver.solve(bordered);
    time2 = ((double) clock() - t2) / CLOCKS_PER_SEC;
    std::cout << "solved in " << time2 << "seconds" << std::endl;
    std::cout << "x = " << (perm.inverse() * xordered).transpose() << std::endl;
    std::cout << "x = " << x.transpose() << " (solution without reordering)" << std::endl;

    // SOLVING AND REORDERING ------------------------------------
    t3 = clock();
    SparseLU<SparseMatrix<double, ColMajor, SizeEigen>, CCOLAMDOrdering<SizeEigen> > solver2;
    solver2.compute(A);
    if (solver2.info() != Success)
    {
        std::cout << "decomposition failed" << std::endl;
        return 0;
    }
    x = solver2.solve(b);
    time3 = ((double) clock() - t3) / CLOCKS_PER_SEC;
    std::cout << "solved in " << time3 << "seconds" << std::endl;
    std::cout << "x = " << x.transpose() << std::endl;
}


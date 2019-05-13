/*
 * test_ccolamd_blocks.cpp
 *
 *  Created on: Jun 12, 2015
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
#include <eigen3/Eigen/CholmodSupport>
#include <eigen3/Eigen/SparseLU>

// ccolamd
#include "solver/ccolamd_ordering.h"

using namespace Eigen;

void eraseSparseBlock(SparseMatrix<double>& original, const unsigned int& row, const unsigned int& Nrows, const unsigned int& col, const unsigned int& Ncols)
{
  for (unsigned int i = row; i < row + Nrows; i++)
    for (unsigned int j = col; j < row + Ncols; j++)
      original.coeffRef(i,j) = 0.0;

  original.makeCompressed();
}

void addSparseBlock(const MatrixXd& ins, SparseMatrix<double>& original, const unsigned int& row, const unsigned int& col)
{
  for (unsigned int r=0; r<ins.rows(); ++r)
    for (unsigned int c = 0; c < ins.cols(); c++)
      original.coeffRef(r + row, c + col) += ins(r,c);
}

void permutation_2_block_permutation(const PermutationMatrix<Dynamic, Dynamic, int> &perm, PermutationMatrix<Dynamic, Dynamic, int> &perm_blocks, const int dim, const int size)
{
    ArrayXXi idx(dim, size);
    idx.row(0) = dim * perm.indices().transpose();
    for (int i = 1; i<dim; i++)
        idx.row(i) = idx.row(i-1) + 1;
    Map<ArrayXi> idx_blocks(idx.data(), dim*size, 1);
    perm_blocks.indices() = idx_blocks;
}

//main
int main(int argc, char *argv[])
{
    if (argc != 3 || atoi(argv[1]) < 1|| atoi(argv[2]) < 1)
    {
        std::cout << "Please call me with: [./test_ccolamd SIZE DIM], where:" << std::endl;
        std::cout << "     - SIZE: integer size of the problem" << std::endl;
        std::cout << "     - DIM: integer dimension of the nodes" << std::endl;
        std::cout << "EXIT due to bad user input" << std::endl << std::endl;
        return -1;
    }
    int size = atoi(argv[1]);
    int dim = atoi(argv[2]);

    SparseMatrix<double> H(size * dim, size * dim), H_ordered(size * dim, size * dim), H_block_ordered(size * dim, size * dim);
    SparseMatrix<int> FactorMatrix(size,size);
    CholmodSupernodalLLT < SparseMatrix<double> > solver, solver2, solver3;
    PermutationMatrix<Dynamic, Dynamic, int> perm(size), perm_blocks(size * dim);
    CCOLAMDOrdering<int> ordering;
    VectorXi block_ordering_factors = VectorXi::Ones(size);
    VectorXi ordering_factors = VectorXi::Ones(size*dim);
    VectorXd b(size * dim), b_ordered(size * dim), b_block_ordered(size * dim), x_block_ordered(size * dim), x_ordered(size * dim), x(size * dim);
    clock_t t1, t2, t3;
    double time1, time2, time3;

    MatrixXd omega = MatrixXd::Constant(dim, dim, 0.1) + MatrixXd::Identity(dim, dim);

    // BUILD THE PROBLEM ----------------------------
    //Fill H & b
    for (int i = 0; i < size; i++)
    {
        addSparseBlock(5*omega, H, i*dim, i*dim);
        FactorMatrix.insert(i,i) = 1;
        if (i > 0)
        {
            addSparseBlock(omega, H, i*dim, (i-1)*dim);
            addSparseBlock(omega, H, (i-1)*dim, i*dim);
            FactorMatrix.insert(i,i-1) = 1;
            FactorMatrix.insert(i-1,i) = 1;
        }
        b.segment(i*dim, dim) = VectorXd::Constant(dim, i+1);
    }
    addSparseBlock(2*omega, H, 0, (size - 1)*dim);
    addSparseBlock(2*omega, H, (size-1)*dim, 0);
    FactorMatrix.insert(0,size-1) = 1;
    FactorMatrix.insert(size-1,0) = 1;

    std::cout << "Solving factor graph:" << std::endl;
    std::cout << "Factors: " << std::endl << FactorMatrix * MatrixXi::Identity(size,size) << std::endl << std::endl;

    // SOLVING WITHOUT REORDERING ------------------------------------
    // solve Hx = b
    t1 = clock();
    solver.compute(H);
    if (solver.info() != Success)
    {
        std::cout << "decomposition failed" << std::endl;
        return 0;
    }
    x = solver.solve(b);
    time1 = ((double) clock() - t1) / CLOCKS_PER_SEC;

    // SOLVING AFTER REORDERING ------------------------------------
    // ordering factors
    ordering_factors.segment(dim * (size-1), dim) = VectorXi::Constant(dim,2);
    ordering_factors.segment(0, dim) = VectorXi::Constant(dim,2);

    // variable ordering
    t2 = clock();
    H.makeCompressed();

    std::cout << "Reordering using CCOLAMD:" << std::endl;
    std::cout << "ordering_factors = " << std::endl << ordering_factors.transpose() << std::endl << std::endl;
    ordering(H, perm, ordering_factors.data());

    b_ordered = perm * b;
    H_ordered = H.twistedBy(perm);

    // solve Hx = b
    solver2.compute(H_ordered);
    if (solver2.info() != Success)
    {
        std::cout << "decomposition failed" << std::endl;
        return 0;
    }
    x_ordered = solver2.solve(b_ordered);
    x_ordered = perm.inverse() * x_ordered;
    time2 = ((double) clock() - t2) / CLOCKS_PER_SEC;

    // SOLVING AFTER BLOCK REORDERING ------------------------------------
    // ordering factors
    block_ordering_factors(size-1) = 2;
    block_ordering_factors(0) = 2;

    // block ordering
    t3 = clock();
    FactorMatrix.makeCompressed();

    std::cout << "Reordering using Block CCOLAMD:" << std::endl;
    std::cout << "block_ordering_factors = " << std::endl << block_ordering_factors.transpose() << std::endl << std::endl;
    ordering(FactorMatrix, perm_blocks, block_ordering_factors.data());

    // variable ordering
    permutation_2_block_permutation(perm_blocks, perm , dim, size);
    b_block_ordered = perm * b;
    H_block_ordered = H.twistedBy(perm);

    // solve Hx = b
    solver3.compute(H_block_ordered);
    if (solver3.info() != Success)
    {
        std::cout << "decomposition failed" << std::endl;
        return 0;
    }
    x_block_ordered = solver3.solve(b_block_ordered);
    x_block_ordered = perm.inverse() * x_block_ordered;
    time3 = ((double) clock() - t3) / CLOCKS_PER_SEC;

    // RESULTS ------------------------------------
    std::cout << "NO REORDERING:    solved in " << time1*1e3 << " ms" << std::endl;
    std::cout << "REORDERING:       solved in " << time2*1e3 << " ms" << std::endl;
    std::cout << "BLOCK REORDERING: solved in " << time3*1e3 << " ms" << std::endl;
    //std::cout << "x = " << x.transpose() << std::endl;
    //std::cout << "x = " << x_ordered.transpose() << std::endl;
    //std::cout << "x = " << x_block_ordered.transpose() << std::endl;
}


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
          if (ins(r,c) != 0)
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

    // Problem variables
    //CholmodSupernodalLLT < SparseMatrix<double> > solver, solver2, solver3;
    SparseLU < SparseMatrix<double>, NaturalOrdering<int> > solver, solver2, solver3;
    MatrixXd omega = MatrixXd::Constant(dim, dim, 0.1) + MatrixXd::Identity(dim, dim);
    SparseMatrix<double> H(dim,dim),
                         H_ordered(dim,dim),
                         H_b_ordered(dim,dim);
    VectorXd b(dim),
             b_ordered(dim),
             b_b_ordered(dim),
             x_b_ordered(dim),
             x_ordered(dim),
             x(dim);

    // ordering variables
    SparseMatrix<int> factors(1,1), factors_ordered(1,1);
    ArrayXi acc_permutation(dim),
            acc_permutation_b(dim),
            acc_permutation_factors(1);
    acc_permutation = ArrayXi::LinSpaced(dim,0,dim-1);
    acc_permutation_b = acc_permutation;
    acc_permutation_factors(0) = 0;

    CCOLAMDOrdering<int> ordering;
    VectorXi factor_ordering_factors(1);
    VectorXi ordering_factors(1);

    // results variables
    clock_t t1, t2, t3;
    double time1=0, time2=0, time3=0;

    // INITIAL STATE
    addSparseBlock(5*omega, H, 0, 0);
    factors.insert(0,0) = 1;
    b.head(dim) = VectorXd::LinSpaced(Sequential, dim, 0, dim-1);

    std::cout << "STARTING INCREMENTAL TEST" << std::endl << std::endl;

    // INCREMENTAL LOOP
    for (int i = 1; i < size; i++)
    {
        std::cout << "========================= STEP " << i << ":" << std::endl;
        // AUGMENT THE PROBLEM ----------------------------
        H.conservativeResize((i+1)*dim,(i+1)*dim);
        H_ordered.conservativeResize((i+1)*dim,(i+1)*dim);
        H_b_ordered.conservativeResize((i+1)*dim,(i+1)*dim);
        b.conservativeResize((i+1)*dim);
        b_ordered.conservativeResize((i+1)*dim);
        b_b_ordered.conservativeResize((i+1)*dim);
        x.conservativeResize((i+1)*dim);
        x_ordered.conservativeResize((i+1)*dim);
        x_b_ordered.conservativeResize((i+1)*dim);
        factors.conservativeResize(i+1, i+1);

        // Odometry
        addSparseBlock(5*omega, H, i*dim, i*dim);
        addSparseBlock(omega, H, i*dim, (i-1)*dim);
        addSparseBlock(omega, H, (i-1)*dim, i*dim);
        factors.insert(i,i) = 1;
        factors.insert(i,i-1) = 1;
        factors.insert(i-1,i) = 1;

        // Loop Closure
        if (i == size-1)
        {
            addSparseBlock(2*omega, H, 0, i*dim);
            addSparseBlock(2*omega, H, i*dim, 0);
            factors.insert(0,i) = 1;
            factors.insert(i,0) = 1;
        }

        // r.h.v
        b.segment(i*dim, dim) = VectorXd::LinSpaced(Sequential, dim, dim*i, dim *(i+1)-1);

        std::cout << "Solving factor graph:" << std::endl;
        std::cout << "Factors: " << std::endl << factors * MatrixXi::Identity((i+1), (i+1)) << std::endl << std::endl;
//        std::cout << "H: " << std::endl << H * MatrixXd::Identity(dim*(i+1), dim*(i+1)) << std::endl << std::endl;

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
        time1 += ((double) clock() - t1) / CLOCKS_PER_SEC;

        // SOLVING WITH REORDERING ------------------------------------
        // Order with previous orderings
        acc_permutation.conservativeResize(dim*(i+1));
        acc_permutation.tail(dim) = ArrayXi::LinSpaced(dim,dim*i,dim*(i+1)-1);
        PermutationMatrix<Dynamic, Dynamic, int> acc_permutation_matrix(dim*(i+1));
        acc_permutation_matrix.indices() = acc_permutation;
        b_ordered = acc_permutation_matrix * b;
        H_ordered = H.twistedBy(acc_permutation_matrix);

        // ordering factors
        ordering_factors.resize(dim*(i+1));
        ordering_factors = ((H_ordered.rightCols(3) * MatrixXd::Ones(3,1)).array() == 0).select(VectorXi::Zero(dim*(i+1)),VectorXi::Ones(dim*(i+1)));

        // variable ordering
        t2 = clock();
        H_ordered.makeCompressed();

        PermutationMatrix<Dynamic, Dynamic, int> permutation_matrix(dim*(i+1));
        ordering(H_ordered, permutation_matrix, ordering_factors.data());

        // applying ordering
        acc_permutation_matrix = permutation_matrix * acc_permutation_matrix;
        acc_permutation = acc_permutation_matrix.indices();
        b_ordered = permutation_matrix * b_ordered;
        H_ordered = H_ordered.twistedBy(permutation_matrix);

        // solve Hx = b
        solver2.compute(H_ordered);
        if (solver2.info() != Success)
        {
            std::cout << "decomposition failed" << std::endl;
            return 0;
        }
        x_ordered = solver2.solve(b_ordered);
        x_ordered = acc_permutation_matrix.inverse() * x_ordered;
        time2 += ((double) clock() - t2) / CLOCKS_PER_SEC;

        // SOLVING WITH BLOCK REORDERING ------------------------------------
        // Order with previous orderings
        acc_permutation_b.conservativeResize(dim*(i+1));
        acc_permutation_b.tail(dim) = ArrayXi::LinSpaced(dim,dim*i,dim*(i+1)-1);
        PermutationMatrix<Dynamic, Dynamic, int> acc_permutation_b_matrix(dim*(i+1));
        acc_permutation_b_matrix.indices() = acc_permutation_b;
        b_b_ordered = acc_permutation_b_matrix * b;
        H_b_ordered = H.twistedBy(acc_permutation_b_matrix);

        acc_permutation_factors.conservativeResize(i+1);
        acc_permutation_factors(i) = i;
        PermutationMatrix<Dynamic, Dynamic, int> acc_permutation_factors_matrix(dim*(i+1));
        acc_permutation_factors_matrix.indices() = acc_permutation_factors;
        factors_ordered = factors.twistedBy(acc_permutation_factors_matrix);

        // ordering factors
        factor_ordering_factors.resize(i);
        factor_ordering_factors = factors_ordered.rightCols(1);

        // block ordering
        t3 = clock();
        factors_ordered.makeCompressed();

        PermutationMatrix<Dynamic, Dynamic, int> permutation_factors_matrix(i+1);
        ordering(factors_ordered, permutation_factors_matrix, factor_ordering_factors.data());

        // applying ordering
        permutation_2_block_permutation(permutation_factors_matrix, permutation_matrix , dim, i+1);
        acc_permutation_factors_matrix = permutation_factors_matrix * acc_permutation_factors_matrix;
        acc_permutation_factors = acc_permutation_factors_matrix.indices();
        acc_permutation_b_matrix = permutation_matrix * acc_permutation_b_matrix;
        acc_permutation_b = acc_permutation_b_matrix.indices();
        b_b_ordered = permutation_matrix * b_b_ordered;
        H_b_ordered = H_b_ordered.twistedBy(permutation_matrix);

        // solve Hx = b
        solver3.compute(H_b_ordered);
        if (solver3.info() != Success)
        {
            std::cout << "decomposition failed" << std::endl;
            return 0;
        }
        x_b_ordered = solver3.solve(b_b_ordered);
        x_b_ordered = acc_permutation_b_matrix.inverse() * x_b_ordered;
        time3 += ((double) clock() - t3) / CLOCKS_PER_SEC;

        // RESULTS ------------------------------------
        std::cout << "========================= RESULTS " << i << ":" << std::endl;
        std::cout << "NO REORDERING:    solved in " << time1*1e3 << " ms" << std::endl;
        std::cout << "REORDERING:       solved in " << time2*1e3 << " ms" << std::endl;
        std::cout << "BLOCK REORDERING: solved in " << time3*1e3 << " ms" << std::endl;
        std::cout << "x1 = " << x.transpose() << std::endl;
        std::cout << "x2 = " << x_ordered.transpose() << std::endl;
        std::cout << "x3 = " << x_b_ordered.transpose() << std::endl;
    }

    // RESULTS ------------------------------------
    std::cout << "NO REORDERING:    solved in " << time1*1e3 << " ms" << std::endl;
    std::cout << "REORDERING:       solved in " << time2*1e3 << " ms" << std::endl;
    std::cout << "BLOCK REORDERING: solved in " << time3*1e3 << " ms" << std::endl;

        //std::cout << "x = " << x.transpose() << std::endl;
        //std::cout << "x = " << x_ordered.transpose() << std::endl;
        //std::cout << "x = " << x_b_ordered.transpose() << std::endl;
}


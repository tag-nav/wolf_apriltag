/*
 * test_iQR.cpp
 *
 *  Created on: Jun 17, 2015
 *      Author: jvallve
 */

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
#include <eigen3/Eigen/SparseQR>
#include <eigen3/Eigen/SPQRSupport>

// ccolamd
#include "solver/ccolamd_ordering.h"

using namespace Eigen;

class block_pruning
{
    public:
        int col, row, Nrows, Ncols;
        block_pruning(int _col, int _row, int _Nrows, int _Ncols) :
                col(_col),
                row(_row),
                Nrows(_Nrows),
                Ncols(_Ncols)
        {
            //
        }
        bool operator()(int i, int j, double) const
        {
            return (i < row || i > row + Nrows-1) || (j < col || j > col + Ncols-1);
        }
};

void eraseSparseBlock(SparseMatrix<double>& original, const unsigned int& row, const unsigned int& col, const unsigned int& Nrows, const unsigned int& Ncols)
{
    // prune all non-zero elements that not satisfy the 'keep' operand
    // elements that are not in the block rows or are not in the block columns should be kept
    //original.prune([](int i, int j, double) { return (i < row || i > row + Nrows-1) || (j < col || j > col + Ncols-1); });

    block_pruning bp(row, col, Nrows, Ncols);
    original.prune(bp);

//  for (unsigned int i = row; i < row + Nrows; i++)
//    for (unsigned int j = col; j < row + Ncols; j++)
//      original.coeffRef(i,j) = 0.0;
//
//  original.prune(0);
}

void addSparseBlock(const MatrixXd& ins, SparseMatrix<double>& original, const unsigned int& row, const unsigned int& col)
{
  for (unsigned int r=0; r<ins.rows(); ++r)
      for (unsigned int c = 0; c < ins.cols(); c++)
          if (ins(r,c) != 0)
              original.coeffRef(r + row, c + col) += ins(r,c);
}

void permutation_2_block_permutation(const PermutationMatrix<Dynamic, Dynamic, int> &perm_nodes, PermutationMatrix<Dynamic, Dynamic, int> &perm_variables, const int dim)
{
    ArrayXXi idx(dim, perm_nodes.indices().rows());
    idx.row(0) = dim * perm_nodes.indices().transpose();

    for (int i = 1; i<dim; i++)
        idx.row(i) = idx.row(i-1) + 1;
    Map<ArrayXi> idx_blocks(idx.data(), dim*perm_nodes.indices().rows(), 1);
    perm_variables.indices() = idx_blocks;
}

void augment_permutation(PermutationMatrix<Dynamic, Dynamic, int> &perm, const int new_size)
{
    int old_size = perm.indices().size();
    int dim = new_size - old_size;
    VectorXi new_indices(new_size);
    new_indices.head(old_size)= perm.indices();
    new_indices.tail(dim) = ArrayXi::LinSpaced(dim, old_size, new_size-1);
    perm.resize(new_size);
    perm.indices() = new_indices;
}

//main
int main(int argc, char *argv[])
{
    if (argc != 3 || atoi(argv[1]) < 1|| atoi(argv[2]) < 1)
    {
        std::cout << "Please call me with: [./test_iQR SIZE DIM], where:" << std::endl;
        std::cout << "     - SIZE: integer size of the problem" << std::endl;
        std::cout << "     - DIM: integer dimension of the nodes" << std::endl;
        std::cout << "EXIT due to bad user input" << std::endl << std::endl;
        return -1;
    }
    int size = atoi(argv[1]);
    int dim = atoi(argv[2]);

    // Problem variables
    SparseQR < SparseMatrix<double>, NaturalOrdering<int>> solver_ordered, solver_unordered, solver_ordered_partial;

    MatrixXd omega = MatrixXd::Constant(dim, dim, 0.1) + MatrixXd::Identity(dim, dim);
    SparseMatrix<double> A(0,0),
                         A_ordered(0,0),
                         R(0,0);
    VectorXd b,
             x,
             x_ordered,
             x_ordered_partial;
    int n_measurements = 0;
    int n_nodes = 0;

    // ordering variables
    SparseMatrix<int> A_nodes_ordered(0,0);
    PermutationMatrix<Dynamic, Dynamic, int> acc_permutation_nodes_matrix(0);

    CCOLAMDOrdering<int> ordering, partial_ordering;
    VectorXi nodes_ordering_factors;

    // results variables
    clock_t t_ordering, t_solving_ordered_full, t_solving_unordered, t_solving_ordered_partial, t4;
    double time_ordering=0, time_solving_unordered=0, time_solving_ordered=0, time_solving_ordered_partial=0;

    std::cout << "STARTING INCREMENTAL QR TEST" << std::endl << std::endl;

    // GENERATING MEASUREMENTS
    std::vector<std::vector<int>> measurements;
    for (int i = 0; i < size; i++)
    {
        std::vector<int> meas(0);
        if (i == 0) //prior
        {
            meas.push_back(0);
            measurements.push_back(meas);
            meas.clear();
        }
        else //odometry
        {
            meas.push_back(i-1);
            meas.push_back(i);
            measurements.push_back(meas);
            meas.clear();
        }
        if (i > size / 2) // loop closures
        {
            meas.push_back(0);
            meas.push_back(i);
            measurements.push_back(meas);
            meas.clear();
        }
    }

    // INCREMENTAL LOOP
    for (unsigned int i = 0; i < measurements.size(); i++)
    {
        std::cout << "========================= MEASUREMENT " << i << ":" << std::endl;
        std::vector<int> measurement = measurements.at(i);

        // AUGMENT THE PROBLEM ----------------------------
        n_measurements++;
        while (n_nodes < measurement.back()+1)
        {
            n_nodes++;
            // Resize accumulated permutations
            augment_permutation(acc_permutation_nodes_matrix, n_nodes);

            // Resize state
            x.conservativeResize(n_nodes*dim);
            x_ordered.conservativeResize(n_nodes*dim);
            x_ordered_partial.conservativeResize(n_nodes*dim);
        }
        A.conservativeResize(n_measurements*dim,n_nodes*dim);
        A_ordered.conservativeResize(n_measurements*dim,n_nodes*dim);
        R.conservativeResize(n_nodes*dim,n_nodes*dim);
        b.conservativeResize(n_measurements*dim);
        A_nodes_ordered.conservativeResize(n_measurements,n_nodes);

        // ADD MEASUREMENTS
        int min_ordered_node = n_nodes;
        for (unsigned int j = 0; j < measurement.size(); j++)
        {
            int ordered_node = acc_permutation_nodes_matrix.indices()(measurement.at(j));

            addSparseBlock(2*omega, A, A.rows()-dim, measurement.at(j) * dim);
            addSparseBlock(2*omega, A_ordered, A_ordered.rows()-dim, ordered_node * dim);

            A_nodes_ordered.coeffRef(A_nodes_ordered.rows()-1, ordered_node) = 1;

            b.segment(b.size() - dim, dim) = VectorXd::LinSpaced(dim, b.size()-dim, b.size()-1);
            // store minimum ordered node
            if (min_ordered_node > ordered_node)
                min_ordered_node = ordered_node;
        }

//        std::cout << "Solving Ax = b" << std::endl;
//        std::cout << "A_nodes_ordered: " << std::endl << MatrixXi::Identity(A_nodes_ordered.rows(), A_nodes_ordered.rows()) * A_nodes_ordered  << std::endl << std::endl;
//        std::cout << "A: " << std::endl << MatrixXd::Identity(A.rows(), A.rows()) * A  << std::endl << std::endl;
//        std::cout << "A_ordered: " << std::endl << MatrixXd::Identity(A.rows(), A.rows()) * A_ordered  << std::endl << std::endl;
//        std::cout << "b: " << std::endl << b.transpose() << std::endl << std::endl;

        // BLOCK REORDERING ------------------------------------
        t_ordering = clock();
        int ordered_nodes = n_nodes - min_ordered_node;
        int unordered_nodes = n_nodes - ordered_nodes;
        if (n_nodes > 1 && ordered_nodes > 2) // only reordering when involved nodes in the measurement are not the two last ones
        {
            // SUBPROBLEM ORDERING (from first node variable to last one)
            std::cout << "ordering partial problem: " << min_ordered_node << " to "<< n_nodes - 1 << std::endl;
            SparseMatrix<int> sub_A_nodes_ordered = A_nodes_ordered.rightCols(ordered_nodes);

            // partial ordering factors
            VectorXi nodes_partial_ordering_factors = sub_A_nodes_ordered.bottomRows(1).transpose();

            // computing nodes partial ordering
            A_nodes_ordered.makeCompressed();
            PermutationMatrix<Dynamic, Dynamic, int> partial_permutation_nodes_matrix(ordered_nodes);
            partial_ordering(sub_A_nodes_ordered, partial_permutation_nodes_matrix, nodes_partial_ordering_factors.data());

            // node ordering to variable ordering
            PermutationMatrix<Dynamic, Dynamic, int> partial_permutation_matrix(A_ordered.cols());
            permutation_2_block_permutation(partial_permutation_nodes_matrix, partial_permutation_matrix , dim);

            // apply partial orderings
            A_nodes_ordered.rightCols(ordered_nodes) = (A_nodes_ordered.rightCols(ordered_nodes) * partial_permutation_nodes_matrix.transpose()).sparseView();
            A_ordered.rightCols(ordered_nodes * dim) = (A_ordered.rightCols(ordered_nodes * dim) * partial_permutation_matrix.transpose()).sparseView();
            R.rightCols(ordered_nodes * dim) = (R.rightCols(ordered_nodes * dim) * partial_permutation_matrix.transpose()).sparseView();

            // ACCUMULATING PERMUTATIONS
            PermutationMatrix<Dynamic, Dynamic, int> permutation_nodes_matrix(VectorXi::LinSpaced(n_nodes, 0, n_nodes - 1)); // identity permutation
            permutation_nodes_matrix.indices().tail(ordered_nodes) = partial_permutation_nodes_matrix.indices() + VectorXi::Constant(ordered_nodes, n_nodes - ordered_nodes);
            acc_permutation_nodes_matrix = permutation_nodes_matrix * acc_permutation_nodes_matrix;
        }
        time_ordering += ((double) clock() - t_ordering) / CLOCKS_PER_SEC;
        // std::cout << "incrementally ordered A Block structure: " << std::endl << MatrixXi::Identity(A_nodes_ordered.rows(), A_nodes_ordered.rows()) * A_nodes_ordered  << std::endl << std::endl;
        //std::cout << "ordered A: " << std::endl << MatrixXd::Identity(A_ordered.rows(), A_ordered.rows()) * A_ordered << std::endl << std::endl;
        //std::cout << "b: " << std::endl << b.transpose() << std::endl << std::endl;

        // SOLVING
        // solving ordered subproblem
        t_solving_ordered_partial = clock();
        A_nodes_ordered.makeCompressed();
        A_ordered.makeCompressed();

        // finding measurements block
        SparseMatrix<int> measurements_to_initial = A_nodes_ordered.col(min_ordered_node);
//        std::cout << "measurements_to_initial " << measurements_to_initial << std::endl;
//        std::cout << "measurements_to_initial.innerIndex()[measurements_to_initial.outerIndex()[0]] " << measurements_to_initial.innerIndex()[measurements_to_initial.outerIndex()[0]] << std::endl;
        int initial_measurement = measurements_to_initial.innerIndex()[measurements_to_initial.outerIndex()[0]];

        SparseMatrix<double> A_ordered_partial = A_ordered.bottomRightCorner((n_nodes - initial_measurement) * dim, ordered_nodes * dim);
        solver_ordered_partial.compute(A_ordered_partial);
        if (solver_ordered_partial.info() != Success)
        {
            std::cout << "decomposition failed" << std::endl;
            return 0;
        }
        std::cout << "R new" << std::endl << MatrixXd::Identity(ordered_nodes * dim, ordered_nodes * dim) * solver_ordered_partial.matrixR() << std::endl;
        x_ordered_partial.tail(ordered_nodes * dim) = solver_ordered_partial.solve(b.tail(ordered_nodes * dim));
        std::cout << "x_ordered_partial.tail(ordered_nodes * dim)" << std::endl << x_ordered_partial.tail(ordered_nodes * dim).transpose() << std::endl;
        // store new part of R (equivalent to R.bottomRightCorner(ordered_nodes * dim, ordered_nodes * dim) = solver3.matrixR();)
        eraseSparseBlock(R, unordered_nodes * dim, unordered_nodes * dim, ordered_nodes * dim, ordered_nodes * dim);
        addSparseBlock(solver_ordered_partial.matrixR(), R, unordered_nodes * dim, unordered_nodes * dim);
        std::cout << "R" << std::endl << MatrixXd::Identity(R.rows(), R.rows()) * R << std::endl;
        R.makeCompressed();

        // solving not ordered subproblem
        if (unordered_nodes > 0)
        {
            std::cout << "--------------------- solving unordered part" << std::endl;
            SparseMatrix<double> R1 = R.topLeftCorner(unordered_nodes * dim, unordered_nodes * dim);
            std::cout << "R1" << std::endl << MatrixXd::Identity(R1.rows(), R1.rows()) * R1 << std::endl;
            SparseMatrix<double> R2 = R.topRightCorner(unordered_nodes * dim, ordered_nodes * dim);
            std::cout << "R2" << std::endl << MatrixXd::Identity(R2.rows(), R2.rows()) * R2 << std::endl;
            solver_ordered_partial.compute(R1);
            if (solver_ordered_partial.info() != Success)
            {
                std::cout << "decomposition failed" << std::endl;
                return 0;
            }
            x_ordered_partial.head(unordered_nodes * dim) = solver_ordered_partial.solve(b.head(unordered_nodes * dim) - R2 * x_ordered_partial.tail(ordered_nodes * dim));
        }
        // undo ordering
        PermutationMatrix<Dynamic, Dynamic, int> acc_permutation_matrix(A_ordered.cols());
        permutation_2_block_permutation(acc_permutation_nodes_matrix, acc_permutation_matrix , dim);
        x_ordered_partial = acc_permutation_matrix.inverse() * x_ordered_partial;
        time_solving_ordered_partial += ((double) clock() - t_solving_ordered_partial) / CLOCKS_PER_SEC;

        // SOLVING
        // full ordered problem
        t_solving_ordered_full = clock();
        A_nodes_ordered.makeCompressed();
        A_ordered.makeCompressed();
        solver_ordered.compute(A_ordered);
        if (solver_ordered.info() != Success)
        {
            std::cout << "decomposition failed" << std::endl;
            return 0;
        }
        x_ordered = solver_ordered.solve(b);
        std::cout << "solver_ordered.matrixR()" << std::endl << MatrixXd::Identity(A_ordered.cols(), A_ordered.cols()) * solver_ordered.matrixR() << std::endl;
        // undo ordering
        PermutationMatrix<Dynamic, Dynamic, int> acc_permutation_matrix2(A_ordered.cols());
        permutation_2_block_permutation(acc_permutation_nodes_matrix, acc_permutation_matrix2 , dim);
        x_ordered = acc_permutation_matrix.inverse() * x_ordered;
        time_solving_ordered += ((double) clock() - t_solving_ordered_full) / CLOCKS_PER_SEC;

        // WITHOUT ORDERING
        t_solving_unordered = clock();
        A.makeCompressed();
        solver_unordered.compute(A);
        if (solver_unordered.info() != Success)
        {
            std::cout << "decomposition failed" << std::endl;
            return 0;
        }
        //std::cout << "no ordering? " << solver_unordered.colsPermutation().indices().transpose() << std::endl;
        x = solver_unordered.solve(b);
        std::cout << "solver_unordered.matrixR()" << std::endl << MatrixXd::Identity(A.cols(), A.cols()) * solver_unordered.matrixR() << std::endl;
        time_solving_unordered += ((double) clock() - t_solving_unordered) / CLOCKS_PER_SEC;

        // RESULTS ------------------------------------
        std::cout << "========================= RESULTS " << i << ":" << std::endl;
        std::cout << "NO REORDERING:      solved in " << time_solving_unordered*1e3 << " ms | " << solver_unordered.matrixR().nonZeros() << " nonzeros in R"<< std::endl;
        std::cout << "BLOCK REORDERING:   solved in " << time_solving_ordered*1e3 << " ms | " << solver_ordered.matrixR().nonZeros() << " nonzeros in R"<< std::endl;
        std::cout << "BLOCK REORDERING 2: solved in " << time_solving_ordered_partial*1e3 << " ms | " << R.nonZeros() << " nonzeros in R"<< std::endl;

        std::cout << "x =                 " << x.transpose() << std::endl;
        std::cout << "x_ordered =         " << x_ordered.transpose() << std::endl;
        std::cout << "x_ordered_partial = " << x_ordered_partial.transpose() << std::endl;
        if ((x_ordered_partial-x_ordered).maxCoeff() < 1e-10)
            std::cout << "Both solutions are equals (tolerance " << (x_ordered_partial-x_ordered).maxCoeff() << ")" << std::endl;
        else
            std::cout << "DIFFERENT SOLUTIONS!!!!!!!! max difference " << (x_ordered_partial-x_ordered).maxCoeff() << std::endl;
    }
}


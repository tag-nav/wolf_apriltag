/*
 * test_iQR_wolf.cpp
 *
 *  Created on: Jun 17, 2015
 *      Author: jvallve
 */

//std includes
#include <cstdlib>
#include <string>
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
#include <Eigen/SPQRSupport>

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

void eraseSparseBlock(SparseMatrix<double, ColMajor>& original, const unsigned int& row, const unsigned int& col, const unsigned int& Nrows, const unsigned int& Ncols)
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

void addSparseBlock(const MatrixXd& ins, SparseMatrix<double, ColMajor>& original, const unsigned int& row, const unsigned int& col)
{
  for (unsigned int r=0; r<ins.rows(); ++r)
      for (unsigned int c = 0; c < ins.cols(); c++)
          if (ins(r,c) != 0)
              original.coeffRef(r + row, c + col) += ins(r,c);
}

struct node
{
    public:
        int id;
        int dim;
        int location;
        int order;

        node(const int _id, const int _dim, const int _location, const int _order) :
            id(_id),
            dim(_dim),
            location(_location),
            order(_order)
        {

        }
};

struct measurement
{
    public:
        std::vector<MatrixXd> jacobians;
        std::vector<int> nodes_idx;
        VectorXd error;
        int dim;
        bool odometry_type;
        int location;

        measurement(const MatrixXd & _jacobian1, const int _idx1, const VectorXd &_error, const int _meas_dim, bool _odometry_type=false) :
            jacobians({_jacobian1}),
            nodes_idx({_idx1}),
            error(_error),
            dim(_meas_dim),
            odometry_type(_odometry_type),
            location(0)
        {
            //jacobians.push_back(_jacobian1);
        }

        measurement(const MatrixXd & _jacobian1, const int _idx1, const MatrixXd & _jacobian2, const int _idx2, const VectorXd &_error, const int _meas_dim, bool _odometry_type=false) :
            jacobians({_jacobian1, _jacobian2}),
            nodes_idx({_idx1, _idx2}),
            error(_error),
            dim(_meas_dim),
            odometry_type(_odometry_type),
            location(0)
        {

        }
};

class SolverQR
{
    protected:
        std::string name_;
        SparseQR < SparseMatrix<double, ColMajor>, NaturalOrdering<int>> solver_;
        SparseMatrix<double, ColMajor> A_, R_;
        VectorXd b_, x_incr_;
        int n_measurements;
        int n_nodes_;
        std::vector<node> nodes_;
        std::vector<measurement> measurements_;

        // ordering
        SparseMatrix<int, ColMajor> A_nodes_;
        PermutationMatrix<Dynamic, Dynamic, int> acc_node_permutation_;

        CCOLAMDOrdering<int> orderer_;
        VectorXi node_ordering_restrictions_;
        int first_ordered_node_;

        // time
        clock_t t_ordering_, t_solving_, t_managing_;
        double time_ordering_, time_solving_, time_managing_;

    public:
        SolverQR(const std::string &_name) :
            name_(_name),
            A_(0,0),
            R_(0,0), 
//            b_(0),
//            x_(0),
            n_measurements(0),
            n_nodes_(0),
            A_nodes_(0,0),
            acc_node_permutation_(0),
//            nodes_(0),
//            measurements_(0),
            first_ordered_node_(0),
            t_ordering_(0),
            t_solving_(0),
            t_managing_(0),
            time_ordering_(0),
            time_solving_(0),
            time_managing_(0)
        {
            //
        }

        virtual ~SolverQR()
        {
            
        }

        void add_state_unit(const int node_dim, const int node_idx)
        {
            t_managing_ = clock();

            n_nodes_++;
            nodes_.push_back(node(node_idx, node_dim, x_incr_.size(), n_nodes_-1));

            // Resize accumulated permutations
            augment_permutation(acc_node_permutation_, n_nodes_);

            // Resize state
            x_incr_.conservativeResize(x_incr_.size() + node_dim);

            // Resize problem
            A_.conservativeResize(A_.rows(), A_.cols() + node_dim);
            R_.conservativeResize(R_.cols() + node_dim, R_.cols() + node_dim);
            //A_nodes_.conservativeResize(n_measurements, n_nodes); // not necessary

            time_managing_ += ((double) clock() - t_managing_) / CLOCKS_PER_SEC;
        }

        void addFactor(const measurement& _meas)
        {
            t_managing_ = clock();

            assert(_meas.jacobians.size() == _meas.nodes_idx.size());
            assert(_meas.error.size() == _meas.dim);

            n_measurements++;
            measurements_.push_back(_meas);
            measurements_.back().location = A_.rows();

            // Resize problem
            A_.conservativeResize(A_.rows() + _meas.dim, A_.cols());
            b_.conservativeResize(b_.size() + _meas.dim);
            A_nodes_.conservativeResize(n_measurements,n_nodes_);

            // ADD MEASUREMENTS
            first_ordered_node_ = n_nodes_;
            for (unsigned int j = 0; j < _meas.nodes_idx.size(); j++)
            {
                assert(acc_node_permutation_.indices()(_meas.nodes_idx.at(j)) == nodes_.at(_meas.nodes_idx.at(j)).order);

                int ordered_node = nodes_.at(_meas.nodes_idx.at(j)).order;//acc_permutation_nodes_.indices()(_nodes_idx.at(j));

                addSparseBlock(_meas.jacobians.at(j), A_, A_.rows()-_meas.dim, nodes_.at(_meas.nodes_idx.at(j)).location);

                A_nodes_.coeffRef(A_nodes_.rows()-1, ordered_node) = 1;

                assert(_meas.jacobians.at(j).cols() == nodes_.at(_meas.nodes_idx.at(j)).dim);
                assert(_meas.jacobians.at(j).rows() == _meas.dim);

                // store minimum ordered node
                if (first_ordered_node_ > ordered_node)
                    first_ordered_node_ = ordered_node;
            }

            // error
            b_.tail(_meas.dim) = _meas.error;

            time_managing_ += ((double) clock() - t_managing_) / CLOCKS_PER_SEC;
        }

        void ordering(const int & _first_ordered_node)
        {
            t_ordering_ = clock();

            // full problem ordering
            if (_first_ordered_node == 0)
            {
                // ordering ordering factors
                node_ordering_restrictions_.resize(n_nodes_);
                node_ordering_restrictions_ = A_nodes_.bottomRows(1).transpose();

                // computing nodes partial ordering_
                A_nodes_.makeCompressed();
                PermutationMatrix<Dynamic, Dynamic, int> incr_permutation_nodes(n_nodes_);
                orderer_(A_nodes_, incr_permutation_nodes, node_ordering_restrictions_.data());

                // node ordering to variable ordering
                PermutationMatrix<Dynamic, Dynamic, int> incr_permutation(A_.cols());
                nodePermutation2VariablesPermutation(incr_permutation_nodes, incr_permutation);

                // apply partial_ordering orderings
                A_nodes_ = (A_nodes_ * incr_permutation_nodes.transpose()).sparseView();
                A_ = (A_ * incr_permutation.transpose()).sparseView();

                // ACCUMULATING PERMUTATIONS
                accumulatePermutation(incr_permutation_nodes);
            }

            // partial ordering
            else
            {
                int ordered_nodes = n_nodes_ - _first_ordered_node;
                int unordered_nodes = n_nodes_ - ordered_nodes;
                if (ordered_nodes > 2) // only reordering when involved nodes in the measurement are not the two last ones
                {
                    // SUBPROBLEM ORDERING (from first node variable to last one)
                    //std::cout << "ordering partial_ordering problem: " << _first_ordered_node << " to "<< n_nodes_ - 1 << std::endl;
                    SparseMatrix<int> sub_A_nodes_ = A_nodes_.rightCols(ordered_nodes);

                    // _partial_ordering ordering_ factors
                    node_ordering_restrictions_.resize(ordered_nodes);
                    node_ordering_restrictions_ = sub_A_nodes_.bottomRows(1).transpose();

                    // computing nodes partial ordering_
                    sub_A_nodes_.makeCompressed();
                    PermutationMatrix<Dynamic, Dynamic, int> partial_permutation_nodes(ordered_nodes);
                    orderer_(sub_A_nodes_, partial_permutation_nodes, node_ordering_restrictions_.data());

                    // node ordering to variable ordering
                    PermutationMatrix<Dynamic, Dynamic, int> partial_permutation(A_.cols());
                    nodePermutation2VariablesPermutation(partial_permutation_nodes, partial_permutation);

                    // apply partial_ordering orderings
                    int ordered_variables = A_.cols() - nodes_.at(_first_ordered_node).location;
                    A_nodes_.rightCols(ordered_nodes) = (A_nodes_.rightCols(ordered_nodes) * partial_permutation_nodes.transpose()).sparseView();
                    A_.rightCols(ordered_variables) = (A_.rightCols(ordered_variables) * partial_permutation.transpose()).sparseView();
                    R_.rightCols(ordered_variables) = (R_.rightCols(ordered_variables) * partial_permutation.transpose()).sparseView();

                    // ACCUMULATING PERMUTATIONS
                    accumulatePermutation(partial_permutation_nodes);
                }
            }
            time_ordering_ += ((double) clock() - t_ordering_) / CLOCKS_PER_SEC;
        }

        bool solve(const int mode)
        {
            bool batch = (mode !=2 || first_ordered_node_ == 0);
            bool order = (mode !=0 && n_nodes_ > 1);

            // BATCH
            if (batch)
            {
                // REORDER
                if (order)
                    ordering(0);

                //print_problem();

                // SOLVE
                t_solving_ = clock();
                A_.makeCompressed();
                solver_.compute(A_);
                if (solver_.info() != Success)
                {
                    std::cout << "decomposition failed" << std::endl;
                    return 0;
                }
                x_incr_ = solver_.solve(b_);
                R_ = solver_.matrixR();
                //std::cout << "R" << std::endl << MatrixXd::Identity(R_.cols(), R_.cols()) * R_ << std::endl;
                time_solving_ += ((double) clock() - t_solving_) / CLOCKS_PER_SEC;
            }
            // INCREMENTAL
            else
            {
                // REORDER SUBPROBLEM
                ordering(first_ordered_node_);
                //print_problem();

                // SOLVE ORDERED SUBPROBLEM
                t_solving_= clock();
                A_nodes_.makeCompressed();
                A_.makeCompressed();

                // finding measurements block
                SparseMatrix<int> measurements_to_initial = A_nodes_.col(first_ordered_node_);
        //        std::cout << "measurements_to_initial " << measurements_to_initial << std::endl;
        //        std::cout << "measurements_to_initial.innerIndex()[measurements_to_initial.outerIndex()[0]] " << measurements_to_initial.innerIndex()[measurements_to_initial.outerIndex()[0]] << std::endl;
                int first_ordered_measurement = measurements_to_initial.innerIndex()[measurements_to_initial.outerIndex()[0]];
                int ordered_measurements = A_.rows() - measurements_.at(first_ordered_measurement).location;
                int ordered_variables = A_.cols() - nodes_.at(first_ordered_node_).location;
                int unordered_variables = nodes_.at(first_ordered_node_).location;

                SparseMatrix<double, ColMajor> A_partial = A_.bottomRightCorner(ordered_measurements, ordered_variables);
                solver_.compute(A_partial);
                if (solver_.info() != Success)
                {
                    std::cout << "decomposition failed" << std::endl;
                    return 0;
                }
                //std::cout << "R new" << std::endl << MatrixXd::Identity(A_partial.cols(), A_partial.cols()) * solver_.matrixR() << std::endl;
                x_incr_.tail(ordered_variables) = solver_.solve(b_.tail(ordered_measurements));

                // store new part of R
                eraseSparseBlock(R_, unordered_variables, unordered_variables, ordered_variables, ordered_variables);
                //std::cout << "R" << std::endl << MatrixXd::Identity(R_.rows(), R_.rows()) * R_ << std::endl;
                addSparseBlock(solver_.matrixR(), R_, unordered_variables, unordered_variables);
                //std::cout << "R" << std::endl << MatrixXd::Identity(R_.rows(), R_.rows()) * R_ << std::endl;
                R_.makeCompressed();

                // solving not ordered subproblem
                if (unordered_variables > 0)
                {
                    //std::cout << "--------------------- solving unordered part" << std::endl;
                    SparseMatrix<double, ColMajor> R1 = R_.topLeftCorner(unordered_variables, unordered_variables);
                    //std::cout << "R1" << std::endl << MatrixXd::Identity(R1.rows(), R1.rows()) * R1 << std::endl;
                    SparseMatrix<double, ColMajor> R2 = R_.topRightCorner(unordered_variables, ordered_variables);
                    //std::cout << "R2" << std::endl << MatrixXd::Identity(R2.rows(), R2.rows()) * R2 << std::endl;
                    solver_.compute(R1);
                    if (solver_.info() != Success)
                    {
                        std::cout << "decomposition failed" << std::endl;
                        return 0;
                    }
                    x_incr_.head(unordered_variables) = solver_.solve(b_.head(unordered_variables) - R2 * x_incr_.tail(ordered_variables));
                }

            }
            // UNDO ORDERING FOR RESULT
            PermutationMatrix<Dynamic, Dynamic, int> acc_permutation(A_.cols());
            nodePermutation2VariablesPermutation(acc_node_permutation_, acc_permutation); // TODO via pointers
            x_incr_ = acc_permutation.inverse() * x_incr_;

            time_solving_ += ((double) clock() - t_solving_) / CLOCKS_PER_SEC;

            return 1;
        }

        void nodePermutation2VariablesPermutation(const PermutationMatrix<Dynamic, Dynamic, int> &_perm_nodes, PermutationMatrix<Dynamic, Dynamic, int> &perm_variables)
        {
            ArrayXi locations = perm_nodes_2_locations(_perm_nodes);

            int last_idx = 0;
            for (unsigned int i = 0; i<locations.size(); i++)
            {
                perm_variables.indices().segment(last_idx, nodes_.at(i).dim) = VectorXi::LinSpaced(nodes_.at(i).dim, locations(i), locations(i)+nodes_.at(i).dim-1);
                last_idx += nodes_.at(i).dim;
            }
        }

        ArrayXi perm_nodes_2_locations(const PermutationMatrix<Dynamic, Dynamic, int> &_perm_nodes)
        {
            ArrayXi indices = _perm_nodes.indices().array();

            for (unsigned int i = 0; i<indices.size(); i++)
                indices = (indices > indices(i)).select(indices + nodes_.at(i).dim-1, indices);

            return indices;
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

        void accumulatePermutation(const PermutationMatrix<Dynamic, Dynamic, int> &perm)
        {
            printName();
            //std::cout << std::endl << "old acc_permutation_nodes_ " << acc_permutation_nodes_.indices().transpose() << std::endl;
            //std::cout << "incr perm " << perm.indices().transpose() << std::endl;

            // acumulate permutation
            if (perm.size() == acc_node_permutation_.size()) //full permutation
                acc_node_permutation_ = perm * acc_node_permutation_;
            else //partial permutation
            {
                PermutationMatrix<Dynamic, Dynamic, int> incr_permutation_nodes(VectorXi::LinSpaced(n_nodes_, 0, n_nodes_ - 1)); // identity permutation
                incr_permutation_nodes.indices().tail(perm.size()) = perm.indices() + VectorXi::Constant(perm.size(), n_nodes_ - perm.size());
                //std::cout << "incr perm " << incr_permutation_nodes.indices().transpose() << std::endl;
                acc_node_permutation_ = incr_permutation_nodes * acc_node_permutation_;
            }
            //std::cout << "new acc_permutation_nodes_ " << acc_permutation_nodes_.indices().transpose() << std::endl;

            // update nodes orders and locations
            ArrayXi locations = perm_nodes_2_locations(acc_node_permutation_);
            for (unsigned int i = 0; i < nodes_.size(); i++)
            {
                nodes_.at(i).order = acc_node_permutation_.indices()(i);
                nodes_.at(i).location = locations(i);
            }
        }

        void printName()
        {
            std::cout << name_;
        }

        void printResults()
        {
            printName();
            std::cout << " solved in " << time_solving_*1e3 << " ms | " << R_.nonZeros() << " nonzeros in R"<< std::endl;
            std::cout << "x = " << x_incr_.transpose() << std::endl;
        }

        void printProblem()
        {
            printName();
            std::cout << std::endl << "A_nodes_: " << std::endl << MatrixXi::Identity(A_nodes_.rows(), A_nodes_.rows()) * A_nodes_  << std::endl << std::endl;
            std::cout << "A_: " << std::endl << MatrixXd::Identity(A_.rows(), A_.rows()) * A_  << std::endl << std::endl;
            std::cout << "b_: " << std::endl << b_.transpose() << std::endl << std::endl;
        }
};

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

    // Problems
    SolverQR solver_ordered("FULL ORDERED");
    SolverQR solver_unordered("UNORDERED");
    SolverQR solver_ordered_partial("PARTIALLY ORDERED");

    MatrixXd omega = MatrixXd::Constant(dim, dim, 0.1) + MatrixXd::Identity(dim, dim);

    // results variables
    clock_t t_ordering, t_solving_ordered_full, t_solving_unordered, t_solving_ordered_partial, t4;
    double time_ordering=0, time_solving_unordered=0, time_solving_ordered=0, time_solving_ordered_partial=0;

    std::cout << "STARTING INCREMENTAL QR TEST" << std::endl << std::endl;

    // GENERATING MEASUREMENTS
    std::vector<measurement> measurements;
    for (int i = 0; i < size; i++)
    {
        std::vector<int> meas(0);
        if (i == 0) //prior
            measurements.push_back(measurement(omega, 0, VectorXd::LinSpaced(dim, 0, dim-1), dim));

        else //odometry
            measurements.push_back(measurement(2*omega, i-1, 2*omega, i, VectorXd::LinSpaced(dim, dim * i, dim * (i+1)-1), dim, true));

        if (i > size / 2) //loop closures
            measurements.push_back(measurement(4*omega, 0, 4*omega, i, VectorXd::LinSpaced(dim, dim * i, dim * (i+1)-1), dim));
    }

    // INCREMENTAL LOOP
    for (unsigned int i = 0; i < measurements.size(); i++)
    {
        std::cout << "========================= MEASUREMENT " << i << ":" << std::endl;

        // AUGMENT THE PROBLEM ----------------------------
        if (measurements.at(i).odometry_type || i == 0) // if odometry, augment the problem
        {
            solver_unordered.add_state_unit(dim, i);
            solver_ordered.add_state_unit(dim, i);
            solver_ordered_partial.add_state_unit(dim,i);
        }

        // ADD MEASUREMENTS
        solver_unordered.addFactor(measurements.at(i));
        solver_ordered.addFactor(measurements.at(i));
        solver_ordered_partial.addFactor(measurements.at(i));

        // PRINT PROBLEM
        solver_unordered.printProblem();
        solver_ordered.printProblem();
        solver_ordered_partial.printProblem();

        // SOLVING
        solver_unordered.solve(0);
        solver_ordered.solve(1);
        solver_ordered_partial.solve(2);

        // RESULTS ------------------------------------
        std::cout << "========================= RESULTS " << i << ":" << std::endl;
        solver_unordered.printResults();
        solver_ordered.printResults();
        solver_ordered_partial.printResults();

//        if ((x_ordered_partial-x_ordered).maxCoeff() < 1e-10)
//            std::cout << "Both solutions are equals (tolerance " << (x_ordered_partial-x_ordered).maxCoeff() << ")" << std::endl;
//        else
//            std::cout << "DIFFERENT SOLUTIONS!!!!!!!! max difference " << (x_ordered_partial-x_ordered).maxCoeff() << std::endl;
    }
}


/*
 * test_SPQR.cpp
 *
 *  Created on: Jun 18, 2015
 *      Author: jvallve
 */

#include <iostream>
#include <eigen3/Eigen/SPQRSupport>
#include <eigen3/Eigen/CholmodSupport>
#include "SuiteSparseQR.hpp"

using namespace Eigen;

int main (int argc, char **argv)
{
    ///////////////////////////////////////////////////////////////////////
    // Eigen Support SPQR
    SPQR < SparseMatrix<double> > solver;
    //solver.setSPQROrdering(0); // no ordering -> segmentation fault

    SparseMatrix<double> matA(4,3);
    matA.coeffRef(0,0) = 0.1;
    matA.coeffRef(1,0) = 0.4;
    matA.coeffRef(1,1) = 0.2;
    matA.coeffRef(2,1) = 0.4;
    matA.coeffRef(2,2) = 0.2;
    matA.coeffRef(3,2) = 0.1;

    std::cout << "matA: " << std::endl << matA << std::endl;

    VectorXd b_ = VectorXd::Ones(4);
    VectorXd x_(3);

    std::cout << "b_: " << std::endl << b_ << std::endl;

    solver.compute(matA);
    if (solver.info() != Success)
    {
        std::cout << "decomposition failed" << std::endl;
        return 0;
    }
    std::cout << "R: " << std::endl << solver.matrixR() << std::endl;
    x_ = solver.solve(b_);
    std::cout << "solved x_" << std::endl << x_ << std::endl;
    std::cout << "ordering: " << solver.colsPermutation().indices().transpose() << std::endl;

    ///////////////////////////////////////////////////////////////////////
    // Directly in suitesparse
    cholmod_common Common, *cc ;
    cholmod_sparse A ;
    cholmod_dense *X, *B, *Residual ;
    double rnorm, one [2] = {1,0}, minusone [2] = {-1,0} ;
    int mtype ;

    // start CHOLMOD
    cc = &Common ;
    cholmod_l_start (cc) ;

    // load A
    A = viewAsCholmod(matA);
    //A = (cholmod_sparse *) cholmod_l_read_matrix (stdin, 1, &mtype, cc) ;
    std::cout << "A.xtype " << A.xtype << std::endl;
    std::cout << "A.nrow " << A.nrow << std::endl;
    std::cout << "A.ncol " << A.ncol << std::endl;

    // B = ones (size (A,1),1)
    B = cholmod_l_ones (A.nrow, 1, A.xtype, cc) ;

    std::cout << "2" << std::endl;
    // X = A\B
    //X = SuiteSparseQR <double> (0, SPQR_DEFAULT_TOL, &A, B, cc) ;
    X = SuiteSparseQR <double> (&A, B, cc);

    std::cout << "3" << std::endl;
    // rnorm = norm (B-A*X)
    Residual = cholmod_l_copy_dense (B, cc) ;
    std::cout << "4" << std::endl;
    cholmod_l_sdmult (&A, 0, minusone, one, X, Residual, cc) ;
    std::cout << "5" << std::endl;
    rnorm = cholmod_l_norm_dense (Residual, 2, cc) ;
    printf ("2-norm of residual: %8.1e\n", rnorm) ;
    printf ("rank %ld\n", cc->SPQR_istat [4]) ;

    // free everything and finish CHOLMOD
    cholmod_l_free_dense (&Residual, cc) ;
    //cholmod_l_free_sparse (A, cc) ;
    cholmod_l_free_dense (&X, cc) ;
    cholmod_l_free_dense (&B, cc) ;
    cholmod_l_finish (cc) ;
    return (0) ;
}

/**
 * \file wolf.h
 * \brief General typedefs for the Wolf project
 * \author Joan Sola
 *  Created on: 28/05/2014
 */

#ifndef WOLF_H_
#define WOLF_H_

// Enable project-specific definitions and macros
#include "internal/config.h"
#include "base/logging.h"

//includes from Eigen lib
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Sparse>
#include <libgen.h>

//includes from std lib
#include <list>
#include <map>
#include <memory> // shared_ptr and weak_ptr

// System specifics
#include <sys/stat.h>

namespace wolf {

/**
 * \brief scalar type for the Wolf project
 *
 * This typedef makes it possible to recompile the whole Wolf project with double, float, or other precisions.
 *
 * To change the project precision, just edit wolf.h and change the precision of this typedef.
 *
 * Do NEVER forget to use Wolf scalar definitions when you code!!!
 *
 * Do NEVER use plain Eigen scalar types!!! (This is redundant with the above. But just to make sure you behave!)
 *
 * The ONLY exception to this rule is when you need special precision. The ONLY example by now is the time stamp which uses double.
 */
//typedef float Scalar;         // Use this for float, 32 bit precision
typedef double Scalar;        // Use this for double, 64 bit precision
//typedef long double Scalar;   // Use this for long double, 128 bit precision

/**
 * \brief Vector and Matrices size type for the Wolf project
 *
 * We use the default defined in Eigen (std::ptrdiff_t, a signed integer)
 *
 */
typedef EIGEN_DEFAULT_DENSE_INDEX_TYPE SizeEigen;

/**
 * \brief Other containers size type for the Wolf project
 *
 * We use the default defined in std (std::size_t, an unsigned integer)
 *
 */
typedef std::size_t SizeStd;

#define M_TORAD 0.017453292519943295769236907684886127134  // pi / 180
#define M_TODEG 57.295779513082320876798154814105170332    // 180 / pi

namespace Constants{

// Wolf standard tolerance
const Scalar EPS = 1e-8;
// Wolf smmmmall tolerance
const Scalar EPS_SMALL = 1e-16;
}

} // namespace wolf

///////////////////////////////////////////
// Construct types for any scalar defined in the typedef Scalar above
////////////////////////////////////////////
/** \brief Namespace extending Eigen definitions
 *
 * We redefine all algebra and gemoetry types to hold double or single precision floats.
 * The appended letter indicating this is 's', so that we have, e.g.,
 * - VectorXf   Vector of floats - defined by Eigen
 * - VectorXd   Vector of doubles - defined by Eigen
 * - VectorXs   Vector of either double of float, depending on the type \b Scalar, defined by Wolf.
 *
 */
namespace Eigen  // Eigen namespace extension
{
// 1. Vectors and Matrices
typedef Matrix<wolf::Scalar, 1, 1, RowMajor> Matrix1s;                ///< 2x2 matrix of real Scalar type
typedef Matrix<wolf::Scalar, 2, 2, RowMajor> Matrix2s;                ///< 2x2 matrix of real Scalar type
typedef Matrix<wolf::Scalar, 3, 3, RowMajor> Matrix3s;                ///< 3x3 matrix of real Scalar type
typedef Matrix<wolf::Scalar, 4, 4, RowMajor> Matrix4s;                ///< 4x4 matrix of real Scalar type
typedef Matrix<wolf::Scalar, 6, 6, RowMajor> Matrix6s;                ///< 6x6 matrix of real Scalar type
typedef Matrix<wolf::Scalar, 7, 7, RowMajor> Matrix7s;                ///< 7x7 matrix of real Scalar type
typedef Matrix<wolf::Scalar, Dynamic, Dynamic, RowMajor> MatrixXs;    ///< variable size matrix of real Scalar type
typedef Matrix<wolf::Scalar, 1, 1> Vector1s;                ///< 1-vector of real Scalar type
typedef Matrix<wolf::Scalar, 2, 1> Vector2s;                ///< 2-vector of real Scalar type
typedef Matrix<wolf::Scalar, 3, 1> Vector3s;                ///< 3-vector of real Scalar type
typedef Matrix<wolf::Scalar, 4, 1> Vector4s;                ///< 4-vector of real Scalar type
typedef Matrix<wolf::Scalar, 5, 1> Vector5s;                ///< 5-vector of real Scalar type
typedef Matrix<wolf::Scalar, 6, 1> Vector6s;                ///< 6-vector of real Scalar type
typedef Matrix<wolf::Scalar, 7, 1> Vector7s;                ///< 7-vector of real Scalar type
typedef Matrix<wolf::Scalar, 8, 1> Vector8s;                ///< 8-vector of real Scalar type
typedef Matrix<wolf::Scalar, 9, 1> Vector9s;                ///< 9-vector of real Scalar type
typedef Matrix<wolf::Scalar, 10, 1> Vector10s;              ///< 10-vector of real Scalar type
typedef Matrix<wolf::Scalar, Dynamic, 1> VectorXs;          ///< variable size vector of real Scalar type
typedef Matrix<wolf::Scalar, 1, 2> RowVector2s;             ///< 2-row-vector of real Scalar type
typedef Matrix<wolf::Scalar, 1, 3> RowVector3s;             ///< 3-row-vector of real Scalar type
typedef Matrix<wolf::Scalar, 1, 4> RowVector4s;             ///< 4-row-vector of real Scalar type
typedef Matrix<wolf::Scalar, 1, 7> RowVector7s;             ///< 7-row-vector of real Scalar type
typedef Matrix<wolf::Scalar, 1, Dynamic> RowVectorXs;       ///< variable size row-vector of real Scalar type

// 2. Quaternions and other rotation things
typedef Quaternion<wolf::Scalar> Quaternions;               ///< Quaternion of real Scalar type
typedef AngleAxis<wolf::Scalar> AngleAxiss;                 ///< Angle-Axis of real Scalar type
typedef Rotation2D<wolf::Scalar> Rotation2Ds;               ///< Rotation2D of real Scalar type
typedef RotationBase<wolf::Scalar, 3> Rotation3Ds;          ///< Rotation3D of real Scalar type

// 3. Translations
typedef Translation<wolf::Scalar, 2> Translation2ds;
typedef Translation<wolf::Scalar, 3> Translation3ds;

// 4. Transformations
typedef Transform<wolf::Scalar,2,Affine> Affine2ds;         ///< Affine2d of real Scalar type
typedef Transform<wolf::Scalar,3,Affine> Affine3ds;         ///< Affine3d of real Scalar type

typedef Transform<wolf::Scalar,2,Isometry> Isometry2ds;     ///< Isometry2d of real Scalar type
typedef Transform<wolf::Scalar,3,Isometry> Isometry3ds;     ///< Isometry3d of real Scalar type

// 3. Sparse matrix
typedef SparseMatrix<wolf::Scalar, RowMajor, int> SparseMatrixs;
}

namespace wolf {

//////////////////////////////////////////////////////////
/** Check Eigen Matrix sizes, both statically and dynamically
 *
 * Help:
 *
 * The WOLF project implements many template functions using Eigen Matrix and Quaternions, in different versions
 * (Static size, Dynamic size, Map, Matrix expression).
 *
 * Eigen provides some macros for STATIC assert of matrix sizes, the most common of them are (see Eigen's StaticAssert.h):
 *
 *      EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE
 *      EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE
 *      EIGEN_STATIC_ASSERT_VECTOR_ONLY
 *
 * but they do not work if the evaluated types are of dynamic size.
 *
 * In order to achieve full templatization over both dynamic and static sizes, we use extensively a prototype of this kind:
 *
 * template<typename Derived>
 * inline Eigen::Matrix<typename Derived::Scalar, 3, 3> function(const Eigen::MatrixBase<Derived>& _v){
 *
 *     MatrixSizeCheck<3,1>::check(_v); /// We check here the size of the input parameter
 *
 *     typedef typename Derived::Scalar T;
 *
 *     ... code ...
 *
 *     return M;
 *     }
 *
 * The function :  MatrixSizeCheck <Rows, Cols>::check(M)   checks that the Matrix M is of size ( Rows x Cols ).
 * This check is performed statically or dynamically, depending on the type of argument provided.
 */
template<int Size, int DesiredSize>
struct StaticSizeCheck
{
        template<typename T>
        StaticSizeCheck(const T&)
        {
            static_assert(Size == DesiredSize, "Size of static Vector or Matrix does not match");
        }
};
//
template<int DesiredSize>
struct StaticSizeCheck<Eigen::Dynamic, DesiredSize>
{
        template<typename T>
        StaticSizeCheck(const T& t)
        {
            assert(t == DesiredSize && "Size of dynamic Vector or Matrix does not match");
        }
};
//
template<int DesiredR, int DesiredC>
struct MatrixSizeCheck
{
        /** \brief Assert matrix size dynamically or statically
         *
         * @param t the variable for which we wish to assert the size.
         *
         * Usage: to assert that t is size (Rows x Cols)
         *
         *  MatrixSizeCheck<Rows, Cols>::check(t);
         */
        template<typename T>
        static void check(const T& t)
        {
            StaticSizeCheck<T::RowsAtCompileTime, DesiredR>(t.rows());
            StaticSizeCheck<T::ColsAtCompileTime, DesiredC>(t.cols());
        }
};
//
// End of check matrix sizes /////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////
//      TYPEDEFS FOR POINTERS, LISTS AND ITERATORS IN THE WOLF TREE
/////////////////////////////////////////////////////////////////////////

#define WOLF_PTR_TYPEDEFS(ClassName) \
        class ClassName; \
        typedef std::shared_ptr<ClassName>          ClassName##Ptr; \
        typedef std::shared_ptr<const ClassName>    ClassName##ConstPtr; \
        typedef std::weak_ptr<ClassName>            ClassName##WPtr;

#define WOLF_LIST_TYPEDEFS(ClassName) \
        class ClassName; \
        typedef std::list<ClassName##Ptr>          ClassName##PtrList; \
        typedef ClassName##PtrList::iterator          ClassName##Iter; \
        typedef ClassName##PtrList::reverse_iterator  ClassName##RevIter;

#define WOLF_STRUCT_PTR_TYPEDEFS(StructName) \
        struct StructName; \
        typedef std::shared_ptr<StructName>          StructName##Ptr; \
        typedef std::shared_ptr<const StructName>    StructName##ConstPtr; \

// NodeBase
WOLF_PTR_TYPEDEFS(NodeBase);

// Problem
WOLF_PTR_TYPEDEFS(Problem);

// Hardware
WOLF_PTR_TYPEDEFS(HardwareBase);

// - Sensors
WOLF_PTR_TYPEDEFS(SensorBase);
WOLF_LIST_TYPEDEFS(SensorBase);

// - - Intrinsics
WOLF_STRUCT_PTR_TYPEDEFS(IntrinsicsBase);

// - Processors
WOLF_PTR_TYPEDEFS(ProcessorBase);
WOLF_LIST_TYPEDEFS(ProcessorBase);

// - ProcessorMotion
WOLF_PTR_TYPEDEFS(ProcessorMotion);

// - - Processor params
WOLF_STRUCT_PTR_TYPEDEFS(ProcessorParamsBase);

// Trajectory
WOLF_PTR_TYPEDEFS(TrajectoryBase);

// - Frame
WOLF_PTR_TYPEDEFS(FrameBase);
WOLF_LIST_TYPEDEFS(FrameBase);

// - Capture
WOLF_PTR_TYPEDEFS(CaptureBase);
WOLF_LIST_TYPEDEFS(CaptureBase);

// - Feature
WOLF_PTR_TYPEDEFS(FeatureBase);
WOLF_LIST_TYPEDEFS(FeatureBase);

// - Factor
WOLF_PTR_TYPEDEFS(FactorBase);
WOLF_LIST_TYPEDEFS(FactorBase);

// Map
WOLF_PTR_TYPEDEFS(MapBase);

// - Landmark
WOLF_PTR_TYPEDEFS(LandmarkBase);
WOLF_LIST_TYPEDEFS(LandmarkBase);

// - - State blocks
WOLF_PTR_TYPEDEFS(StateBlock);
WOLF_LIST_TYPEDEFS(StateBlock);
WOLF_PTR_TYPEDEFS(StateAngle);
WOLF_PTR_TYPEDEFS(StateQuaternion);

// - - Local Parametrization
WOLF_PTR_TYPEDEFS(LocalParametrizationBase);

// ==================================================
// Some dangling functions

inline bool file_exists(const std::string& _name)
{
    struct stat buffer;
    return (stat (_name.c_str(), &buffer) == 0);
}

inline const Eigen::Vector3s gravity(void) {
    return Eigen::Vector3s(0,0,-9.806);
}

template <typename T, int N, int RC>
bool isSymmetric(const Eigen::Matrix<T, N, N, RC>& M,
                 const T eps = wolf::Constants::EPS)
{
  return M.isApprox(M.transpose(), eps);
}

template <typename T, int N, int RC>
bool isPositiveSemiDefinite(const Eigen::Matrix<T, N, N, RC>& M, const T& eps = Constants::EPS)
{
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<T, N, N, RC> > eigensolver(M);

  if (eigensolver.info() == Eigen::Success)
  {
    // All eigenvalues must be >= 0:
    return (eigensolver.eigenvalues().array() >= eps).all();
  }

  return false;
}

template <typename T, int N, int RC>
bool isCovariance(const Eigen::Matrix<T, N, N, RC>& M, const T& eps = Constants::EPS)
{
  return isSymmetric(M) && isPositiveSemiDefinite(M, eps);
}

#define WOLF_ASSERT_COVARIANCE_MATRIX(x) \
  assert(isCovariance(x, Constants::EPS_SMALL) && "Not a covariance");

#define WOLF_ASSERT_INFORMATION_MATRIX(x) \
  assert(isCovariance(x, Scalar(0.0)) && "Not an information matrix");

template <typename T, int N, int RC>
bool makePosDef(Eigen::Matrix<T,N,N,RC>& M, const T& eps = Constants::EPS)
{
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<T,N,N,RC> > eigensolver(M);

    if (eigensolver.info() == Eigen::Success)
    {
        // All eigenvalues must be >= 0:
        Scalar epsilon = eps;
        while ((eigensolver.eigenvalues().array() < eps).any())
        {
            //std::cout << "----- any negative eigenvalue or too close to zero\n";
            //std::cout << "previous eigenvalues: " << eigensolver.eigenvalues().transpose() << std::endl;
            //std::cout << "previous determinant: " << M.determinant() << std::endl;
            M = eigensolver.eigenvectors() *
                eigensolver.eigenvalues().cwiseMax(epsilon).asDiagonal() *
                eigensolver.eigenvectors().transpose();
            eigensolver.compute(M);
            //std::cout << "epsilon used: " << epsilon << std::endl;
            //std::cout << "posterior eigenvalues: " << eigensolver.eigenvalues().transpose() << std::endl;
            //std::cout << "posterior determinant: " << M.determinant() << std::endl;
            epsilon *=10;
        }
        WOLF_ASSERT_COVARIANCE_MATRIX(M);

        return epsilon != eps;
    }
    else
        WOLF_ERROR("Couldn't compute covariance eigen decomposition");

    return false;
}

//===================================================

} // namespace wolf

#endif /* WOLF_H_ */

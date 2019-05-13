/**
 * \file test_eigen_template.cpp
 *
 *  Created on: Sep 12, 2016
 *      \author: jsola
 */

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <iostream>

template <int Size, int DesiredSize>
struct StaticSizeCheck {
        template <typename T>
        StaticSizeCheck(const T&) {
            static_assert(Size == DesiredSize, "Static sizes do not match");
        }
};

template <int DesiredSize>
struct StaticSizeCheck<Eigen::Dynamic, DesiredSize> {
        template <typename T>
        StaticSizeCheck(const T& t) {
            assert(t == DesiredSize && "Dynamic sizes do not match");
        }
};

template <int DesiredR, int DesiredC>
struct MatrixSizeCheck {
        template <typename T>
        static void check(const T& t) {
            StaticSizeCheck<T::RowsAtCompileTime, DesiredR>(t.rows());
            StaticSizeCheck<T::ColsAtCompileTime, DesiredC>(t.cols());
        }
};

template<typename Derived>
inline Eigen::Quaternion<typename Derived::Scalar> v2q(const Eigen::MatrixBase<Derived>& _v){

    MatrixSizeCheck<3, 1>::check(_v);

    Eigen::Quaternion<typename Derived::Scalar> q;
    typename Derived::Scalar angle = _v.norm();
    typename Derived::Scalar angle_half = angle/2.0;
    if (angle > 1e-8)
    {
        q.w() = cos(angle_half);
        q.vec() = _v / angle * sin(angle_half);
        return q;
    }
    else
    {
        q.w() = cos(angle_half);
        q.vec() = _v * ( (typename Derived::Scalar)0.5 - angle_half*angle_half/(typename Derived::Scalar)12.0 ); // see the Taylor series of sinc(x) ~ 1 - x^2/3!, and have q.vec = v/2 * sinc(angle_half)
        return q;
    }
}

int main(void)
{
    using namespace Eigen;

    VectorXd x(10);
    x << 1,2,3,4,5,6,7,8,9,10;

    Quaterniond q;
    Map<Quaterniond> qm(x.data()+5);

    // Static vector
    Vector3d v;
    v << 1,2,3;
    q  = v2q(v);
    qm = v2q(v);
    std::cout << q.coeffs().transpose() << std::endl;
    std::cout << qm.coeffs().transpose() << std::endl;

    // Dynamic matrix
    Matrix<double, Dynamic, Dynamic> M(3,1);
    M << 1, 2, 3;
    q  = v2q(M);
    std::cout << q.coeffs().transpose() << std::endl;

    // Dynamic vector segment
    x << 1,2,3,4,5,6,7,8,9,10;
    q  = v2q(x.head(3));
    std::cout << q.coeffs().transpose() << std::endl;

    // Map over dynamic vector
    Map<VectorXd> m(x.data(), 3);
    q  = v2q(m);
    std::cout << q.coeffs().transpose() << std::endl;

    // Float scalar
    Vector3f vf;
    Quaternionf qf;
    vf << 1,2,3;
    qf = v2q(vf);
    std::cout << qf.coeffs().transpose() << std::endl;

    //    // Static assert at compile time
    //    Vector2d v2;
    //    q= v2q(v2);
    //    std::cout << q.coeffs().transpose() << std::endl;

}

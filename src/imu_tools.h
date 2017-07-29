/*
 * imu_tools.h
 *
 *  Created on: Jul 29, 2017
 *      Author: jsola
 */

#ifndef IMU_TOOLS_H_
#define IMU_TOOLS_H_


#include "wolf.h"
#include "rotations.h"

namespace wolf 
{
namespace imu {
using namespace Eigen;

template<typename T = wolf::Scalar>
inline Matrix<T, 10, 1> identity()
{
    Matrix<T, 10, 1> ret;
    ret.setZero();
    ret(6) = 1.0;
    return ret;
}

//template<>
//inline Matrix<Scalar, 10, 1> identity()
//{
//    Matrix<Scalar, 10, 1> ret;
//    ret.setZero();
//    ret(6, 1) = 1.0;
//    return ret;
//}

template<typename D1, typename D2, typename D3>
inline void compose(const MatrixBase<D1>& d1,
                 const MatrixBase<D2>& d2,
                 const Scalar& dt,
                 MatrixBase<D3>& sum)
{
    MatrixSizeCheck<10, 1>::check(d1);
    MatrixSizeCheck<10, 1>::check(d2);
    MatrixSizeCheck<10, 1>::check(sum);

    Map<const Matrix<typename D1::Scalar, 3, 1> >   dp1    ( &d1( 0 ) );
    Map<const Quaternion<typename D1::Scalar> >     dq1    ( &d1( 3 ) );
    Map<const Matrix<typename D1::Scalar, 3, 1> >   dv1    ( &d1( 7 ) );
    Map<const Matrix<typename D2::Scalar, 3, 1> >   dp2    ( &d2( 0 ) );
    Map<const Quaternion<typename D2::Scalar> >     dq2    ( &d2( 3 ) );
    Map<const Matrix<typename D2::Scalar, 3, 1> >   dv2    ( &d2( 7 ) );
    Map<Matrix<typename D3::Scalar, 3, 1> >         sum_p  ( &sum( 0 ));
    Map<Quaternion<typename D3::Scalar> >           sum_q  ( &sum( 3 ));
    Map<Matrix<typename D3::Scalar, 3, 1> >         sum_v  ( &sum( 7 ));

    sum_p = dp1 + dv1*dt + dq1*dp2;
    sum_q =                dq1*dq2;
    sum_v = dv1 +          dq1*dv2;
}

template<typename D1, typename D2, typename D3>
inline void between(const MatrixBase<D1>& d1,
                  const MatrixBase<D2>& d2,
                  const Scalar& dt,
                  MatrixBase<D3>& d2_minus_d1)
{
    MatrixSizeCheck<10, 1>::check(d1);
    MatrixSizeCheck<10, 1>::check(d2);
    MatrixSizeCheck<10, 1>::check(d2_minus_d1);

    Map<const Matrix<typename D1::Scalar, 3, 1> >   dp1    ( &d1(0) );
    Map<const Quaternion<typename D1::Scalar> >     dq1    ( &d1(3) );
    Map<const Matrix<typename D1::Scalar, 3, 1> >   dv1    ( &d1(7) );
    Map<const Matrix<typename D2::Scalar, 3, 1> >   dp2    ( &d2(0) );
    Map<const Quaternion<typename D2::Scalar> >     dq2    ( &d2(3) );
    Map<const Matrix<typename D2::Scalar, 3, 1> >   dv2    ( &d2(7) );
    Map<Matrix<typename D3::Scalar, 3, 1> >         diff_p (&d2_minus_d1(0));
    Map<Quaternion<typename D3::Scalar> >           diff_q (&d2_minus_d1(3));
    Map<Matrix<typename D3::Scalar, 3, 1> >         diff_v (&d2_minus_d1(7));

    diff_p = dq1.conjugate() * ( dp2 - dp1 - dv1*dt );
    diff_q = dq1.conjugate() *   dq2;
    diff_v = dq1.conjugate() * ( dv2 - dv1 );
}

template<typename Derived>
Matrix<typename Derived::Scalar, 9, 1> lift(const MatrixBase<Derived>& delta_in)
{
    MatrixSizeCheck<10, 1>::check(delta_in);

    Matrix<typename Derived::Scalar, 9, 1> ret;

    Map<const Matrix<typename Derived::Scalar, 3, 1> >   dp_in  ( &delta_in(0) );
    Map<const Quaternion<typename Derived::Scalar> >     dq_in  ( &delta_in(3) );
    Map<const Matrix<typename Derived::Scalar, 3, 1> >   dv_in  ( &delta_in(7) );
    Map<Matrix<typename Derived::Scalar, 3, 1> >         dp_ret     ( & ret(0) );
    Map<Matrix<typename Derived::Scalar, 3, 1> >         do_ret     ( & ret(3) );
    Map<Matrix<typename Derived::Scalar, 3, 1> >         dv_ret     ( & ret(6) );

    dp_ret = dp_in;
    do_ret = log_q(dq_in);
    dv_ret = dv_in;

    return ret;
}

template<typename Derived>
Matrix<typename Derived::Scalar, 10, 1> retract(const MatrixBase<Derived>& d_in)
{
    MatrixSizeCheck<9, 1>::check(d_in);

    Matrix<typename Derived::Scalar, 10, 1> ret;

    Map<const Matrix<typename Derived::Scalar, 3, 1> >   dp_in  ( &d_in(0) );
    Map<const Matrix<typename Derived::Scalar, 3, 1> >   do_in  ( &d_in(3) );
    Map<const Matrix<typename Derived::Scalar, 3, 1> >   dv_in  ( &d_in(6) );
    Map<Matrix<typename Derived::Scalar, 3, 1> >         dp     ( & ret(0) );
    Map<Quaternion<typename Derived::Scalar> >           dq     ( & ret(3) );
    Map<Matrix<typename Derived::Scalar, 3, 1> >         dv     ( & ret(7) );

    dp = dp_in;
    dq = exp_q(do_in);
    dv = dv_in;

    return ret;
}

template<typename D1, typename D2, typename D3>
inline void compare(const MatrixBase<D1>& d1,
                  const MatrixBase<D2>& d2,
                  MatrixBase<D3>& err)
{
    Matrix<typename D3::Scalar, 10, 1> delta_err;

    between(d1, d2, 0.0, delta_err);

    err = lift(delta_err);
}



} // namespace imu
} // namespace wolf

#endif /* IMU_TOOLS_H_ */

/*
 * gtest_imu_tools.cpp
 *
 *  Created on: Jul 29, 2017
 *      Author: jsola
 */

#include "imu_tools.h"

#include "utils_gtest.h"


using namespace Eigen;
using namespace wolf;
using namespace imu;

TEST(IMU_tools, identity)
{
    VectorXs id_true;
    id_true.setZero(10);
    id_true(6) = 1.0;

    VectorXs id = identity<Scalar>();
    ASSERT_MATRIX_APPROX(id, id_true, 1e-10);
}

TEST(IMU_tools, identity_split)
{
    VectorXs p(3), qv(4), v(3);
    Quaternions q;

    identity(p,q,v);
    ASSERT_MATRIX_APPROX(p, Vector3s::Zero(), 1e-10);
    ASSERT_QUATERNION_APPROX(q, Quaternions::Identity(), 1e-10);
    ASSERT_MATRIX_APPROX(v, Vector3s::Zero(), 1e-10);

    identity(p,qv,v);
    ASSERT_MATRIX_APPROX(p , Vector3s::Zero(), 1e-10);
    ASSERT_MATRIX_APPROX(qv, (Vector4s()<<0,0,0,1).finished(), 1e-10);
    ASSERT_MATRIX_APPROX(v , Vector3s::Zero(), 1e-10);
}

TEST(IMU_tools, inverse)
{
    VectorXs d(10), id(10), iid(10), iiid(10), I(10);
    Vector4s qv;
    Scalar dt = 0.1;

    qv = (Vector4s() << 3, 4, 5, 6).finished().normalized();
    d << 0, 1, 2, qv, 7, 8, 9;

    id   = inverse(d, dt);

    compose(id, d, dt, I);
    ASSERT_MATRIX_APPROX(I, identity(), 1e-10);
    compose(d, id, -dt, I); // Observe -dt is reversed !!
    ASSERT_MATRIX_APPROX(I, identity(), 1e-10);

    inverse(id, -dt, iid); // Observe -dt is reversed !!
    ASSERT_MATRIX_APPROX( d,  iid, 1e-10);
    iiid = inverse(iid, dt);
    ASSERT_MATRIX_APPROX(id, iiid, 1e-10);
}

TEST(IMU_tools, compose_between)
{
    VectorXs dx1(10), dx2(10), dx3(10);
    Matrix<Scalar, 10, 1> d1, d2, d3;
    Vector4s qv;
    Scalar dt = 0.1;

    qv = (Vector4s() << 3, 4, 5, 6).finished().normalized();
    dx1 << 0, 1, 2, qv, 7, 8, 9;
    d1 = dx1;
    qv = (Vector4s() << 6, 5, 4, 3).finished().normalized();
    dx2 << 9, 8, 7, qv, 2, 1, 0;
    d2 = dx2;

    // combinations of templates for sum()
    compose(dx1, dx2, dt, dx3);
    compose(d1, d2, dt, d3);
    ASSERT_MATRIX_APPROX(d3, dx3, 1e-10);

    compose(d1, dx2, dt, dx3);
    d3 = compose(dx1, d2, dt);
    ASSERT_MATRIX_APPROX(d3, dx3, 1e-10);

    // minus(d1, d3) should recover delta_2
    VectorXs diffx(10);
    Matrix<Scalar,10,1> diff;
    between(d1, d3, dt, diff);
    ASSERT_MATRIX_APPROX(diff, d2, 1e-10);

    // minus(d3, d1, -dt) should recover inverse(d2, dt)
    diff = between(d3, d1, -dt);
    ASSERT_MATRIX_APPROX(diff, inverse(d2, dt), 1e-10);
}

TEST(IMU_tools, compose_between_with_state)
{
    VectorXs x(10), d(10), x2(10), x3(10), d2(10), d3(10);
    Vector4s qv;
    Scalar dt = 0.1;

    qv = (Vector4s() << 3, 4, 5, 6).finished().normalized();
    x << 0, 1, 2, qv, 7, 8, 9;
    qv = (Vector4s() << 6, 5, 4, 3).finished().normalized();
    d << 9, 8, 7, qv, 2, 1, 0;

    composeOverState(x, d, dt, x2);
    x3 = composeOverState(x, d, dt);
    ASSERT_MATRIX_APPROX(x3, x2, 1e-10);

    // betweenStates(x, x2) should recover d
    betweenStates(x, x2, dt, d2);
    d3 = betweenStates(x, x2, dt);
    ASSERT_MATRIX_APPROX(d2, d, 1e-10);
    ASSERT_MATRIX_APPROX(d3, d, 1e-10);
    ASSERT_MATRIX_APPROX(betweenStates(x, x2, dt), d, 1e-10);

    // x + (x2 - x) = x2
    ASSERT_MATRIX_APPROX(composeOverState(x, betweenStates(x, x2, dt), dt), x2, 1e-10);

    // (x + d) - x = d
    ASSERT_MATRIX_APPROX(betweenStates(x, composeOverState(x, d, dt), dt), d, 1e-10);
}

TEST(IMU_tools, lift_retract)
{
    VectorXs d_min(9); d_min << 0, 1, 2, .3, .4, .5, 6, 7, 8; // use angles in the ball theta < pi

    // transform to delta
    VectorXs delta = retract(d_min);

    // expected delta
    Vector3s dp = d_min.head(3);
    Quaternions dq = v2q(d_min.segment(3,3));
    Vector3s dv = d_min.tail(3);
    VectorXs delta_true(10); delta_true << dp, dq.coeffs(), dv;
    ASSERT_MATRIX_APPROX(delta, delta_true, 1e-10);

    // transform to a new tangent -- should be the original tangent
    VectorXs d_from_delta = lift(delta);
    ASSERT_MATRIX_APPROX(d_from_delta, d_min, 1e-10);

    // transform to a new delta -- should be the previous delta
    VectorXs delta_from_d = retract(d_from_delta);
    ASSERT_MATRIX_APPROX(delta_from_d, delta, 1e-10);
}

TEST(IMU_tools, diff)
{
    VectorXs d1(10), d2(10);
    Vector4s qv = (Vector4s() << 3, 4, 5, 6).finished().normalized();
    d1 << 0, 1, 2, qv, 7, 8, 9;
    d2 = d1;

    VectorXs err(9);
    diff(d1, d2, err);
    ASSERT_MATRIX_APPROX(err, VectorXs::Zero(9), 1e-10);
    ASSERT_MATRIX_APPROX(diff(d1, d2), VectorXs::Zero(9), 1e-10);
}

TEST(IMU_tools, compose_jacobians)
{
    VectorXs d1(10), d2(10), d3(10), d1_pert(10), d2_pert(10), d3_pert(10); // deltas
    VectorXs t1(9), t2(9); // tangents
    Matrix<Scalar, 9, 9> J1_a, J2_a, J1_n, J2_n;
    Vector4s qv1, qv2;
    Scalar dt = 0.1;
    Scalar dx = 1e-6;
    qv1 = (Vector4s() << 3, 4, 5, 6).finished().normalized();
    d1 << 0, 1, 2, qv1, 7, 8, 9;
    qv2 = (Vector4s() << 1, 2, 3, 4).finished().normalized();
    d2 << 9, 8, 7, qv2, 2, 1, 0;

    // analytical jacobians
    compose(d1, d2, dt, d3, J1_a, J2_a);

    // numerical jacobians
    for (unsigned int i = 0; i<9; i++)
    {
        t1      . setZero();
        t1(i)   = dx;

        // Jac wrt first delta
        d1_pert = plus(d1, t1);                 //     (d1 + t1)
        d3_pert = compose(d1_pert, d2, dt);     //     (d1 + t1) + d2
        t2      = diff(d3, d3_pert);            //   { (d2 + t1) + d2 } - { d1 + d2 }
        J1_n.col(i) = t2/dx;                    // [ { (d2 + t1) + d2 } - { d1 + d2 } ] / dx

        // Jac wrt second delta
        d2_pert = plus(d2, t1);                 //          (d2 + t1)
        d3_pert = compose(d1, d2_pert, dt);     //     d1 + (d2 + t1)
        t2      = diff(d3, d3_pert);            //   { d1 + (d2 + t1) } - { d1 + d2 }
        J2_n.col(i) = t2/dx;                    // [ { d1 + (d2 + t1) } - { d1 + d2 } ] / dx
    }

    // check that numerical and analytical match
    ASSERT_MATRIX_APPROX(J1_a, J1_n, 1e-4);
    ASSERT_MATRIX_APPROX(J2_a, J2_n, 1e-4);
}

TEST(IMU_tools, body2delta_jacobians)
{
    VectorXs delta(10), delta_pert(10); // deltas
    VectorXs body(6), pert(6);
    VectorXs tang(9); // tangents
    Matrix<Scalar, 9, 6> J_a, J_n; // analytic and numerical jacs
    Vector4s qv;;
    Scalar dt = 0.1;
    Scalar dx = 1e-6;
    qv = (Vector4s() << 3, 4, 5, 6).finished().normalized();
    delta << 0, 1, 2,   qv,   7, 8, 9;
    body << 1, 2, 3,   3, 2, 1;

    // analytical jacobians
    body2delta(body, dt, delta, J_a);

    // numerical jacobians
    for (unsigned int i = 0; i<6; i++)
    {
        pert      . setZero();
        pert(i)   = dx;

        // Jac wrt first delta
        delta_pert = body2delta(body + pert, dt);   //     delta(body + pert)
        tang       = diff(delta, delta_pert);       //   delta(body + pert) -- delta(body)
        J_n.col(i) = tang/dx;                       // ( delta(body + pert) -- delta(body) ) / dx
    }

    // check that numerical and analytical match
    ASSERT_MATRIX_APPROX(J_a, J_n, 1e-4);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}


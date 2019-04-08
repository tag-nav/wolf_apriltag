/**
 * \file gtest_rotation.cpp
 *
 *  Created on: Oct 13, 2016
 *      \author: AtDinesh
 */

//Eigen
#include <Eigen/Geometry>

//Wolf
#include "base/wolf.h"
#include "base/rotations.h"

//std
#include <iostream>
#include <fstream>
#include <iomanip>
#include <ctime>
#include <cmath>
#include "utils_gtest.h"

//#define write_results

// THESE ARE UNITARY TESTS FOR METHODS IN ROTATION.H

using namespace wolf;
using namespace Eigen;

TEST(exp_q, unit_norm)
{
    Vector3s v0  = Vector3s::Random();
    Scalar scale = 1.0;
    for (int i = 0; i < 20; i++)
    {
        Vector3s v = v0 * scale;
        Quaternions q = exp_q(v);
        ASSERT_NEAR(q.norm(), 1.0, 1e-10) << "Failed at scale 1e-" << i << " with angle = " << 2.0*q.vec().norm();
        scale /= 10;
    }
}

TEST(rotations, pi2pi)
{
    ASSERT_NEAR(M_PI_2, pi2pi((Scalar)M_PI_2), 1e-10);
    ASSERT_NEAR(-M_PI_2, pi2pi(3.0*M_PI_2), 1e-10);
    ASSERT_NEAR(-M_PI_2, pi2pi(-M_PI_2), 1e-10);
    ASSERT_NEAR(M_PI_2, pi2pi(-3.0*M_PI_2), 1e-10);
    //    ASSERT_NEAR(M_PI, pi2pi(M_PI), 1e-10); // Exact PI is not safely testable because of numeric issues.
    ASSERT_NEAR(M_PI-.01, pi2pi(M_PI-.01), 1e-10);
    ASSERT_NEAR(-M_PI+.01, pi2pi(M_PI+.01), 1e-10);
}

TEST(skew, Skew_vee)
{
    using namespace wolf;
    Vector3s vec3 = Vector3s::Random();
    Matrix3s skew_mat;
    skew_mat = skew(vec3);

    // vee
    Vector3s vec3_bis;
    vec3_bis = vee(skew_mat);

    ASSERT_TRUE(vec3_bis == vec3);
}

TEST(exp_q, v2q_q2v)
{
    using namespace wolf;
    //defines scalars
    wolf::Scalar deg_to_rad = M_PI/180.0;

    Vector4s vec0, vec1;

    //v2q
    Vector3s rot_vector0, rot_vector1;
    rot_vector0 = Vector3s::Random();
    rot_vector1 = rot_vector0 * 100 *deg_to_rad; //far from origin
    rot_vector0 = rot_vector0*deg_to_rad;

    Quaternions quat0, quat1;
    quat0 = v2q(rot_vector0);
    quat1 = v2q(rot_vector1);

    //q2v
    Vector3s quat_to_v0, quat_to_v1;
    VectorXs quat_to_v0x, quat_to_v1x;

    quat_to_v0 = q2v(quat0);
    quat_to_v1 = q2v(quat1);
    quat_to_v0x = q2v(quat0);
    quat_to_v1x = q2v(quat1);

    ASSERT_MATRIX_APPROX(rot_vector0, quat_to_v0, wolf::Constants::EPS);
    ASSERT_MATRIX_APPROX(rot_vector1, quat_to_v1, wolf::Constants::EPS);
    ASSERT_MATRIX_APPROX(rot_vector0, quat_to_v0x, wolf::Constants::EPS);
    ASSERT_MATRIX_APPROX(rot_vector1, quat_to_v1x, wolf::Constants::EPS);
}

TEST(exp_R, v2R_R2v)
{
    using namespace wolf;
    //First test is to verify we get the good result with v -> v2R -> R2v -> v
    //test 2 : how small can angles in rotation vector be ?

    //definition
    wolf::Scalar deg_to_rad = M_PI/180.0;
    Vector3s rot_vector0, rot_vector1;

    rot_vector0 = Vector3s::Random();
    rot_vector1 = rot_vector0 * 100 *deg_to_rad; //far from origin
    rot_vector0 = rot_vector0*deg_to_rad;

    Matrix3s rot0, rot1;
    rot0 = v2R(rot_vector0);
    rot1 = v2R(rot_vector1);

    //R2v
    Vector3s rot0_vec, rot1_vec;
    rot0_vec = R2v(rot0);
    rot1_vec = R2v(rot1);

    //check now
    ASSERT_MATRIX_APPROX(rot0_vec, rot_vector0, wolf::Constants::EPS);
    ASSERT_MATRIX_APPROX(rot1_vec, rot_vector1, wolf::Constants::EPS);
}

TEST(log_R, R2v_v2R_limits)
{
    using namespace wolf;
    //test 2 : how small can angles in rotation vector be ?
    wolf::Scalar scale = 1;
    Matrix3s v_to_R, initial_matrix;
    Vector3s  R_to_v;

    //Vector3s rv;
    for(int i = 0; i<8; i++){
        initial_matrix = v2R(Vector3s::Random().eval() * scale);

        R_to_v = R2v(initial_matrix);     
        v_to_R = v2R(R_to_v);

        ASSERT_MATRIX_APPROX(v_to_R, initial_matrix, wolf::Constants::EPS);
        scale = scale*0.1;
    }
}

TEST(log_R, R2v_v2R_AAlimits)
{
    using namespace wolf;
    //let's see how small the angles can be here : limit reached at scale/10 =  1e-16
    wolf::Scalar scale = 1;
    Matrix3s rotation_mat;
    Vector3s rv;

    for(int i = 0; i<8; i++){
        rotation_mat = v2R(Vector3s::Random().eval() * scale);

        //rv = R2v(rotation_mat); //decomposing R2v below
        AngleAxis<wolf::Scalar> aa0 = AngleAxis<wolf::Scalar>(rotation_mat);
        rv = aa0.axis() * aa0.angle();
        //std::cout << "aa0.axis : " << aa0.axis().transpose() << ",\t aa0.angles :" << aa0.angle() <<std::endl;

        ASSERT_FALSE(rv == Vector3s::Zero());
        scale = scale*0.1;
    }
}

TEST(exp_q, v2q2R2v)
{
    using namespace wolf;
    wolf::Scalar scale = 1;
    // testing new path : vector -> quaternion -> matrix -> vector

    for(int i = 0; i< 8; i++){
        Vector3s vector_ = Vector3s::Random()*scale;
        Quaternions quat_ = v2q(vector_);
        Matrix3s mat_ = quat_.matrix();
        Vector3s vector_bis = R2v(mat_);

        ASSERT_MATRIX_APPROX(vector_, vector_bis, wolf::Constants::EPS);
        scale = scale*0.1;
    }
}

TEST(rotations, AngleAxis_limits)
{
    using namespace wolf;
    //Hypothesis : problem with construction of AngleAxis objects.
    // Example : if R = I + [delta t]_x (happenning in the IMU case with delta t = 0.001). Then angle mays be evaluated as 0 (due to cosinus definition ?) 
    // Here we try to get the AngleAxis object from a random rotation matrix, then get back to the rotation matrix using AngleAxis::toRotationMatrix()

    wolf::Scalar scale = 1;
    Matrix3s res, res_i, rotation_mati, rotation_mat;
    Vector3s rv;

    for(int i = 0; i<8; i++){ //FIX ME : Random() will not create a rotation matrix. Then, R2v(Random_matrix()) makes no sense at all.
        rotation_mat = v2R(Vector3s::Random().eval() * scale);
        rotation_mati = rotation_mat;

        //rv = R2v(rotation_mat); //decomposing R2v below
        AngleAxis<wolf::Scalar> aa0 = AngleAxis<wolf::Scalar>(rotation_mat);
        rv = aa0.axis() * aa0.angle();
        //std::cout << "aa0.axis : " << aa0.axis().transpose() << ",\t aa0.angles :" << aa0.angle() <<std::endl;
        res = aa0.toRotationMatrix();

        // now we set the diagonal to identity
        AngleAxis<wolf::Scalar> aa1 = AngleAxis<wolf::Scalar>(rotation_mat);
        rv = aa1.axis() * aa1.angle();
        //std::cout << "aa1.axis : " << aa0.axis().transpose() << ",\t aa1.angles :" << aa0.angle() <<std::endl;
        res_i = aa1.toRotationMatrix();

        ASSERT_MATRIX_APPROX(res, rotation_mat, wolf::Constants::EPS);
        ASSERT_MATRIX_APPROX(res_i, rotation_mati, wolf::Constants::EPS);
        scale = scale*0.1;
    }
}

TEST(compose, Quat_compos_const_rateOfTurn)
{
    using namespace wolf;

                                // ********* constant rate of turn *********

    /* First idea was to integrate data on SO3 (succesive composition of quaternions : q = q * dq(w*dt) <=> q = q * dq(w*dt) * q' (mathematically)) and in R3
    (q2v(v2q(v0*n*dt))). with v0 << 30.0*deg_to_rad, 5.0*deg_to_rad, 10.0*deg_to_rad : constant rate-of-turn in rad/s and dt the time step.
    But this is not OK, we cannot expect those 2 rotation integration to be equal.
    The whole point of the SO3 thing is that we cannot integrate rotation in the R3 space and expect it to work. This is why we must integrate it in the manifold of SO3
    
    more specifically : 
    - At a constant velocity, because we keep a constant rotation axis, the integral is the same.
    - for non-contant velocities, especially if we change the axis of rotation, then it’s not the same, and the only good method is the SO3.

    We change the idea :
    define orientation and derive ox, oy, oz so that we get the rate of turn wx, wy, wz.
    Then compare the final orientation from rotation matrix composition and quaternion composition
    */

    wolf::Scalar deg_to_rad = M_PI/180.0;
    Eigen::Matrix3s rot0(Eigen::Matrix3s::Identity());
    Eigen::Quaternions q0, qRot;
    q0.setIdentity();
    Eigen::Vector3s tmp_vec; //will be used to store rate of turn data

    const unsigned int x_rot_vel = 5;   // deg/s
    const unsigned int y_rot_vel = 2;   // deg/s
    const unsigned int z_rot_vel = 10;  // deg/s

    wolf::Scalar tmpx, tmpy, tmpz;
    /*
        ox oy oz evolution in degrees (for understanding) --> converted in rad
        with * pi/180
        ox = x_rot_vel * t; %express angle in rad before using sinus
        oy = y_rot_vel * t;
        oz = z_rot_vel * t;

        corresponding rate of turn
        %rate of turn expressed in radians/s
        wx = x_rot_vel;
        wy = y_rot_vel;
        wz = z_rot_vel;
     */

     //there is no need to compute the rate of turn at each time because it is constant here : 
    tmpx = deg_to_rad * x_rot_vel;  // rad/s
    tmpy = deg_to_rad * y_rot_vel;
    tmpz = deg_to_rad * z_rot_vel;
    tmp_vec << tmpx, tmpy, tmpz;
    const wolf::Scalar dt = 0.1;

    for(unsigned int data_iter = 0; data_iter < 100; data_iter ++)
    {   
        rot0 = rot0 * v2R(tmp_vec*dt);
        q0 = q0 * v2q(tmp_vec*dt); //succesive composition of quaternions : q = q * dq(w*dt) <=> q = q * dq(w*dt) * q' (mathematically)

    }

    // Compare results from rotation matrix composition and quaternion composition
     qRot = (v2q(R2v(rot0)));
     
     Eigen::Vector3s final_orientation(q2v(qRot));
     ASSERT_TRUE((final_orientation - wolf::q2v(q0)).isMuchSmallerThan(1,wolf::Constants::EPS)) << "final orientation expected : " << final_orientation.transpose() << 
     "\n computed final orientation : " << wolf::q2v(q0).transpose() << std::endl;
}

TEST(compose, Quat_compos_var_rateOfTurn)
{
    using namespace wolf;

                                //********* changing rate of turn - same freq for all axis *********

    /* First idea was to integrate data on SO3 (succesive composition of quaternions : q = q * dq(w*dt) <=> q = q * dq(w*dt) * q' (mathematically)) and in R3
    (q2v(v2q(v0*n*dt))). with v0 << 30.0*deg_to_rad, 5.0*deg_to_rad, 10.0*deg_to_rad : constant rate-of-turn in rad/s and dt the time step.
    But this is not OK, we cannot expect those 2 rotation integration to be equal.
    The whole point of the SO3 thing is that we cannot integrate rotation in the R3 space and expect it to work. This is why we must integrate it in the manifold of SO3
    
    more specifically : 
    - At a constant velocity, because we keep a constant rotation axis, the integral is the same.
    - for non-contant velocities, especially if we change the axis of rotation, then it’s not the same, and the only good method is the SO3.

    We change the idea :
    define orientation and derive ox, oy, oz so that we get the rate of turn wx, wy, wz.
    Then compare the final orientation from ox, oy, oz and quaternion we get by data integration

     ******* RESULT : ******* 
    The error in this test is due to discretization. The smaller is dt and the better is the integration !
    with dt = 0.001, the error is in 1e-5
    */

    wolf::Scalar deg_to_rad = M_PI/180.0;
    Eigen::Matrix3s rot0(Eigen::Matrix3s::Identity());
    Eigen::Quaternions q0, qRot;
    q0.setIdentity();

    Eigen::Vector3s tmp_vec; //will be used to store rate of turn data
    wolf::Scalar time = 0;    
    const unsigned int x_rot_vel = 15;   // deg/s
    const unsigned int y_rot_vel = 15;   // deg/s
    const unsigned int z_rot_vel = 15;  // deg/s

    wolf::Scalar tmpx, tmpy, tmpz;
    /*
        ox oy oz evolution in degrees (for understanding) --> converted in rad
        with * pi/180
        ox = pi*sin(x_rot_vel * t * pi/180); %express angle in rad before using sinus
        oy = pi*sin(y_rot_vel * t * pi/180);
        oz = pi*sin(z_rot_vel * t * pi/180);

        corresponding rate of turn
        %rate of turn expressed in radians/s
        wx = pi * x_rot_vel * cos(x_rot_vel * t * pi/180) * pi/180;
        wy = pi * y_rot_vel * cos(y_rot_vel * t * pi/180) * pi/180;
        wz = pi * z_rot_vel * cos(z_rot_vel * t * pi/180) * pi/180;
     */

    const wolf::Scalar dt = 0.001;

    for(unsigned int data_iter = 0; data_iter <= 10000; data_iter ++)
    {   
        tmpx = M_PI*x_rot_vel*cos(wolf::toRad(x_rot_vel * time))*deg_to_rad;
        tmpy = M_PI*y_rot_vel*cos(wolf::toRad(y_rot_vel * time))*deg_to_rad;
        tmpz = M_PI*z_rot_vel*cos(wolf::toRad(z_rot_vel * time))*deg_to_rad;
        tmp_vec << tmpx, tmpy, tmpz;

        rot0 = rot0 * v2R(tmp_vec*dt);
        q0 = q0 * v2q(tmp_vec*dt); //succesive composition of quaternions : q = q * dq(w*dt) <=> q = q * dq(w*dt) * q' (mathematically)

        time += dt;
    }

    // Compare results from rotation matrix composition and quaternion composition
    qRot = (v2q(R2v(rot0)));
     
    Eigen::Vector3s final_orientation(q2v(qRot));
     
    EXPECT_TRUE((final_orientation - wolf::q2v(q0)).isMuchSmallerThan(1,wolf::Constants::EPS)) << "final orientation expected : " << final_orientation.transpose() << 
    "\n computed final orientation : " << wolf::q2v(q0).transpose() << std::endl;
    ASSERT_TRUE((final_orientation - wolf::q2v(q0)).isMuchSmallerThan(1,0.0001)) << "final orientation expected : " << final_orientation.transpose() << 
    "\n computed final orientation : " << wolf::q2v(q0).transpose() << std::endl;

}

TEST(compose, Quat_compos_var_rateOfTurn_diff)
{
    using namespace wolf;

    //      ********* changing rate of turn - different freq for 1 axis *********

    /* First idea was to integrate data on SO3 (succesive composition of quaternions : q = q * dq(w*dt) <=> q = q * dq(w*dt) * q' (mathematically)) and in R3
    (q2v(v2q(v0*n*dt))). with v0 << 30.0*deg_to_rad, 5.0*deg_to_rad, 10.0*deg_to_rad : constant rate-of-turn in rad/s and dt the time step.
    But this is not OK, we cannot expect those 2 rotation integration to be equal.
    The whole point of the SO3 thing is that we cannot integrate rotation in the R3 space and expect it to work. This is why we must integrate it in the manifold of SO3
    
    more specifically : 
    - At a constant velocity, because we keep a constant rotation axis, the integral is the same.
    - for non-contant velocities, especially if we change the axis of rotation, then it’s not the same, and the only good method is the SO3.

    We change the idea :
    define orientation and derive ox, oy, oz so that we get the rate of turn wx, wy, wz.
    Then compare the final orientation from ox, oy, oz and quaternion we get by data integration

    ******* RESULT : ******* 
    Things are more tricky here. The errors go growing with time.
    with dt = 0.001, the error is in 1e-4 for 1 s integration ! But this may also depend on the frequency given to the rotation on each of the axis.
    */

    wolf::Scalar deg_to_rad = M_PI/180.0;
    Eigen::Matrix3s rot0(Eigen::Matrix3s::Identity());
    Eigen::Quaternions q0, qRot;
    q0.setIdentity();

    Eigen::Vector3s tmp_vec; //will be used to store rate of turn data
    wolf::Scalar time = 0;    
    const unsigned int x_rot_vel = 1;   // deg/s
    const unsigned int y_rot_vel = 3;   // deg/s
    const unsigned int z_rot_vel = 6;  // deg/s

    wolf::Scalar tmpx, tmpy, tmpz;
    /*
        ox oy oz evolution in degrees (for understanding) --> converted in rad
        with * pi/180
        ox = pi*sin(x_rot_vel * t * pi/180); %express angle in rad before using sinus
        oy = pi*sin(y_rot_vel * t * pi/180);
        oz = pi*sin(z_rot_vel * t * pi/180);

        corresponding rate of turn
        %rate of turn expressed in radians/s
        wx = pi * x_rot_vel * cos(x_rot_vel * t * pi/180) * pi/180;
        wy = pi * y_rot_vel * cos(y_rot_vel * t * pi/180) * pi/180;
        wz = pi * z_rot_vel * cos(z_rot_vel * t * pi/180) * pi/180;
     */

    const wolf::Scalar dt = 0.001;

    for(unsigned int data_iter = 0; data_iter <= 1000; data_iter ++)
    {   
        tmpx = M_PI*x_rot_vel*cos(wolf::toRad(x_rot_vel * time))*deg_to_rad;
        tmpy = M_PI*y_rot_vel*cos(wolf::toRad(y_rot_vel * time))*deg_to_rad;
        tmpz = M_PI*z_rot_vel*cos(wolf::toRad(z_rot_vel * time))*deg_to_rad;
        tmp_vec << tmpx, tmpy, tmpz;

        rot0 = rot0 * v2R(tmp_vec*dt);
        q0 = q0 * v2q(tmp_vec*dt); //succesive composition of quaternions : q = q * dq(w*dt) <=> q = q * dq(w*dt) * q' (mathematically)

        time += dt;
    }

    // Compare results from rotation matrix composition and quaternion composition
    qRot = (v2q(R2v(rot0)));
     
    Eigen::Vector3s final_orientation(q2v(qRot));

    EXPECT_TRUE((final_orientation - wolf::q2v(q0)).isMuchSmallerThan(1,wolf::Constants::EPS)) << "final orientation expected : " << final_orientation.transpose() << 
    "\n computed final orientation : " << wolf::q2v(q0).transpose() << std::endl;
     
    ASSERT_TRUE((final_orientation - wolf::q2v(q0)).isMuchSmallerThan(1,0.001)) << "final orientation expected : " << final_orientation.transpose() << 
    "\n computed final orientation : " << wolf::q2v(q0).transpose() << std::endl;
}

TEST(Plus, Random)
{
    Quaternions q;
    q               .coeffs().setRandom().normalize();

    Vector3s v;
    v               .setRandom();

    Quaternions q2  = q * exp_q(v);
    Quaternions q3  = exp_q(v) * q;

    ASSERT_QUATERNION_APPROX(plus(q,v)      , q2, 1e-12);
    ASSERT_QUATERNION_APPROX(plus_right(q,v), q2, 1e-12);
    ASSERT_QUATERNION_APPROX(plus_left(v,q) , q3, 1e-12);

}

TEST(Plus, Identity_plus_small)
{
    Quaternions q;
    q               .setIdentity();

    Vector3s v;
    v               .setRandom();
    v              *= 1e-6;

    Quaternions q2;
    q2.w()          = 1;
    q2.vec()        = 0.5*v;

    ASSERT_QUATERNION_APPROX(plus(q,v), q2, 1e-12);
}

TEST(Minus_and_diff, Random)
{
    Quaternions q1, q2, qo;
    q1              .coeffs().setRandom().normalize();
    q2              .coeffs().setRandom().normalize();

    Vector3s vr      = log_q(q1.conjugate() * q2);
    Vector3s vl      = log_q(q2 * q1.conjugate());

    ASSERT_MATRIX_APPROX(minus(q1, q2), vr, 1e-12);
    ASSERT_MATRIX_APPROX(diff(q1, q2), vr, 1e-12);
    ASSERT_MATRIX_APPROX(minus_left(q1, q2), vl, 1e-12);

    qo = plus(q1, minus(q1, q2));
    if (q2.w() * qo.w() < 0) q2.coeffs() = -(q2.coeffs()); // allow q = -q
    ASSERT_QUATERNION_APPROX(qo, q2, 1e-12);

    qo = plus(q1, diff(q1, q2));
    if (q2.w() * qo.w() < 0) q2.coeffs() = -(q2.coeffs()); // allow q = -q
    ASSERT_QUATERNION_APPROX(qo, q2, 1e-12);

    qo = plus_left(minus_left(q1, q2), q1);
    if (q2.w() * qo.w() < 0) q2.coeffs() = -(q2.coeffs()); // allow q = -q
    ASSERT_QUATERNION_APPROX(qo, q2, 1e-12);
}

TEST(Jacobians, Jr)
{
    Vector3s theta; theta.setRandom();
    Vector3s dtheta; dtheta.setRandom(); dtheta *= 1e-4;

    // Check the main Jr property for q and R
    // exp( theta + d_theta ) \approx exp(theta) * exp(Jr * d_theta)
    Matrix3s Jr = jac_SO3_right(theta);
    ASSERT_QUATERNION_APPROX(exp_q(theta+dtheta), exp_q(theta) * exp_q(Jr*dtheta), 1e-8);
    ASSERT_MATRIX_APPROX(exp_R(theta+dtheta), (exp_R(theta) * exp_R(Jr*dtheta)), 1e-8);
}

TEST(Jacobians, Jl)
{
    Vector3s theta; theta.setRandom();
    Vector3s dtheta; dtheta.setRandom(); dtheta *= 1e-4;

    // Check the main Jl property for q and R
    // exp( theta + d_theta ) \approx exp(Jl * d_theta) * exp(theta)
    Matrix3s Jl = jac_SO3_left(theta);
    ASSERT_QUATERNION_APPROX(exp_q(theta+dtheta), exp_q(Jl*dtheta) * exp_q(theta), 1e-8);
    ASSERT_MATRIX_APPROX(exp_R(theta+dtheta), (exp_R(Jl*dtheta) * exp_R(theta)), 1e-8);

    // Jl = Jr.tr
    ASSERT_MATRIX_APPROX(Jl, jac_SO3_right(theta).transpose(), 1e-8);

    // Jl = R*Jr
    ASSERT_MATRIX_APPROX(Jl, exp_R(theta)*jac_SO3_right(theta), 1e-8);
}

TEST(Jacobians, Jr_inv)
{
    Vector3s theta; theta.setRandom();
    Vector3s dtheta; dtheta.setRandom(); dtheta *= 1e-4;
    Quaternions q = v2q(theta);
    Matrix3s    R = v2R(theta);

    // Check the main Jr_inv property for q and R
    // log( R * exp(d_theta) ) \approx log( R ) + Jrinv * d_theta
    Matrix3s Jr_inv = jac_SO3_right_inv(theta);
    ASSERT_MATRIX_APPROX(log_q(q * exp_q(dtheta)), log_q(q) + Jr_inv*dtheta, 1e-8);
    ASSERT_MATRIX_APPROX(log_R(R * exp_R(dtheta)), log_R(R) + Jr_inv*dtheta, 1e-8);
}

TEST(Jacobians, Jl_inv)
{
    Vector3s theta; theta.setRandom();
    Vector3s dtheta; dtheta.setRandom(); dtheta *= 1e-4;
    Quaternions q = v2q(theta);
    Matrix3s    R = v2R(theta);

    // Check the main Jl_inv property for q and R
    // log( exp(d_theta) * R ) \approx log( R ) + Jlinv * d_theta
    Matrix3s Jl_inv = jac_SO3_left_inv(theta);
    ASSERT_MATRIX_APPROX(log_q(exp_q(dtheta) * q), log_q(q) + Jl_inv*dtheta, 1e-8);
    ASSERT_MATRIX_APPROX(log_R(exp_R(dtheta) * R), log_R(R) + Jl_inv*dtheta, 1e-8);
}

TEST(Jacobians, compose)
{

    Vector3s th1(.1,.2,.3), th2(.3,.1,.2);
    Quaternions q1(exp_q(th1));
    Quaternions q2(exp_q(th2));
    Quaternions qc;
    Matrix3s J1a, J2a, J1n, J2n;

    // composition and analytic Jacobians
    wolf::compose(q1, q2, qc, J1a, J2a);

    // Numeric Jacobians
    Scalar dx = 1e-6;
    Vector3s pert;
    Quaternions q1_pert, q2_pert, qc_pert;
    for (int i = 0; i<3; i++)
    {
        pert.setZero();
        pert(i) = dx;

        // Jac wrt q1
        q1_pert     = q1*exp_q(pert);
        qc_pert     = q1_pert * q2;
        J1n.col(i)  = log_q(qc.conjugate()*qc_pert) / dx;

        // Jac wrt q2
        q2_pert     = q2*exp_q(pert);
        qc_pert     = q1 * q2_pert;
        J2n.col(i)  = log_q(qc.conjugate()*qc_pert) / dx;
    }

    ASSERT_MATRIX_APPROX(J1a, J1n, 1e-5);
    ASSERT_MATRIX_APPROX(J2a, J2n, 1e-5);
}

TEST(Jacobians, between)
{

    Vector3s th1(.1,.2,.3), th2(.3,.1,.2);
    Quaternions q1(exp_q(th1));
    Quaternions q2(exp_q(th2));
    Quaternions qc;
    Matrix3s J1a, J2a, J1n, J2n;

    // composition and analytic Jacobians
    wolf::between(q1, q2, qc, J1a, J2a);

    // Numeric Jacobians
    Scalar dx = 1e-6;
    Vector3s pert;
    Quaternions q1_pert, q2_pert, qc_pert;
    for (int i = 0; i<3; i++)
    {
        pert.setZero();
        pert(i) = dx;

        // Jac wrt q1
        q1_pert     = q1*exp_q(pert);
        qc_pert     = q1_pert.conjugate() * q2;
        J1n.col(i)  = log_q(qc.conjugate()*qc_pert) / dx;

        // Jac wrt q2
        q2_pert     = q2*exp_q(pert);
        qc_pert     = q1.conjugate() * q2_pert;
        J2n.col(i)  = log_q(qc.conjugate()*qc_pert) / dx;
    }

    ASSERT_MATRIX_APPROX(J1a, J1n, 1e-5);
    ASSERT_MATRIX_APPROX(J2a, J2n, 1e-5);
}

TEST(exp_q, small)
{
    Vector3s u; u.setRandom().normalize();
    Vector3s v;
    Quaternions q;
    Scalar scale = 1.0;
    for (int i = 0; i<20; i++)
    {
        v               = u*scale;
        q               = exp_q(v);
        Vector3s ratio  = q.vec().array() / v.array();

        WOLF_TRACE("scale = ", scale, "; ratio = ", ratio.transpose());

        scale          /= 10;
    }
    ASSERT_MATRIX_APPROX(q.vec()/(10*scale), u/2, 1e-12);
}

TEST(log_q, double_cover)
{
    Quaternions qp; qp.coeffs().setRandom().normalize();
    Quaternions qn; qn.coeffs() = - qp.coeffs();
    ASSERT_MATRIX_APPROX(log_q(qp), log_q(qn), 1e-16);
}

TEST(log_q, small)
{
    Vector3s u; u.setRandom().normalize();
    Scalar scale = 1.0;
    for (int i = 0; i<20; i++)
    {
        Vector3s v      = u*scale;
        Quaternions q   = exp_q(v);
        Vector3s l      = log_q(q);

        ASSERT_MATRIX_APPROX(v, l, 1e-10);

        scale          /= 10;
    }
}

//<<<<<<< HEAD
//=======
TEST(Conversions, q2R_R2q)
{
    Vector3s v; v.setRandom();
    Quaternions q = v2q(v);
    Matrix3s R = v2R(v);

    Quaternions q_R = R2q(R);
    Quaternions qq_R(R);

    ASSERT_NEAR(q.norm(),    1, wolf::Constants::EPS);
    ASSERT_NEAR(q_R.norm(),  1, wolf::Constants::EPS);
    ASSERT_NEAR(qq_R.norm(), 1, wolf::Constants::EPS);

    ASSERT_MATRIX_APPROX(q.coeffs(), R2q(R).coeffs(), wolf::Constants::EPS);
    ASSERT_MATRIX_APPROX(q.coeffs(), qq_R.coeffs(),   wolf::Constants::EPS);
    ASSERT_MATRIX_APPROX(R,          q2R(q),          wolf::Constants::EPS);
    ASSERT_MATRIX_APPROX(R,          qq_R.matrix(),   wolf::Constants::EPS);
}

TEST(Conversions, e2q_q2e)
{
    Vector3s e, eo;
    Quaternions q;

    e << 0.1, .2, .3;

    q = e2q(e);

    eo = q2e(q);
    ASSERT_MATRIX_APPROX(eo, e, 1e-10);

    eo = q2e(q.coeffs());
    ASSERT_MATRIX_APPROX(eo, e, 1e-10);

}

//>>>>>>> master
TEST(Conversions, e2q_q2R_R2e)
{
    Vector3s e, eo;
    Quaternions q;
    Matrix3s R;

//<<<<<<< HEAD
//    e.setRandom();
//=======
//>>>>>>> master
    e << 0.1, .2, .3;
    q = e2q(e);
    R = q2R(q);

    eo = R2e(R);

//<<<<<<< HEAD
//    WOLF_TRACE("euler    ", e.transpose());
//    WOLF_TRACE("quat     ", q.coeffs().transpose());
//    WOLF_TRACE("R \n", R);
//
//    WOLF_TRACE("euler o  ", eo.transpose());
//
//
//    ASSERT_MATRIX_APPROX(eo, e, 1e-10);
//
//=======
    ASSERT_MATRIX_APPROX(eo, e, 1e-10);
}

TEST(Conversions, e2R_R2e)
{
    Vector3s e, eo;
    Matrix3s R;

    e << 0.1, 0.2, 0.3;

    R  = e2R(e);
    eo = R2e(R);
    ASSERT_MATRIX_APPROX(eo, e, 1e-10);
//>>>>>>> master
}

TEST(Conversions, e2R_R2q_q2e)
{
    Vector3s e, eo;
    Quaternions q;
    Matrix3s R;

//<<<<<<< HEAD
//    e.setRandom();
//    e << 0.1, 0.2, 0.3;
//    R = e2R(e(0), e(1), e(2));
//=======
    e << 0.1, 0.2, 0.3;
    R = e2R(e);
//>>>>>>> master
    q = R2q(R);

    eo = q2e(q.coeffs());

//<<<<<<< HEAD
//    WOLF_TRACE("euler    ", e.transpose());
//    WOLF_TRACE("R \n", R);
//    WOLF_TRACE("quat     ", q.coeffs().transpose());
//
//    WOLF_TRACE("euler o  ", eo.transpose());
//
//
//    ASSERT_MATRIX_APPROX(eo, e, 1e-10);
//
//=======
    ASSERT_MATRIX_APPROX(eo, e, 1e-10);
//>>>>>>> master
}

int main(int argc, char **argv)
{
    using namespace wolf;

    /*
        LIST OF FUNCTIONS : 
        - pi2pi                                                            
        - skew -> Skew_vee                                                        OK
        - vee  -> Skew_vee                                                        OK
        - v2q                                                v -> v2q -> q2v -> v OK (precision wolf::Constants::EPS)
        - Matrix<T, 3, 1> q2v(const Quaternion<T>& _q)       v -> v2q -> q2v -> v OK (precision wolf::Constants::EPS)
        - VectorXs q2v(const Quaternions& _q)                v -> v2q -> q2v -> v OK (precision wolf::Constants::EPS)
        - v2R
        - R2v
        - jac_SO3_right
        - jac_SO3_right_inv
        - jac_SO3_left
        - jac_SO3_left_inv
        - quaternion composition
     */

    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

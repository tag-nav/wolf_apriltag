/**
 * \file gtest_rotation.cpp
 *
 *  Created on: Oct 13, 2016
 *      \author: AtDinesh
 */

//Eigen
#include <Eigen/Geometry>

//Wolf
#include "wolf.h"
#include "../rotations.h"

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

namespace wolf
{

inline Eigen::VectorXs q2v_aa(const Eigen::Quaternions& _q)
{
    Eigen::AngleAxiss aa = Eigen::AngleAxiss(_q);
    return aa.axis() * aa.angle();
}

// 'New' version (alternative version also used by ceres)
template<typename Derived>
inline Eigen::Quaternion<typename Derived::Scalar> v2q_new(const Eigen::MatrixBase<Derived>& _v)
{
    MatrixSizeCheck<3, 1>::check(_v);
    typedef typename Derived::Scalar T;

    Eigen::Quaternion<T> q;
    const T& a0 = _v[0];
    const T& a1 = _v[1];
    const T& a2 = _v[2];
    const T angle_square = a0 * a0 + a1 * a1 + a2 * a2;

    //We need the angle : means we have to take the square root of angle_square, 
    // which is defined for all angle_square beonging to R+ (except 0)
    if (angle_square > (T)0.0 ){
        //sqrt is defined here
        const T angle = sqrt(angle_square);
        const T angle_half = angle / (T)2.0;
        
        q.w() = cos(angle_half);
        q.vec() = _v / angle * sin(angle_half);
        return q;
    }
    else
    {
        //sqrt not defined at 0 and will produce NaNs, thuswe use an approximation with taylor series truncated at one term
        q.w() = (T)1.0;
        q.vec() = _v *(T)0.5; // see the Taylor series of sinc(x) ~ 1 - x^2/3!, and have q.vec = v/2 * sinc(angle_half)
                                                                    //                                 for angle_half == 0 then ,     = v/2
        return q;
    }
}

// 'New' version (alternative version also used by ceres)
template<typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 1> q2v_new(const Eigen::QuaternionBase<Derived>& _q)
{
    typedef typename Derived::Scalar T;
    Eigen::Matrix<T, 3, 1> vec = _q.vec();
    const T sin_angle_square = vec(0) * vec(0) + vec(1) * vec(1) + vec(2) * vec(2);

    //everything shouold be OK for non-zero rotations
    if (sin_angle_square > (T)0.0)
    {
        const T sin_angle = sqrt(sin_angle_square);
        const T& cos_angle = _q.w();

        /* If (cos_theta < 0) then theta >= pi/2 , means : angle for angle_axis vector >= pi (== 2*theta) 
                    |-> results in correct rotation but not a normalized angle_axis vector 
    
        In that case we observe that 2 * theta ~ 2 * theta - 2 * pi,
        which is equivalent saying
    
            theta - pi = atan(sin(theta - pi), cos(theta - pi))
                        = atan(-sin(theta), -cos(theta))
        */
        const T two_angle = T(2.0) * ((cos_angle < 0.0) ? atan2(-sin_angle, -cos_angle) : atan2(sin_angle, cos_angle));
        const T k = two_angle / sin_angle;
        return vec * k;
    }
    else
    { // small-angle approximation using truncated Taylor series
        //zero rotation --> sqrt will result in NaN
        return vec * (T)2.0; // log = 2 * vec * ( 1 - norm(vec)^2 / 3*w^2 ) / w.
    }
}
    
}

TEST(rotations, v2q_VS_v2q_new) //this test will use functions defined above
{
    using namespace wolf;
    //defines scalars
    wolf::Scalar deg_to_rad = M_PI/180.0;

    Eigen::Vector4s vec0, vec1;

        //v2q
    Eigen::Vector3s rot_vector0, rot_vector1;
    Eigen::Quaternions quat_o, quat_o1, quat_new, quat_new1;
    Eigen::Vector4s vec_o, vec_o1, vec_new, vec_new1;
    Eigen::Vector3s qvec_o, qvec_o1, qvec_new, qvec_new1, qvec_aao, qvec_aa1;
    for (unsigned int iter = 0; iter < 10000; iter ++)
    {
        rot_vector0 = Eigen::Vector3s::Random();
        rot_vector1 = rot_vector0 * 100 *deg_to_rad; //far from origin
        rot_vector0 = rot_vector0 *0.0001*deg_to_rad; //close to origin

        quat_o = v2q(rot_vector0);
        quat_new = v2q_new(rot_vector0);
        quat_o1 = v2q(rot_vector1);
        quat_new1 = v2q_new(rot_vector1);

        //now we do the checking
        vec_o << quat_o.w(), quat_o.x(), quat_o.y(), quat_o.z();
        vec_new << quat_new.w(), quat_new.x(), quat_new.y(), quat_new.z();
        vec_o1 << quat_o1.w(), quat_o1.x(), quat_o1.y(), quat_o1.z();
        vec_new1 << quat_new1.w(), quat_new1.x(), quat_new1.y(), quat_new1.z();

        ASSERT_TRUE((vec_o - vec_new).isMuchSmallerThan(1,wolf::Constants::EPS));
        ASSERT_TRUE((vec_o1 - vec_new1).isMuchSmallerThan(1,wolf::Constants::EPS));
    

        //q2v
        qvec_o     = q2v(quat_o);
        qvec_o1    = q2v(quat_o1);
        qvec_aao   = q2v_aa(quat_o);
        qvec_aa1   = q2v_aa(quat_o1);
        qvec_new   = q2v_new(quat_new);
        qvec_new1  = q2v_new(quat_new1);

        // 'New' version (alternative version also used by ceres) of q2v is working, result with template version gives the same that the regular version with Eigen::Quaternions argument
        ASSERT_TRUE((qvec_aao - qvec_new).isMuchSmallerThan(1,wolf::Constants::EPS)) << "\n qvec_aao : " << qvec_aao.transpose() << "\n qvec_new : " << qvec_new.transpose() << std::endl;
        ASSERT_TRUE((qvec_aa1 - qvec_new1).isMuchSmallerThan(1,wolf::Constants::EPS)) << "\n qvec_aa1 : " << qvec_aa1.transpose() << "\n qvec_new1 : " << qvec_new1.transpose() << std::endl;
        EXPECT_TRUE((qvec_new - rot_vector0).isMuchSmallerThan(1,wolf::Constants::EPS)) << "\n qvec_new : " << qvec_new.transpose() << "\n rot_vector0 : " << rot_vector0.transpose() << std::endl;
        EXPECT_TRUE((qvec_new1 - rot_vector1).isMuchSmallerThan(1,wolf::Constants::EPS)) << "\n qvec_new1 : " << qvec_new1.transpose() << "\n rot_vector1 : " << rot_vector1.transpose() << std::endl;

        // checking current q2v
        ASSERT_TRUE((qvec_aao - qvec_o).isMuchSmallerThan(1,wolf::Constants::EPS)) << "\n qvec_aao : " << qvec_aao.transpose() << "\n qvec_new : " << qvec_new.transpose() << std::endl;
        ASSERT_TRUE((qvec_aa1 - qvec_o1).isMuchSmallerThan(1,wolf::Constants::EPS)) << "\n qvec_aa1 : " << qvec_aa1.transpose() << "\n qvec_new1 : " << qvec_new1.transpose() << std::endl;
        EXPECT_TRUE((qvec_o - rot_vector0).isMuchSmallerThan(1,wolf::Constants::EPS)) << "\n qvec_new : " << qvec_new.transpose() << "\n rot_vector0 : " << rot_vector0.transpose() << std::endl;
        EXPECT_TRUE((qvec_o1 - rot_vector1).isMuchSmallerThan(1,wolf::Constants::EPS)) << "\n qvec_new1 : " << qvec_new1.transpose() << "\n rot_vector1 : " << rot_vector1.transpose() << std::endl;
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

TEST(rotations, Skew_vee)
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

TEST(rotations, v2q_q2v)
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

TEST(rotations, v2R_R2v)
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

TEST(rotations, R2v_v2R_limits)
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

TEST(rotations, R2v_v2R_AAlimits)
{
    using namespace wolf;
    //let's see how small the angles can be here : limit reached at scale/10 =  1e-16
    wolf::Scalar scale = 1;
    Matrix3s rotation_mat;
    Vector3s rv;

    for(int i = 0; i<8; i++){
        rotation_mat = v2R(Vector3s::Random().eval() * scale);
        //rotation_mat(0,0) = 1.0;
        //rotation_mat(1,1) = 1.0;
        //rotation_mat(2,2) = 1.0;

        //rv = R2v(rotation_mat); //decomposing R2v below
        AngleAxis<wolf::Scalar> aa0 = AngleAxis<wolf::Scalar>(rotation_mat);
        rv = aa0.axis() * aa0.angle();
        //std::cout << "aa0.axis : " << aa0.axis().transpose() << ",\t aa0.angles :" << aa0.angle() <<std::endl;

        ASSERT_FALSE(rv == Vector3s::Zero());
        scale = scale*0.1;
    }
}

TEST(rotations, v2q2R2v)
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


TEST(rotations, Quat_compos_const_rateOfTurn)
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

TEST(rotations, Quat_compos_var_rateOfTurn)
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

TEST(rotations, Quat_compos_var_rateOfTurn_diff)
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

TEST(rotations, q2R_R2q)
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

TEST(rotations, Jr)
{
    Vector3s theta; theta.setRandom();
    Vector3s dtheta; dtheta.setRandom(); dtheta *= 1e-4;

    // Check the main Jr property for q and R
    // exp( theta + d_theta ) \approx exp(theta) * exp(Jr * d_theta)
    Matrix3s Jr = jac_SO3_right(theta);
    ASSERT_QUATERNION_APPROX(exp_q(theta+dtheta), exp_q(theta) * exp_q(Jr*dtheta), 1e-8);
    ASSERT_MATRIX_APPROX(exp_R(theta+dtheta), (exp_R(theta) * exp_R(Jr*dtheta)), 1e-8);
}

TEST(rotations, Jl)
{
    Vector3s theta; theta.setRandom();
    Vector3s dtheta; dtheta.setRandom(); dtheta *= 1e-4;

    // Check the main Jl property for q and R
    // exp( theta + d_theta ) \approx exp(Jl * d_theta) * exp(theta)
    Matrix3s Jl = jac_SO3_left(theta);
    ASSERT_QUATERNION_APPROX(exp_q(theta+dtheta), exp_q(Jl*dtheta) * exp_q(theta), 1e-8);
    ASSERT_MATRIX_APPROX(exp_R(theta+dtheta), (exp_R(Jl*dtheta) * exp_R(theta)), 1e-8);
}

TEST(rotations, Jr_inv)
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

TEST(rotations, Jl_inv)
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

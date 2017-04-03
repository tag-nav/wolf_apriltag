/**
 * \file test_motion_2d.cpp
 *
 *  Created on: Mar 15, 2016
 *      \author: jsola
 */

#include "utils_gtest.h"

// Classes under test
#include "../processor_odom_2D.h"

// Wolf includes
#include "../capture_fix.h"
#include "../state_block.h"
#include "../wolf.h"
#include "../ceres_wrapper/ceres_manager.h"

// STL includes
#include <map>
#include <list>
#include <algorithm>
#include <iterator>

// General includes
#include <iostream>
#include <iomanip>      // std::setprecision

using namespace wolf;
using namespace Eigen;

void showBuffer(const MotionBuffer& _buffer, std::string _label = "", const TimeStamp _t0 = TimeStamp(0.0))
{
    std::cout << _label << " <";
    for (const auto &s : _buffer.get())
        std::cout << s.ts_ - _t0 << ' ';
    std::cout << ">" << std::endl;
}

VectorXs plus(const VectorXs& _pose, const Vector2s& _data)
{
    VectorXs _pose_plus_data(3);
    _pose_plus_data(0) = _pose(0) + cos(_pose(2) + _data(1) / 2) * _data(0);
    _pose_plus_data(1) = _pose(1) + sin(_pose(2) + _data(1) / 2) * _data(0);
    _pose_plus_data(2) = pi2pi(_pose(2) + _data(1));
    return _pose_plus_data;
}

MatrixXs plus_jac_u(const VectorXs& _pose, const Vector2s& _data)
{
    MatrixXs Ju(3,2);
    Ju(0, 0) =  cos(_pose(2) + _data(1) / 2);
    Ju(0, 1) = -sin(_pose(2) + _data(1) / 2) * _data(0) / 2 ;
    Ju(1, 0) =  sin(_pose(2) + _data(1) / 2);
    Ju(1, 1) =  cos(_pose(2) + _data(1) / 2) * _data(0) / 2 ;
    Ju(2, 0) = 0.0;
    Ju(2, 1) = 1.0;
    return Ju;
}

MatrixXs plus_jac_x(const VectorXs& _pose, const Vector2s& _data)
{
    Matrix3s Jx;
    Jx(0, 0) = 1.0;
    Jx(0, 1) = 0.0;
    Jx(0, 2) = -sin(_pose(2) + _data(1) / 2) * _data(0);
    Jx(1, 0) = 0.0;
    Jx(1, 1) = 1.0;
    Jx(1, 2) =  cos(_pose(2) + _data(1) / 2) * _data(0);
    Jx(2, 0) = 0.0;
    Jx(2, 1) = 0.0;
    Jx(2, 2) = 1.0;
    return Jx;
}

TEST(ProcessorMotion2D, VoteForKfAndSolve)
{
    std::cout << std::setprecision(3);
    // time
    TimeStamp t0(0.0), t = t0;
    Scalar dt = .01;
    // Origin frame:
    Vector3s x0(.5, -.5 -sqrt(.5), M_PI_4);
    Eigen::Matrix3s x0_cov = Eigen::Matrix3s::Identity() * 0.1;
    // motion data
    VectorXs data(Vector2s(1, M_PI_4) ); // advance 1m turn pi/4 rad (45 deg). Need 8 steps for a complete turn
    Eigen::MatrixXs data_cov = Eigen::MatrixXs::Identity(2, 2) * 0.01;
    int N = 7; // number of process() steps

    // Create Wolf tree nodes
    ProblemPtr problem = Problem::create(FRM_PO_2D);
    SensorBasePtr sensor_odom2d = problem->installSensor("ODOM 2D", "odom", Vector3s(0,0,0));
    ProcessorParamsOdom2DPtr params(std::make_shared<ProcessorParamsOdom2D>());
    params->dist_traveled_th_   = 100;
    params->elapsed_time_th_    = 2.5*dt; // force KF at every third process()
    params->cov_det_th_         = 100;
    ProcessorBasePtr prc_base = problem->installProcessor("ODOM 2D", "odom", sensor_odom2d, params);
    ProcessorOdom2DPtr processor_odom2d = std::static_pointer_cast<ProcessorOdom2D>(prc_base);

    // NOTE: We integrate and create KFs as follows:
    // i=    0    1    2    3    4    5    6
    // KF -- * -- * -- KF - * -- * -- KF - *

    // Ceres wrapper
    CeresManager ceres_manager(problem);

    // Origin Key Frame
    FrameBasePtr origin_frame = problem->setPrior(x0, x0_cov, t0);
    ceres_manager.solve();
    ceres_manager.computeCovariances(ALL_MARGINALS);

    std::cout << "Initial pose : " << problem->getCurrentState().transpose() << std::endl;
    std::cout << "Initial covariance : " << std::endl << problem->getLastKeyFrameCovariance() << std::endl;
    std::cout << "Motion data  : " << data.transpose() << std::endl;

    // Check covariance values
    Eigen::Vector3s integrated_pose = x0;
    Eigen::Matrix3s integrated_cov = x0_cov;
    Eigen::Vector3s integrated_delta = Eigen::Vector3s::Zero();
    Eigen::Matrix3s integrated_delta_cov = Eigen::Matrix3s::Zero();
    Eigen::MatrixXs Ju(3, 2);
    Eigen::Matrix3s Jx;
    std::vector<Eigen::VectorXs> integrated_pose_vector;
    std::vector<Eigen::MatrixXs> integrated_cov_vector;

    std::cout << "\nIntegrating data..." << std::endl;

    t += dt;
    // Capture to use as container for all incoming data
    CaptureMotionPtr capture = std::make_shared<CaptureMotion>(t, sensor_odom2d, data, data_cov, nullptr);

    for (int i=0; i<N; i++)
    {
        std::cout << "-------------------\nStep " << i << " at time " << t << std::endl;
        // re-use capture with updated timestamp
        capture->setTimeStamp(t);

        // Processor
        sensor_odom2d->process(capture);
        Matrix3s odom2d_delta_cov = processor_odom2d->integrateBufferCovariance(processor_odom2d->getBuffer());
        std::cout << "State(" << (t - t0) << ") : " << processor_odom2d->getCurrentState().transpose() << std::endl;
        std::cout << "PRC  cov: \n" << odom2d_delta_cov << std::endl;

        // Integrate Delta
        if (i==2 || i==5) // keyframes
        {
            integrated_delta.setZero();
            integrated_delta_cov.setZero();
        }
        else
        {
            Ju = plus_jac_u(integrated_delta, data);
            Jx = plus_jac_x(integrated_delta, data);
            integrated_delta = plus(integrated_delta, data);
            integrated_delta_cov = Jx * integrated_delta_cov * Jx.transpose() + Ju * data_cov * Ju.transpose();
        }
        std::cout << "TEST cov: \n" << integrated_delta_cov << std::endl;

//        ASSERT_EIGEN_APPROX(processor_odom2d->getMotion().delta_, integrated_delta);
        ASSERT_EIGEN_APPROX(odom2d_delta_cov, integrated_delta_cov);

        // Integrate pose
        Ju = plus_jac_u(integrated_pose, data);
        Jx = plus_jac_x(integrated_pose, data);
        integrated_pose = plus(integrated_pose, data);
        integrated_cov = Jx * integrated_cov * Jx.transpose() + Ju * data_cov * Ju.transpose();

        // Pose error -- fix spurious error from pi to -pi
        Vector3s error_pose = processor_odom2d->getCurrentState() - integrated_pose;
        error_pose(2) = pi2pi(error_pose(2));
        ASSERT_EIGEN_APPROX(error_pose, Vector3s::Zero());

        integrated_pose_vector.push_back(integrated_pose);
        integrated_cov_vector.push_back(integrated_cov);

        t += dt;
    }

    // Solve
    ceres::Solver::Summary summary = ceres_manager.solve();
    ceres_manager.computeCovariances(ALL_MARGINALS);

    std::cout << "After solving the problem, covariance of the last keyframe:" << std::endl;
    std::cout << "WOLF:\n"      << problem->getLastKeyFrameCovariance() << std::endl;
    std::cout << "REFERENCE:\n" << integrated_cov_vector[5] << std::endl;

    ASSERT_EIGEN_APPROX(problem->getLastKeyFrameCovariance() , integrated_cov_vector[5]);
}

TEST(ProcessorMotion2D, SplitAndSolve)
{
    std::cout << std::setprecision(3);
    // time
    TimeStamp t0(0.0), t = t0;
    Scalar dt = .01;
    // Origin frame:
    Vector3s x0(.5, -.5 -sqrt(.5), M_PI_4);
    Eigen::Matrix3s x0_cov = Eigen::Matrix3s::Identity() * 0.1;
    // motion data
    VectorXs data(Vector2s(1, M_PI_4) ); // advance 1m turn pi/4 rad (45 deg). Need 8 steps for a complete turn
    Eigen::MatrixXs data_cov = Eigen::MatrixXs::Identity(2, 2) * 0.01;
    int N = 8; // number of process() steps

    // NOTE: We integrate and create KFs as follows:
    // i=    0    1    2    3    4    5    6
    // KF -- * -- * -- * -- * -- * -- * -- * : no keyframes during integration
    // And we split as follows
    //                           s          : exact time stamp t_split     = 5*dt
    //              s                       : fractional time stamp t_split = 2.2*dt


    // Create Wolf tree nodes
    ProblemPtr problem = Problem::create(FRM_PO_2D);
    SensorBasePtr sensor_odom2d = problem->installSensor("ODOM 2D", "odom", Vector3s(0,0,0));
    ProcessorParamsOdom2DPtr params(std::make_shared<ProcessorParamsOdom2D>());
    params->dist_traveled_th_   = 100; // don't make keyframes
    params->elapsed_time_th_    = 100;
    params->cov_det_th_         = 100;
    ProcessorBasePtr prc_base = problem->installProcessor("ODOM 2D", "odom", sensor_odom2d, params);
    ProcessorOdom2DPtr processor_odom2d = std::static_pointer_cast<ProcessorOdom2D>(prc_base);

    // Ceres wrapper
    CeresManager ceres_manager(problem);

    // Origin Key Frame
    FrameBasePtr origin_frame = problem->setPrior(x0, x0_cov, t0);
    ceres_manager.solve();
    ceres_manager.computeCovariances(ALL_MARGINALS);

    std::cout << "Initial pose : " << problem->getCurrentState().transpose() << std::endl;
    std::cout << "Initial covariance : " << std::endl << problem->getLastKeyFrameCovariance() << std::endl;
    std::cout << "Motion data  : " << data.transpose() << std::endl;

    // Check covariance values
    Eigen::Vector3s integrated_pose = x0;
    Eigen::Matrix3s integrated_cov = x0_cov;
    Eigen::Vector3s integrated_delta = Eigen::Vector3s::Zero();
    Eigen::Matrix3s integrated_delta_cov = Eigen::Matrix3s::Zero();
    Eigen::MatrixXs Ju(3, 2);
    Eigen::Matrix3s Jx;
    std::vector<Eigen::VectorXs> integrated_pose_vector;
    std::vector<Eigen::MatrixXs> integrated_cov_vector;

    std::cout << "\nIntegrating data..." << std::endl;

    t += dt;
    // Capture to use as container for all incoming data
    CaptureMotionPtr capture = std::make_shared<CaptureMotion>(t, sensor_odom2d, data, data_cov, nullptr);

    for (int i=0; i<N; i++)
    {
        std::cout << "-------------------\nStep " << i << " at time " << t << std::endl;
        // re-use capture with updated timestamp
        capture->setTimeStamp(t);

        // Processor
        sensor_odom2d->process(capture);
        Matrix3s odom2d_delta_cov = processor_odom2d->integrateBufferCovariance(processor_odom2d->getBuffer());
        std::cout << "State(" << (t - t0) << ") : " << processor_odom2d->getCurrentState().transpose() << std::endl;
        std::cout << "PRC  cov: \n" << odom2d_delta_cov << std::endl;

        // Integrate Delta
        if (false) // don't make keyframes
        {
            integrated_delta.setZero();
            integrated_delta_cov.setZero();
        }
        else
        {
            Ju = plus_jac_u(integrated_delta, data);
            Jx = plus_jac_x(integrated_delta, data);
            integrated_delta = plus(integrated_delta, data);
            integrated_delta_cov = Jx * integrated_delta_cov * Jx.transpose() + Ju * data_cov * Ju.transpose();
        }
        std::cout << "TEST cov: \n" << integrated_delta_cov << std::endl;

//        ASSERT_EIGEN_APPROX(odom2d_delta_cov, integrated_delta_cov);

        // Integrate pose
        Ju = plus_jac_u(integrated_pose, data);
        Jx = plus_jac_x(integrated_pose, data);
        integrated_pose = plus(integrated_pose, data);
        integrated_cov = Jx * integrated_cov * Jx.transpose() + Ju * data_cov * Ju.transpose();

//        ASSERT_EIGEN_APPROX(processor_odom2d->getCurrentState(), integrated_pose);

        integrated_pose_vector.push_back(integrated_pose);
        integrated_cov_vector.push_back(integrated_cov);

        t += dt;
    }

    // Solve
    ceres::Solver::Summary summary = ceres_manager.solve();
    ceres_manager.computeCovariances(ALL_MARGINALS);

    std::cout << "After solving the problem, covariance of the last keyframe:" << std::endl;
    std::cout << "WOLF:\n"      << problem->getLastKeyFrameCovariance() << std::endl;
    std::cout << "REFERENCE:\n" << integrated_cov_vector[N-1] << std::endl;

    // Split at an exact timestamp, t_split = n_split*dt;
    int n_split = 5;
    TimeStamp t_split (t0 + n_split*dt);

    showBuffer(processor_odom2d->getBuffer(), "Original buffer:", t0);

    std::cout << "Split time:                  " << t_split - t0 << std::endl;

    FrameBasePtr keyframe_split = problem->emplaceFrame(KEY_FRAME, processor_odom2d->getState(t_split), t_split);

    processor_odom2d->keyFrameCallback(keyframe_split, 0);
    //    problem_ptr->print(4,1,1,1);

    showBuffer((std::static_pointer_cast<CaptureMotion>(keyframe_split->getCaptureList().front()))->getBuffer(), "New buffer: oldest part: ", t0);
    showBuffer(processor_odom2d->getBuffer(), "Original keeps the newest: ", t0);

    ceres_manager.solve();
    ceres_manager.computeCovariances();

    ASSERT_EIGEN_APPROX(problem->getLastKeyFrameCovariance() , integrated_cov_vector[n_split - 1]);
}

//TEST(ProcessorMotion2D, Motion2D)
//{
//    std::cout << std::setprecision(3);
//    // time
//    TimeStamp t0(0.0), t = t0;
//    Scalar dt = .01;
//    // Origin frame:
//    Vector3s x0(.5, -.5 -sqrt(.5), M_PI_4);
//    Eigen::Matrix3s x0_cov = Eigen::Matrix3s::Identity() * 0.0000001;
//    // motion data
//    VectorXs data(Vector2s(1, M_PI_4) ); // advance 1m turn pi/4 rad (45 deg). Need 8 steps for a complete turn
//    Eigen::MatrixXs data_cov = Eigen::MatrixXs::Identity(2, 2) * 0.01;
//
//    // Create Wolf tree nodes
//    ProblemPtr problem_ptr = Problem::create(FRM_PO_2D);
//    SensorBasePtr sensor_odom_ptr = problem_ptr->installSensor("ODOM 2D", "odom", Vector3s(0,0,0));
//    ProcessorParamsOdom2DPtr params(std::make_shared<ProcessorParamsOdom2D>());
//    params->dist_traveled_th_   = 100;
//    params->elapsed_time_th_    = 100;
//    params->cov_det_th_         = 100;
//    ProcessorBasePtr prc = problem_ptr->installProcessor("ODOM 2D", "odom", sensor_odom_ptr, params);
//    ProcessorOdom2DPtr odom2d_ptr = std::static_pointer_cast<ProcessorOdom2D>(prc);
//
//    // Ceres wrapper
//    ceres::Solver::Options ceres_options;
////    ceres_options.minimizer_type = ceres::TRUST_REGION; //ceres::TRUST_REGION;LINE_SEARCH
////    ceres_options.max_line_search_step_contraction = 1e-3;
////    ceres_options.max_num_iterations = 100;
//    CeresManager* ceres_manager_ptr = new CeresManager(problem_ptr, ceres_options);
//
//
//    // Origin Key Frame
//    FrameBasePtr origin_frame = problem_ptr->setPrior(x0, x0_cov, t0);
//    ceres_manager_ptr->solve();
//    ceres_manager_ptr->computeCovariances(ALL_MARGINALS);
//
//    std::cout << "Initial pose : " << problem_ptr->getCurrentState().transpose() << std::endl;
//    std::cout << "Initial covariance : " << std::endl << problem_ptr->getLastKeyFrameCovariance() << std::endl;
//    std::cout << "Motion data  : " << data.transpose() << std::endl;
//
//    std::cout << "\nIntegrating states at synchronous time values..." << std::endl;
//
//    std::cout << "State(" << (t - t0) << ") : " << odom2d_ptr->getCurrentState().transpose() << std::endl;
//    // Capture to use as container for all incoming data
//    t += dt;
//    CaptureMotionPtr cap_ptr = std::make_shared<CaptureMotion>(t, sensor_odom_ptr, data, data_cov, nullptr);
//
//    // Check covariance values
//    Eigen::Vector3s integrated_x = x0;
//    Eigen::Matrix3s integrated_covariance = x0_cov;
//    Eigen::Vector3s integrated_delta = Eigen::Vector3s::Zero();
//    Eigen::Matrix3s integrated_delta_covariance = Eigen::Matrix3s::Zero();
//    Eigen::MatrixXs Ju(3, 2);
//    Eigen::Matrix3s Jx;
//    std::vector<Eigen::VectorXs> integrated_x_vector;
//    std::vector<Eigen::MatrixXs> integrated_covariance_vector;
//
//    for (int i = 0; i < 8; i++)
//    {
//        // Processor
//        sensor_odom_ptr->process(cap_ptr);
//        Matrix3s prc_cov = odom2d_ptr->integrateBufferCovariance(odom2d_ptr->getBuffer());
//        std::cout << "State(" << (t - t0) << ") : " << odom2d_ptr->getCurrentState().transpose() << std::endl;
//
//        // Integrate Delta
//        Ju = plus_jac_u(integrated_delta, data);
//        Jx = plus_jac_x(integrated_delta, data);
//        integrated_delta = plus(integrated_delta, data);
//        integrated_delta_covariance = Jx * integrated_delta_covariance * Jx.transpose() + Ju * data_cov * Ju.transpose();
//
//        ASSERT_EIGEN_APPROX(prc_cov, integrated_delta_covariance);
//
//        // Integrate pose
//        Ju = plus_jac_u(integrated_x, data);
//        Jx = plus_jac_x(integrated_x, data);
//        integrated_x = plus(integrated_x, data);
//        integrated_covariance = Jx * integrated_covariance * Jx.transpose() + Ju * data_cov * Ju.transpose();
//
//        ASSERT_EIGEN_APPROX(odom2d_ptr->getCurrentState(), integrated_x);
//
//        integrated_x_vector.push_back(integrated_x);
//        integrated_covariance_vector.push_back(integrated_covariance);
//
//        //        std::cout << "PRC  cov: \n" << prc_cov << std::endl;
//        //        std::cout << "TEST cov: \n" << integrated_delta_covariance << std::endl;
//
//
//        // Timestamp
//        t += dt;
//        cap_ptr->setTimeStamp(t);
//    }
//
//    std::cout << "\nQuery states at asynchronous time values..." << std::endl;
//
//    t = t0;
//    Scalar dt_query = 0.0065; // new dt
//    for (int i = 1; i <= 15; i++)
//    {
//        std::cout << "State(" << (t - t0) << ") = " << odom2d_ptr->getState(t).transpose() << std::endl;
//        t += dt_query;
//    }
//    std::cout << "       ^^^^^^^   After the last time-stamp the buffer keeps returning the last member." << std::endl;
//
//    // Split the buffer
//
//    std::cout << "\nSplitting the buffer!\n---------------------" << std::endl;
//    showBuffer(odom2d_ptr->getBuffer(), "Original buffer:", t0);
//
//    // first split at exact timestamp
//    Scalar Dt = 0.04; // this is multiple of dt
//    TimeStamp t_split = t0 + Dt;
//    std::cout << "Split time:                  " << t_split - t0 << std::endl;
//
//    FrameBasePtr new_keyframe_ptr = problem_ptr->emplaceFrame(KEY_FRAME, odom2d_ptr->getState(t_split), t_split);
//
//    odom2d_ptr->keyFrameCallback(new_keyframe_ptr, 0);
////    problem_ptr->print(4,1,1,1);
//
//    showBuffer((std::static_pointer_cast<CaptureMotion>(new_keyframe_ptr->getCaptureList().front()))->getBuffer(), "New buffer: oldest part: ", t0);
//    showBuffer(odom2d_ptr->getBuffer(), "Original keeps the newest: ", t0);
//
//    std::cout << "Motion factor: " << std::endl;
//    std::cout << "  Delta     : \n" << new_keyframe_ptr->getCaptureList().front()->getFeatureList().front()->getMeasurement().transpose() << std::endl;
//    std::cout << "  Covariance: \n" << new_keyframe_ptr->getCaptureList().front()->getFeatureList().front()->getMeasurementCovariance() << std::endl;
//
//    std::cout << "getState with TS previous than the last keyframe: " << (t_split-t0)-0.5 << std::endl;
//    std::cout << odom2d_ptr->getState(t_split-0.5).transpose() << std::endl;
//
//    // Solve
//    ceres::Solver::Summary summary = ceres_manager_ptr->solve();
//    //std::cout << summary.FullReport() << std::endl;
//    ceres_manager_ptr->computeCovariances(ALL_MARGINALS);
//
//    int i = Dt/dt - 1;
//    if ((problem_ptr->getFrameCovariance(new_keyframe_ptr) - integrated_covariance_vector[i]).array().abs().maxCoeff() > 1e-10)
//    {
//        std::cout << "After solving the problem, covariance of new keyframe:" << std::endl;
//        std::cout << "WOLF:\n"      << problem_ptr->getFrameCovariance(new_keyframe_ptr) << std::endl;
//        std::cout << "REFERENCE:\n" << integrated_covariance_vector[i] << std::endl;
//        std::cout << "ERROR:\n"     << problem_ptr->getFrameCovariance(new_keyframe_ptr) - integrated_covariance_vector[i] << std::endl;
//        std::cout << "1st TEST COVARIANCE CHECK ------> ERROR!: Integrated covariance different from reference." << std::endl;
//    }
//
//    ASSERT_EIGEN_APPROX(problem_ptr->getFrameCovariance(new_keyframe_ptr) , integrated_covariance_vector[i]);
//
//
//    // second split as non-exact timestamp
//    Dt = 0.022; // this is NOT multiple of dt
//    t_split = t0 + Dt;
//    std::cout << "\nSplitting the buffer!\n---------------------" << std::endl;
//    std::cout << "Split time (asynchronous and older than previous keyframe):  " << t_split - t0 << std::endl;
//
//    new_keyframe_ptr = problem_ptr->emplaceFrame(KEY_FRAME, odom2d_ptr->getState(t_split), t_split);
//
//    odom2d_ptr->keyFrameCallback(new_keyframe_ptr, 0);
////    problem_ptr->print(4,1,1,1);
//
//    // Solve
//    summary = ceres_manager_ptr->solve();
////    std::cout << summary.FullReport() << std::endl;
//    ceres_manager_ptr->computeCovariances(ALL_MARGINALS);
//
//    i = Dt/dt - 1;
//    if ((problem_ptr->getFrameCovariance(new_keyframe_ptr) - integrated_covariance_vector[i]).array().abs().maxCoeff() > 1e-10)
//    {
//        std::cout << "After solving the problem, covariance of new keyframe:" << std::endl;
//        std::cout << "WOLF:" << std::endl << problem_ptr->getFrameCovariance(new_keyframe_ptr) << std::endl;
//        std::cout << "REFERENCE:" << std::endl << integrated_covariance_vector[i] << std::endl;
//        std::cout << "ERROR:" << std::endl << problem_ptr->getFrameCovariance(new_keyframe_ptr) - integrated_covariance_vector[i] << std::endl;
//        std::cout << "2nd TEST COVARIANCE CHECK ------> ERROR!: Integrated covariance different from reference." << std::endl;
//
//    }
//    ASSERT_TRUE((problem_ptr->getFrameCovariance(new_keyframe_ptr) - integrated_covariance_vector[i]).isMuchSmallerThan(1.0, Constants::EPS));
//
//
////    std::cout << "All in one row:            < ";
////    for (const auto &s : (std::static_pointer_cast<CaptureMotion>(new_keyframe_ptr->getCaptureList().front()))->getBuffer().get())
////        std::cout << s.ts_ - t0 << ' ';
////    std::cout << "> " << t_split - t0 << " < ";
////    for (const auto &s : odom2d_ptr->getBuffer().get())
////        std::cout << s.ts_ - t0 << ' ';
////    std::cout << ">" << std::endl;
//
//    // Free allocated memory
//    delete ceres_manager_ptr;
//    std::cout << "ceres manager deleted" << std::endl;
//    problem_ptr.reset();
//    std::cout << "problem deleted" << std::endl;
//}


int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}


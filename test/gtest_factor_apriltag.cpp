//--------LICENSE_START--------
//
// Copyright (C) 2020,2021,2022,2023 Institut de Robòtica i Informàtica Industrial, CSIC-UPC.
// Authors: Joan Solà Ortega (jsola@iri.upc.edu)
// All rights reserved.
//
// This file is part of WOLF
// WOLF is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
//--------LICENSE_END--------
#include <core/utils/utils_gtest.h>

#include "core/common/wolf.h"
#include "core/utils/logging.h"

#include "core/ceres_wrapper/solver_ceres.h"
#include "core/processor/factory_processor.h"

#include "vision/capture/capture_image.h"

#include "apriltag/processor/processor_tracker_landmark_apriltag.h"
#include "apriltag/factor/factor_apriltag.h"
#include "apriltag/internal/config.h"

#include <apriltag/apriltag.h>

using namespace Eigen;
using namespace wolf;
using std::static_pointer_cast;

////////////////////////////////////////////////////////////////
/*
 * Wrapper class to be able to have setOrigin() and setLast() in ProcessorTrackerLandmarkApriltag
 */
WOLF_PTR_TYPEDEFS(ProcessorTrackerLandmarkApriltag_Wrapper);
class ProcessorTrackerLandmarkApriltag_Wrapper : public ProcessorTrackerLandmarkApriltag
{
    public:
        ProcessorTrackerLandmarkApriltag_Wrapper(ParamsProcessorTrackerLandmarkApriltagPtr _params_tracker_landmark_apriltag) :
            ProcessorTrackerLandmarkApriltag(_params_tracker_landmark_apriltag)
        {
            setType("ProcessorTrackerLandmarkApriltag_Wrapper");
        };
        ~ProcessorTrackerLandmarkApriltag_Wrapper() override{}
        void setOriginPtr(const CaptureBasePtr _origin_ptr) { origin_ptr_ = _origin_ptr; }
        void setLastPtr  (const CaptureBasePtr _last_ptr)   { last_ptr_ = _last_ptr; }
        void setIncomingPtr  (const CaptureBasePtr _incoming_ptr)   { incoming_ptr_ = _incoming_ptr; }

        // for factory
        static ProcessorBasePtr create(const std::string& _unique_name, const ParamsProcessorBasePtr _params)
        {
            auto prc_apriltag_params_ = std::static_pointer_cast<ParamsProcessorTrackerLandmarkApriltag>(_params);

            auto prc_ptr = std::make_shared<ProcessorTrackerLandmarkApriltag_Wrapper>(prc_apriltag_params_);

            prc_ptr->setName(_unique_name);

            return prc_ptr;
        }

};
namespace wolf{
// Register in the Factories
WOLF_REGISTER_PROCESSOR(ProcessorTrackerLandmarkApriltag_Wrapper);
}
////////////////////////////////////////////////////////////////

// Use the following in case you want to initialize tests with predefines variables or methods.
class FactorApriltag_class : public testing::Test{
    public:
        Vector3d    pos_camera,   pos_robot,   pos_landmark;
        Vector3d    euler_camera, euler_robot, euler_landmark;
        Quaterniond quat_camera,  quat_robot,  quat_landmark;
        Vector4d    vquat_camera, vquat_robot, vquat_landmark; // quaternions as vectors
        Vector7d    pose_camera,  pose_robot,  pose_landmark;

        ProblemPtr      problem;
        SolverCeresPtr solver;

        SensorCameraPtr camera;
        ProcessorTrackerLandmarkApriltag_WrapperPtr proc_apriltag;

        SensorBasePtr   S;
        FrameBasePtr    F1;
        CaptureImagePtr C1;
        FeatureApriltagPtr  f1;
        LandmarkApriltagPtr lmk1;
        FactorApriltagPtr c_tag;
        apriltag_detection_t    det;

        double rep_error1;
        double rep_error2;
        bool use_rotation;

        Eigen::Matrix6d meas_cov;

        void SetUp() override
        {
            std::string wolf_root = _WOLF_APRILTAG_CODE_DIR;

            // configuration

             /* We have three poses to take into account:
             *  - pose of the camera (extrinsincs)
             *  - pose of the landmark
             *  - pose of the robot (Keyframe)
             *
             * The measurement provides the pose of the landmark wrt camera's current pose in the world.
             * Camera's current pose in World is the composition of the robot pose with the camera extrinsics.
             *
             * The robot has a camera looking forward
             *   S: pos = (0,0,0), ori = (0, 0, 0)
             *
             * There is a point at the origin
             *   P: pos = (0,0,0)
             *
             * The camera is canonical
             *   K = Id.
             *
             * Therefore, P projects exactly at the origin of the camera,
             *   f: p = (0,0)
             *
             * We form a Wolf tree with 1 frames F1, 1 capture C1,
             * 1 feature f1 (measurement), 1 landmark l and 1 apriltag constraint c1:
             *
             *   Frame F1, Capture C1, feature f1, landmark l1, constraint c1
             *
             * The frame pose F1, and the camera pose S
             * in the robot frame are variables subject to optimization
             *
             * We perform a number of tests based on this configuration.*/


            // camera is at origin of robot and looking forward
            // robot is at (0,0,0)
            // landmark is right in front of camera. Its orientation wrt camera is identity.
            pos_camera      << 0,0,0;
            pos_robot       << 0,0,0; //robot is at origin
            pos_landmark    << 0,1,0;
            euler_camera    << 0,0,0;
            //euler_camera    << -M_PI_2, 0, -M_PI_2;
            euler_robot     << 0,0,0;
            euler_landmark  << 0,0,0;
            quat_camera     = e2q(euler_camera);
            quat_robot      = e2q(euler_robot);
            quat_landmark   = e2q(euler_landmark);
            vquat_camera    = quat_camera.coeffs();
            vquat_robot     = quat_robot.coeffs();
            vquat_landmark  = quat_landmark.coeffs();
            pose_camera     << pos_camera, vquat_camera;
            pose_robot      << pos_robot, vquat_robot;
            pose_landmark   << pos_landmark, vquat_landmark;

            // Build problem
            problem = Problem::create("PO", 3);
            solver = std::make_shared<SolverCeres>(problem);

            // Install sensor and processor
            S      = problem->installSensor("SensorCamera", "canonical", pose_camera, wolf_root + "/demos/camera_params_canonical.yaml");
            camera = std::static_pointer_cast<SensorCamera>(S);

            ParamsProcessorTrackerLandmarkApriltagPtr params = std::make_shared<ParamsProcessorTrackerLandmarkApriltag>();
            // Need to set some parameters ? do it now !
            params->tag_family_ = "tag36h11";
            params->time_tolerance = 1;
            //params->name        = params->tag_family_;

            ProcessorBasePtr proc = problem->installProcessor("ProcessorTrackerLandmarkApriltag_Wrapper", "apriltags_wrapper", camera, params);
            proc_apriltag = std::static_pointer_cast<ProcessorTrackerLandmarkApriltag_Wrapper>(proc);

            // F1 is be origin KF
            VectorComposite x0(pose_robot, "PO", {3,4});
            VectorComposite s0("PO", {Vector3d(1,1,1), Vector3d(1,1,1)});
            F1 = problem->setPriorFactor(x0, s0, 0.0);

            //emplace capture & set as last and origin
            C1 = std::static_pointer_cast<CaptureImage>(CaptureBase::emplace<CaptureImage>(F1, 1.0, camera, cv::Mat(2,2,CV_8UC1)));
            proc_apriltag->setOriginPtr(C1);
            proc_apriltag->setLastPtr(C1);

            // the sensor is at origin as well as the robot. The measurement matches with the pose of the tag wrt camera (and also wrt robot and world)
            meas_cov = Eigen::Matrix6d::Identity();
            meas_cov.topLeftCorner(3,3)     *= 1e-2;
            meas_cov.bottomRightCorner(3,3) *= 1e-3;
            int tag_id = 1;

            det.id = tag_id;
            det.p[0][0] =  1.0;
            det.p[0][1] = -1.0;
            det.p[1][0] =  1.0;
            det.p[1][1] =  1.0;
            det.p[2][0] = -1.0;
            det.p[2][1] =  1.0;
            det.p[3][0] = -1.0;
            det.p[3][1] = -1.0;

            rep_error1 = 0.01;
            rep_error2 = 0.1;
            use_rotation = true;

            //emplace feature and landmark
            f1 = std::static_pointer_cast<FeatureApriltag>(FeatureBase::emplace<FeatureApriltag>(C1, pose_landmark, meas_cov, det.id, det, rep_error1, rep_error2, use_rotation));
            lmk1 = std::static_pointer_cast<LandmarkApriltag>(proc_apriltag->emplaceLandmark(f1));
        }
};


TEST_F(FactorApriltag_class, Constructor)
{
    FactorApriltagPtr factor = std::make_shared<FactorApriltag>(
            S,
            F1,
            lmk1,
            f1,
            nullptr,
            false,
            FAC_ACTIVE
    );

    ASSERT_TRUE(factor->getType() == "FactorApriltag");
}

TEST_F(FactorApriltag_class, Check_tree)
{
    auto factor = FactorBase::emplace<FactorApriltag>(f1,
                                                              S,
                                                              F1,
                                                              lmk1,
                                                              f1,
                                                              nullptr,
                                                              false,
                                                              FAC_ACTIVE);
    ASSERT_TRUE(problem->check(1));
}

TEST_F(FactorApriltag_class, solve_F1_P_perturbated)
{
    auto factor = FactorBase::emplace<FactorApriltag>(f1,
                                                              S,
                                                              F1,
                                                              lmk1,
                                                              f1,
                                                              nullptr,
                                                              false,
                                                              FAC_ACTIVE);
    ASSERT_TRUE(problem->check(1));

    // unfix F1, perturbate state
    F1->unfix();
    F1->getP()->perturb();

    std::string report = solver->solve(SolverManager::ReportVerbosity::QUIET); // 0: nothing, 1: BriefReport, 2: FullReport
    ASSERT_MATRIX_APPROX(F1->getState().vector("PO"), pose_robot, 1e-6);
}

TEST_F(FactorApriltag_class, solve_F1_O_perturbated)
{
    auto factor = FactorBase::emplace<FactorApriltag>(f1,
                                                              S,
                                                              F1,
                                                              lmk1,
                                                              f1,
                                                              nullptr,
                                                              false,
                                                              FAC_ACTIVE);

    // unfix F1, perturbate state
    F1->unfix();
    F1->getO()->perturb();

    std::string report = solver->solve(SolverManager::ReportVerbosity::QUIET); // 0: nothing, 1: BriefReport, 2: FullReport
    ASSERT_MATRIX_APPROX(F1->getState().vector("PO"), pose_robot, 1e-6);
}

TEST_F(FactorApriltag_class, Check_initialization)
{
    auto factor = FactorBase::emplace<FactorApriltag>(f1,
                                                              S,
                                                              F1,
                                                              lmk1,
                                                              f1,
                                                              nullptr,
                                                              false,
                                                              FAC_ACTIVE);

    ASSERT_MATRIX_APPROX(F1->getState().vector("PO"), pose_robot, 1e-6);
    ASSERT_MATRIX_APPROX(f1->getMeasurement(), pose_landmark, 1e-6);
    ASSERT_MATRIX_APPROX(lmk1->getState().vector("PO"), pose_landmark, 1e-6);

}

TEST_F(FactorApriltag_class, solve_L1_P_perturbated)
{
    auto factor = FactorBase::emplace<FactorApriltag>(f1,
                                                              S,
                                                              F1,
                                                              lmk1,
                                                              f1,
                                                              nullptr,
                                                              false,
                                                              FAC_ACTIVE);

    // unfix lmk1, perturbate state
    lmk1->unfix();
    lmk1->getP()->perturb();

    std::string report = solver->solve(SolverManager::ReportVerbosity::QUIET); // 0: nothing, 1: BriefReport, 2: FullReport
    ASSERT_MATRIX_APPROX(F1->getState().vector("PO"), pose_robot, 1e-6);
    ASSERT_MATRIX_APPROX(lmk1->getState().vector("PO"), pose_landmark, 1e-6);
}

TEST_F(FactorApriltag_class, solve_L1_O_perturbated)
{
    auto factor = FactorBase::emplace<FactorApriltag>(f1,
                                                              S,
                                                              F1,
                                                              lmk1,
                                                              f1,
                                                              nullptr,
                                                              false,
                                                              FAC_ACTIVE);

    // unfix F1, perturbate state
    lmk1->unfix();
    lmk1->getO()->perturb();

    std::string report = solver->solve(SolverManager::ReportVerbosity::QUIET); // 0: nothing, 1: BriefReport, 2: FullReport
    ASSERT_MATRIX_APPROX(F1->getState().vector("PO"), pose_robot, 1e-6);
    ASSERT_MATRIX_APPROX(lmk1->getState().vector("PO"), pose_landmark, 1e-6);

}

TEST_F(FactorApriltag_class, solve_L1_PO_perturbated)
{
    // Change setup
    Vector3d p_w_r, p_r_c, p_c_l, p_w_l;
    Quaterniond q_w_r, q_r_c, q_c_l, q_w_l;
    p_w_r << 1, 2, 3;
    p_r_c << 4, 5, 6;
    p_c_l << 7, 8, 9;
    q_w_r.coeffs() << 1, 2, 3, 4; q_w_r.normalize();
    q_r_c.coeffs() << 4, 5, 6, 7; q_r_c.normalize();
    q_c_l.coeffs() << 7, 8, 9, 0; q_c_l.normalize();

    q_w_l = q_w_r * q_r_c * q_c_l;
    p_w_l = p_w_r + q_w_r * (p_r_c + q_r_c * p_c_l);

    // Change feature (remove and emplace)
    Vector7d meas;
    meas << p_c_l, q_c_l.coeffs();
    f1->remove();
    f1 = std::static_pointer_cast<FeatureApriltag>(FeatureBase::emplace<FeatureApriltag>(C1, meas, meas_cov, det.id, det, rep_error1, rep_error2, use_rotation));

    // emplace factor
    auto factor = FactorBase::emplace<FactorApriltag>(f1,
                                                      S,
                                                      F1,
                                                      lmk1,
                                                      f1,
                                                      nullptr,
                                                      false,
                                                      FAC_ACTIVE);

    // Change Landmark
    lmk1->getP()->setState(p_w_l);
    lmk1->getO()->setState(q_w_l.coeffs());
    ASSERT_TRUE(lmk1->getP()->stateUpdated());
    ASSERT_TRUE(lmk1->getO()->stateUpdated());

    // Change Frame
    F1->getP()->setState(p_w_r);
    F1->getO()->setState(q_w_r.coeffs());
    F1->fix();
    ASSERT_TRUE(F1->getP()->stateUpdated());
    ASSERT_TRUE(F1->getO()->stateUpdated());

    // Change sensor extrinsics
    S->getP()->setState(p_r_c);
    S->getO()->setState(q_r_c.coeffs());
    ASSERT_TRUE(S->getP()->stateUpdated());
    ASSERT_TRUE(S->getO()->stateUpdated());

    Vector7d t_w_r, t_w_l;
    t_w_r << p_w_r, q_w_r.coeffs();
    t_w_l << p_w_l, q_w_l.coeffs();
    ASSERT_MATRIX_APPROX(F1->getState().vector("PO"), t_w_r, 1e-6);
    ASSERT_MATRIX_APPROX(lmk1->getState().vector("PO"), t_w_l, 1e-6);

    // unfix LMK, perturbate state
    lmk1->unfix();
    lmk1->perturb();

    std::string report = solver->solve(SolverManager::ReportVerbosity::QUIET); // 0: nothing, 1: BriefReport, 2: FullReport
    ASSERT_MATRIX_APPROX(F1->getState().vector("PO").transpose(), t_w_r.transpose(), 1e-6);
    ASSERT_MATRIX_APPROX(lmk1->getState().vector("PO").transpose(), t_w_l.transpose(), 1e-6);

}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}


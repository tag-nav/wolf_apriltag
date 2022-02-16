//--------LICENSE_START--------
//
// Copyright (C) 2020,2021,2022 Institut de Robòtica i Informàtica Industrial, CSIC-UPC.
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
#include "core/math/rotations.h"
#include "core/utils/logging.h"

#include "core/ceres_wrapper/solver_ceres.h"
#include "core/processor/factory_processor.h"

#include "vision/capture/capture_image.h"

#include "apriltag/processor/processor_tracker_landmark_apriltag.h"
#include "apriltag/feature/feature_apriltag_proj.h"
#include "apriltag/factor/factor_apriltag_proj.h"
#include "apriltag/internal/config.h"

#include <apriltag/apriltag.h>

using namespace Eigen;
using namespace wolf;
using std::static_pointer_cast;

Isometry3d pose2iso(const Vector3d& posi, const Quaterniond& quat)
{
    return Translation<double,3>(posi) * quat;
}

Isometry3d pose2iso(const Vector7d& pose)
{
    Vector3d posi =  pose.segment<3>(0);
    Quaterniond quat(pose.segment<4>(3));
    return pose2iso(posi, quat);
}

Vector7d iso2pose(const Isometry3d& T)
{   
    Vector7d pose;
    pose << T.translation(), Quaterniond(T.rotation()).coeffs();
    return pose;
}

Vector7d posiquat2pose(const Vector3d& posi, const Quaterniond& quat)
{   
    Vector7d pose;
    pose << posi, quat.coeffs();
    return pose;
}


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
class FactorApriltagProj_class : public testing::Test{
    public:

        // ground truth needed for individual tests
        Vector3d    p_w_r1, p_w_r2;
        Quaterniond q_w_r1, q_w_r2;
        Vector3d    p_r_c;
        Quaterniond q_r_c;
        Vector3d    p_w_l;
        Quaterniond q_w_l;
        // measurements
        Vector3d p_c1_l;
        Quaterniond q_c1_l;
        Vector3d p_c2_l;
        Quaterniond q_c2_l;

        Matrix3d K;

        ProblemPtr     problem;
        SolverCeresPtr solver;

        SensorCameraPtr camera;
        ProcessorTrackerLandmarkApriltag_WrapperPtr proc_apriltag;

        SensorBasePtr   S;
        FrameBasePtr    F1;
        CaptureImagePtr C1;
        FeatureApriltagProjPtr  f1;
        FeatureApriltagProjPtr  f2;
        LandmarkApriltagPtr lmk1;
        FactorApriltagProjPtr fac_a1;
        FactorApriltagProjPtr fac_a2;
        apriltag_detection_t det;
        
        Eigen::Matrix8d meas_cov;
        double tag_width;

        void SetUp() override
        {
            std::string wolf_root = _WOLF_APRILTAG_ROOT_DIR;

            // configuration

             /* We have three types of poses to take into account:
             *  - robot->camera (extrinsincs)  p_r_c, q_r_c
             *  - world->landmark              p_w_l, q_w_l
             *  - world->robot (Keyframe)      p_w_r, q_w_r
             *
             *  Factor graph with 2 Keyframes and 1 lmks
             * 
             * 
             * 
             *        L
             *        |\
             *        | \
             *   E----|--\
             *        |   \
             *        KF1  KF2
             * 
             * Measurements are pixel projections of the landmarks tags corners (8D), associated error is pixel covariance (8x8)
             * 
             */

            
            // camera is at origin of robot and looking forward
            // robot is at (0,0,0)
            // landmark is right in front of camera. Its orientation wrt camera is identity.


            // ground truth needed for individual tests
            // robot pose 1
            p_w_r1 << 0,0,0;
            Vector3d euler_w_r1; euler_w_r1 << 0,0,0;
            q_w_r1 = e2q(euler_w_r1);
            // robot pose 2
            p_w_r2 << 0,-0.1,0;
            Vector3d euler_w_r2; euler_w_r2 << 0,0,0;
            q_w_r2 = e2q(euler_w_r2);
            // extrinsics
            p_r_c << 0,0,0;
            Vector3d euler_r_c; euler_r_c << toRad(-90.0), 0, toRad(-90.0);  // put FLOATS otherwise 0
            q_r_c = e2q(euler_r_c);
            Matrix3d R = q_r_c.toRotationMatrix();
            std::cout << R << std::endl;
            // landmark (same orientation as camera of robot frame 1 (r1))
            p_w_l << 1,0,0;
            Vector3d euler_w_l; euler_w_l << toRad(-90.0), 0, toRad(-90.0);
            q_w_l = e2q(euler_w_l);

            // camera landmark pose (measurements)
            Isometry3d T_w_r1 = pose2iso(p_w_r1, q_w_r1);
            Isometry3d T_w_r2 = pose2iso(p_w_r2, q_w_r2);
            Isometry3d T_r_c = pose2iso(p_r_c, q_r_c);
            Isometry3d T_w_l = pose2iso(p_w_l, q_w_l);

            Isometry3d T_c1_l = (T_w_r1*T_r_c).inverse() * T_w_l;
            Isometry3d T_c2_l = (T_w_r2*T_r_c).inverse() * T_w_l;
            Vector7d pose_c1_l = iso2pose(T_c1_l);
            Vector7d pose_c2_l = iso2pose(T_c2_l);
            p_c1_l = pose_c1_l.segment<3>(0);
            q_c1_l = Quaterniond(pose_c1_l.segment<4>(3));
            p_c2_l = pose_c2_l.segment<3>(0);
            q_c2_l = Quaterniond(pose_c2_l.segment<4>(3));


            Vector7d pose_w_r1; pose_w_r1 << p_w_r1, q_w_r1.coeffs() ;
            Vector7d pose_w_r2; pose_w_r2 << p_w_r2, q_w_r2.coeffs() ;
            Vector7d pose_r_c; pose_r_c << p_r_c, q_r_c.coeffs() ;
            Vector7d pose_w_l; pose_w_l << p_w_l, q_w_l.coeffs() ;

            // Build problem
            problem = Problem::create("PO", 3);
            solver = std::make_shared<SolverCeres>(problem);

            // Install sensor and processor
            S      = problem->installSensor("SensorCamera", "canonical", pose_r_c, wolf_root + "/demos/camera_params_canonical.yaml");
            camera = std::static_pointer_cast<SensorCamera>(S);

            Vector4d k = camera->getIntrinsic()->getState(); //[cx cy fx fy]
            K << k(2), 0,    k(0),
                0,    k(3), k(1),
                0,    0,    1;

            ParamsProcessorTrackerLandmarkApriltagPtr params = std::make_shared<ParamsProcessorTrackerLandmarkApriltag>();
            // Need to set some parameters ? do it now !
            params->tag_family_ = "tag36h11";
            params->time_tolerance = 1;
            //params->name        = params->tag_family_;

            ProcessorBasePtr proc = problem->installProcessor("ProcessorTrackerLandmarkApriltag_Wrapper", "apriltags_wrapper", camera, params);
            proc_apriltag = std::static_pointer_cast<ProcessorTrackerLandmarkApriltag_Wrapper>(proc);

            // F1 is be origin KF
            VectorComposite x0(pose_w_r1, "PO", {3,4});
            VectorComposite s0("PO", {Vector3d(1,1,1), Vector3d(1,1,1)});
            F1 = problem->setPriorFactor(x0, s0, 0.0);

            // emplace dummy capture & set as last and origin
            C1 = std::static_pointer_cast<CaptureImage>(CaptureBase::emplace<CaptureImage>(F1, 0.0, camera, cv::Mat(2,2,CV_8UC1)));
            proc_apriltag->setOriginPtr(C1);
            proc_apriltag->setLastPtr(C1);

            meas_cov = 1*Eigen::Matrix8d::Identity();  // pixel noise
            int tag_id = 1;

            // unused
            det.id = tag_id;
            det.p[0][0] =  1.0;
            det.p[0][1] = -1.0;
            det.p[1][0] =  1.0;
            det.p[1][1] =  1.0;
            det.p[2][0] = -1.0;
            det.p[2][1] =  1.0;
            det.p[3][0] = -1.0;
            det.p[3][1] = -1.0;

            tag_width = 0.2;
            lmk1 = LandmarkBase::emplace<LandmarkApriltag>(problem->getMap(), pose_w_l, 42, tag_width);


        }
};


TEST_F(FactorApriltagProj_class, Constructor)
{   
    Vector8d meas = Vector8d::Zero();
    f1 = std::static_pointer_cast<FeatureApriltagProj>(FeatureBase::emplace<FeatureApriltagProj>(C1, meas, meas_cov, det.id, tag_width, det));
    FactorApriltagProjPtr factor = std::make_shared<FactorApriltagProj>(
            S,
            F1,
            lmk1,
            f1,
            nullptr,
            false,
            FAC_ACTIVE
    );

    ASSERT_TRUE(factor->getType() == "FactorApriltagProj");
}



TEST_F(FactorApriltagProj_class, problem_1KF)
{

    double s = tag_width/2;
    Eigen::Vector3d l_corn1; l_corn1 << -s,  s, 0; // bottom left
    Eigen::Vector3d l_corn2; l_corn2 <<  s,  s, 0; // bottom right
    Eigen::Vector3d l_corn3; l_corn3 <<  s, -s, 0; // top right
    Eigen::Vector3d l_corn4; l_corn4 << -s, -s, 0; // top left


    Vector8d meas;
    meas << FactorApriltagProj::pinholeProj(K, p_c1_l, q_c1_l, l_corn1),
            FactorApriltagProj::pinholeProj(K, p_c1_l, q_c1_l, l_corn2),
            FactorApriltagProj::pinholeProj(K, p_c1_l, q_c1_l, l_corn3),
            FactorApriltagProj::pinholeProj(K, p_c1_l, q_c1_l, l_corn4);


    f1 = std::static_pointer_cast<FeatureApriltagProj>(FeatureBase::emplace<FeatureApriltagProj>(C1, meas, meas_cov, det.id, tag_width, det));

    //emplace feature and landmark
    auto factor = FactorBase::emplace<FactorApriltagProj>(f1,
                                                          S,
                                                          F1,
                                                          lmk1,
                                                          f1,
                                                          nullptr,
                                                          false,
                                                          FAC_ACTIVE);
    ASSERT_TRUE(problem->check(0));
    problem->print(4,1,1,1);

    ASSERT_MATRIX_APPROX(F1->getState().vector("PO"), posiquat2pose(p_w_r1, q_w_r1), 1e-6);

    // // unfix F1, perturbate state
    // F1->unfix();
    // F1->getP()->perturb();
    // problem->print(4,1,1,1);
    problem->perturb();

    std::string report = solver->solve(SolverManager::ReportVerbosity::QUIET); // 0: nothing, 1: BriefReport, 2: FullReport
    problem->print(4,1,1,1);
    ASSERT_MATRIX_APPROX(F1->getState().vector("PO"), posiquat2pose(p_w_r1, q_w_r1), 1e-6);



    Isometry3d T_w_l_post = pose2iso(lmk1->getState().vector("PO"));
    Isometry3d T_w_r1 = pose2iso(p_w_r1, q_w_r1);
    Isometry3d T_r_c = pose2iso(p_r_c, q_r_c);

    Isometry3d T_c1_l = (T_w_r1*T_r_c).inverse() * T_w_l_post;
    Vector3d p_c1_l = T_c1_l.translation();
    Quaterniond q_c1_l = Quaterniond(T_c1_l.rotation());

    Vector8d proj_post;
    proj_post << FactorApriltagProj::pinholeProj(K, p_c1_l, q_c1_l, l_corn1),
                 FactorApriltagProj::pinholeProj(K, p_c1_l, q_c1_l, l_corn2),
                 FactorApriltagProj::pinholeProj(K, p_c1_l, q_c1_l, l_corn3),
                 FactorApriltagProj::pinholeProj(K, p_c1_l, q_c1_l, l_corn4);
    std::cout << "proj_post" << std::endl;
    std::cout << proj_post.transpose() << std::endl;

}



int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}


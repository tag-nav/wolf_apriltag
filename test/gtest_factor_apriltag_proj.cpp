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

        double rep_error1;
        double rep_error2;
        bool use_rotation;

        Eigen::Matrix8d meas_cov;

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
             *        K1  K2
             * 
             * Measurements are pixel projections of the landmarks tags corners (8D), associated error is pixel covariance (8x8)
             * 
             */

            
            // camera is at origin of robot and looking forward
            // robot is at (0,0,0)
            // landmark is right in front of camera. Its orientation wrt camera is identity.


            // ground truth needed for individual tests
            Vector3d    p_w_r1, p_w_r2;
            Quaterniond q_w_r1, q_w_r2;
            Vector3d    p_r_c;
            Quaterniond q_r_c;
            Vector3d    p_w_l;
            Quaterniond q_w_l;

            // robot poses
            p_w_r1 << 0,0,0;
            Vector3d euler_w_r1; euler_w_r1 << 0,0,toRad(45);
            q_w_r1 = e2q(euler_w_r1);
            p_w_r2 << 0,0,0;
            Vector3d euler_w_r2; euler_w_r2 << 0,0,0;
            q_w_r2 = e2q(euler_w_r2);
            // extrinsics
            p_r_c << 0,0,0;
            Vector3d euler_r_c; euler_r_c <<  toRad(-90), toRad(90), 0;
            q_r_c = e2q(euler_r_c);
            std::cout << "Rot extr" << std::endl;
            std::cout << q_r_c.toRotationMatrix() << std::endl;
            // landmark (same orientation as camera of robot frame 1 (r1))
            p_w_l << 0,0,0;
            Vector3d euler_w_l; euler_w_l <<  toRad(-90), toRad(90), 0;
            q_w_l = e2q(euler_w_l);

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
            C1 = std::static_pointer_cast<CaptureImage>(CaptureBase::emplace<CaptureImage>(F1, 1.0, camera, cv::Mat(2,2,CV_8UC1)));
            proc_apriltag->setOriginPtr(C1);
            proc_apriltag->setLastPtr(C1);

            meas_cov = 1*Eigen::Matrix8d::Identity();  // pixel noise
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

            double tag_width = 0.2;
            Vector8d meas =  Vector8d::Zero();

            //emplace feature and landmark
            f1 = std::static_pointer_cast<FeatureApriltagProj>(FeatureBase::emplace<FeatureApriltagProj>(C1, meas, meas_cov, det.id, tag_width, det));
            lmk1 = std::static_pointer_cast<LandmarkApriltag>(proc_apriltag->emplaceLandmark(f1));
        }
};


TEST_F(FactorApriltagProj_class, Constructor)
{
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


int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}


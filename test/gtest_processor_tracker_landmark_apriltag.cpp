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
#include "core/capture/capture_pose.h"
#include "core/processor/factory_processor.h"

#include "apriltag/processor/processor_tracker_landmark_apriltag.h"
#include "apriltag/feature/feature_apriltag_pose.h"
#include "apriltag/landmark/landmark_apriltag.h"
#include "apriltag/internal/config.h"
using namespace Eigen;
using namespace wolf;


////////////////////////////////////////////////////////////////
/*
 * Wrapper class to be able to have setOriginPtr() and setLastPtr() in ProcessorTrackerLandmarkApriltag
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
        unsigned int getMinFeaturesForKeyframe (){return min_features_for_keyframe_;}
        double getMinTimeVote (){return min_time_span_;}
        void setIncomingDetections(const FeatureBasePtrList _incoming_detections) { detections_incoming_ = _incoming_detections; }
        void setLastDetections(const FeatureBasePtrList _last_detections) { detections_last_ = _last_detections; }

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



////////////////////////////////////////////////////////////////
/*
 * Test class to prepare a little wolf problem to test the class ProcessorTrackerLandmarkApriltag
 *
 * The class ProcessorTrackerLandmarkApriltag is sometimes tested via the wrapper ProcessorTrackerLandmarkApriltag_Wrapper
 */
// Use the following in case you want to initialize tests with predefined variables or methods.
class ProcessorTrackerLandmarkApriltag_class : public testing::Test{

    public:
        ProcessorTrackerLandmarkApriltag_WrapperPtr prc_apr;
        std::string             wolf_root;
        ProblemPtr              problem;
        SensorBasePtr           sen;
        ProcessorBasePtr        prc;
        FrameBasePtr            F1;
        CaptureBasePtr          C1;
        Vector8d corners_vec_;
        int tag_id_;
        double                  rep_error1;
        double                  rep_error2;
        bool                    use_rotation;
        double decision_margin, rep_err1, rep_err2;
        Vector7d pose_default_;
        Matrix6d cov_pose_;

        void SetUp() override
        {
            wolf_root = _WOLF_APRILTAG_ROOT_DIR;

            // configure wolf problem
            problem = Problem::create("PO", 3);
            sen = problem->installSensor("SensorCamera", "camera", (Vector7d()<<0,0,0,0,0,0,1).finished(), wolf_root + "/test/camera_params_canonical.yaml");
            prc     = problem->installProcessor("ProcessorTrackerLandmarkApriltag_Wrapper", "apriltags_wrapper", "camera", wolf_root + "/test/processor_tracker_landmark_apriltag.yaml");
            prc_apr = std::static_pointer_cast<ProcessorTrackerLandmarkApriltag_Wrapper>(prc);

            // set prior
            VectorComposite x0("PO", {Vector3d(0,0,0), Quaterniond::Identity().coeffs()});
            VectorComposite s0("PO", {Vector3d(1,1,1), Vector3d(1,1,1)});
            F1 = problem->setPriorFactor(x0, s0, 0.0);

            // minimal config for the processor to be operative
            C1 = CaptureBase::emplace<CapturePose>(F1, 1.0, sen, Vector7d(), Matrix6d());
            prc_apr->setOriginPtr(C1);
            prc_apr->setLastPtr(C1);

            tag_id_ = 1;
            corners_vec_  << 1.0, -1.0,
                            1.0,  1.0,
                           -1.0,  1.0,
                           -1.0, -1.0;

            rep_error1 = 0.01;
            rep_error2 = 0.1;
            use_rotation = true;
            decision_margin = 10;
            rep_err1 = 1;
            rep_err2 = 2;
            
            // use for tests in which pose value irrelevant
            pose_default_ << 0,0,0, 0,0,0,1;
            cov_pose_.setIdentity();
        }

};
////////////////////////////////////////////////////////////////



/////////////////// TESTS START HERE ///////////////////////////
//                                                            //
TEST(ProcessorTrackerLandmarkApriltag, Constructor)
{
    ParamsProcessorTrackerLandmarkApriltagPtr params = std::make_shared<ParamsProcessorTrackerLandmarkApriltag>();

    ProcessorTrackerLandmarkApriltagPtr p;

    params->tag_family_ = "tag36h11";
    p = std::make_shared<ProcessorTrackerLandmarkApriltag>(params);
    ASSERT_TRUE(p->getTagFamily() == params->tag_family_);

    /* The following families are commented to speed up the test
    params->tag_family_ = "tag16h5";
    p = std::make_shared<ProcessorTrackerLandmarkApriltag>(params);
    ASSERT_TRUE(p->getTagFamily() == params->tag_family_);

    params->tag_family_ = "tag25h9";
    p = std::make_shared<ProcessorTrackerLandmarkApriltag>(params);
    ASSERT_TRUE(p->getTagFamily() == params->tag_family_);

    params->tag_family_ = "tagStandard41h12";
    p = std::make_shared<ProcessorTrackerLandmarkApriltag>(params);
    ASSERT_TRUE(p->getTagFamily() == params->tag_family_);

    params->tag_family_ = "tagCircle49h12";
    p = std::make_shared<ProcessorTrackerLandmarkApriltag>(params);
    ASSERT_TRUE(p->getTagFamily() == params->tag_family_);

    params->tag_family_ = "tagCustom48h12";
    p = std::make_shared<ProcessorTrackerLandmarkApriltag>(params);
    ASSERT_TRUE(p->getTagFamily() == params->tag_family_);

    params->tag_family_ = "tagStandard52h13";
    p = std::make_shared<ProcessorTrackerLandmarkApriltag>(params);
    ASSERT_TRUE(p->getTagFamily() == params->tag_family_);
    */

    params->tag_family_ = "wrong_family";
    WOLF_INFO("The following runtime error \"Unrecognized tag family name. Use e.g. \"tag36h11\".\" is expected and does not imply a failed test:");
    ASSERT_DEATH( { std::make_shared<ProcessorTrackerLandmarkApriltag>(params); }, "" );
}

// TEST_F(ProcessorTrackerLandmarkApriltag_class, voteForKeyFrame)
// {
//     //////////////////////////////////////
//     //////////////////////////////////////
//     // TODO: GTEST IMPLEMENTATION IS WRONG
//     //////////////////////////////////////
//     //////////////////////////////////////
//     double min_time_vote = prc_apr->getMinTimeVote();
//     unsigned int min_features_for_keyframe = prc_apr->getMinFeaturesForKeyframe();
//     double start_ts = 2.0;

//     CaptureBasePtr Ca = CaptureBase::emplace<CapturePose>(F1, start_ts, sen, Vector7d(), Matrix6d());
//     CaptureBasePtr Cb = CaptureBase::emplace<CapturePose>(F1, start_ts + min_time_vote/2, sen, Vector7d(), Matrix6d());
//     CaptureBasePtr Cc = CaptureBase::emplace<CapturePose>(F1, start_ts + 2*min_time_vote, sen, Vector7d(), Matrix6d());
//     CaptureBasePtr Cd = CaptureBase::emplace<CapturePose>(F1, start_ts + 2.5*min_time_vote, sen, Vector7d(), Matrix6d());
//     CaptureBasePtr Ce = CaptureBase::emplace<CapturePose>(F1, start_ts + 3*min_time_vote, sen, Vector7d(), Matrix6d());

//     for (int i=0; i < min_features_for_keyframe; i++){
//         det.id = i;
//         FeatureBase::emplace<FeatureApriltagPose>(Ca, (Vector7d()<<0,0,0,0,0,0,1).finished(), Matrix6d::Identity(), i, det, use_rotation);
//         FeatureBase::emplace<FeatureApriltagPose>(Cc, (Vector7d()<<0,0,0,0,0,0,1).finished(), Matrix6d::Identity(), i, det, use_rotation);
//         if (i != min_features_for_keyframe-1){
//             FeatureBase::emplace<FeatureApriltagPose>(Cd, (Vector7d()<<0,0,0,0,0,0,1).finished(), Matrix6d::Identity(), i, det, use_rotation);
//             FeatureBase::emplace<FeatureApriltagPose>(Ce, (Vector7d()<<0,0,0,0,0,0,1).finished(), Matrix6d::Identity(), i, det, use_rotation);
//         }
//     }

//     // CASE 1: Not enough time between origin and incoming
//     prc_apr->setOriginPtr(Ca);
//     prc_apr->setIncomingPtr(Cb);
//     ASSERT_FALSE(prc_apr->voteForKeyFrame());

//     // CASE 2: Enough time but still too many features in image to trigger a KF
//     prc_apr->setOriginPtr(Ca);
//     prc_apr->setLastPtr(Cb);
//     prc_apr->setIncomingPtr(Cc);
//     ASSERT_FALSE(prc_apr->voteForKeyFrame());

//     // // CASE 3: Enough time, enough features in last, not enough features in incoming
//     // prc_apr->setOriginPtr(Ca);
//     // prc_apr->setLastPtr(Cc);
//     // prc_apr->setIncomingPtr(Cd);
//     // ASSERT_TRUE(prc_apr->voteForKeyFrame());

//     // // CASE 4: Enough time, not enough features in last, not enough features in incoming
//     // prc_apr->setOriginPtr(Ca);
//     // prc_apr->setLastPtr(Cd);
//     // prc_apr->setIncomingPtr(Ce);
//     // ASSERT_FALSE(prc_apr->voteForKeyFrame());
// }

TEST_F(ProcessorTrackerLandmarkApriltag_class, detectNewFeaturesDuplicated)
{
    // No detected features
    FeatureBasePtrList features_out;
    prc_apr->detectNewFeatures(1, C1, features_out);
    ASSERT_EQ(features_out.size(), 0);

    // Some detected features, pose does not matter for this test
    FeatureBasePtrList features_in;

    // feature 0
    tag_id_ = 0;
    auto f0 = std::make_shared<FeatureApriltagPose>(pose_default_, cov_pose_, tag_id_, 0.1, corners_vec_, use_rotation, decision_margin, rep_err1, rep_err2);
    // feature 1 (with same id of feature 0)
    auto f1 = std::make_shared<FeatureApriltagPose>(pose_default_, cov_pose_, tag_id_, 0.1, corners_vec_, use_rotation, decision_margin, rep_err1, rep_err2);

    features_in.push_back(f0);
    features_in.push_back(f1);

    // We just added two features with the same id in the list.
    prc_apr->setLastDetections(features_in);
    // at this point we have 0 detections in last, 2 detections in incoming with same id.
    prc_apr->detectNewFeatures(2, C1, features_out);
    ASSERT_EQ(features_out.size(), 1); // detectNewFeatures should keep only one in the final list of new detected features
}

TEST_F(ProcessorTrackerLandmarkApriltag_class, detectNewFeatures)
{
    // Some detected features, pose does not matter for this test
    FeatureBasePtrList features_in;
    FeatureBasePtrList features_out;

    // feature 0
    tag_id_ = 0;
    auto f0 = std::make_shared<FeatureApriltagPose>(pose_default_, cov_pose_, tag_id_, 0.1, corners_vec_, use_rotation, decision_margin, rep_err1, rep_err2);
    // feature 1
    tag_id_ = 1;
    auto f1 = std::make_shared<FeatureApriltagPose>(pose_default_, cov_pose_, tag_id_, 0.1, corners_vec_, use_rotation, decision_margin, rep_err1, rep_err2);
    // feature 2
    tag_id_ = 2;
    auto f2 = std::make_shared<FeatureApriltagPose>(pose_default_, cov_pose_, tag_id_, 0.1, corners_vec_, use_rotation, decision_margin, rep_err1, rep_err2);

    //we add different features in the list
    features_in.push_back(f0);
    features_in.push_back(f1);
    //these features are set as the predetected features in last to processing an image
    prc_apr->setLastDetections(features_in);
    // at this point we have 0 detections in last, 2 detections in predetected features with different ids, thus we should have 2 new detected features (if max_features set to >= 2)
    prc_apr->detectNewFeatures(2, C1, features_out);
    ASSERT_EQ(features_out.size(), 2);

    // Put some of the features in the graph with emplaceLandmark() and detect some of them as well as others with detectNewFeatures() running again.
    WOLF_WARN("call to function emplaceLandmark() in unit test for detectNewFeatures().")
    WOLF_INFO("emplacing 0....");
    LandmarkBasePtr lmk0 = prc_apr->emplaceLandmark(f0);
    WOLF_INFO("emplacing 1....");
    LandmarkBasePtr lmk1 = prc_apr->emplaceLandmark(f1);

    // Add 1 one more new feature to the detection list
    features_in.push_back(f2);
    prc_apr->setLastDetections(features_in);
    // At this point we have 2 landmarks (for f0 and f1), and 3 predetected features (f0, f1 and f2).
    // Hence we should have 1 new detected feature : f2
    features_out.clear();
    WOLF_INFO("detecting....");
    prc_apr->detectNewFeatures(2, C1, features_out);
    ASSERT_EQ(features_out.size(), 1);
    ASSERT_EQ(std::static_pointer_cast<FeatureApriltagPose>(features_out.front())->getTagId(), 2);
}

TEST_F(ProcessorTrackerLandmarkApriltag_class, emplaceLandmark)
{
    Vector7d pose_landmark((Vector7d()<<1,2,3,1,0,0,0).finished());
    auto f1 = std::make_shared<FeatureApriltagPose>(pose_landmark, cov_pose_, tag_id_, 0.1, corners_vec_, use_rotation, decision_margin, rep_err1, rep_err2);

    LandmarkBasePtr lmk = prc_apr->emplaceLandmark(f1);
    LandmarkApriltagPtr lmk_april = std::static_pointer_cast<LandmarkApriltag>(lmk);

    ASSERT_TRUE(lmk_april->getMap() != nullptr);
    ASSERT_TRUE(lmk_april->getType() == "LandmarkApriltag");
    ASSERT_MATRIX_APPROX(lmk_april->getState().vector("PO"), pose_landmark, 1e-6);
}

TEST_F(ProcessorTrackerLandmarkApriltag_class, emplaceFactor)
{
    
    auto f1 = FeatureBase::emplace<FeatureApriltagPose>(C1, pose_default_, cov_pose_, tag_id_, 0.1, corners_vec_, use_rotation, decision_margin, rep_err1, rep_err2);

    LandmarkBasePtr lmk = prc_apr->emplaceLandmark(f1);
    LandmarkApriltagPtr lmk_april = std::static_pointer_cast<LandmarkApriltag>(lmk);

    auto st = prc_apr->getSensor()->getO()->getState();
    std::cout << prc_apr->getSensor()->getStructure() << std::endl;
    std::cout << st.transpose() << std::endl;

    FactorBasePtr fac = prc_apr->emplaceFactor(f1, lmk);

    ASSERT_TRUE(fac->getFeature() == f1);
    ASSERT_TRUE(fac->getType() == "FactorRelativePose3dWithExtrinsics");
}

TEST_F(ProcessorTrackerLandmarkApriltag_class, computeInformation)
{
    double cx = 320;
    double cy = 240;
    double fx = 320;
    double fy = 320;
    Eigen::Matrix3d K;
    K <<  fx,  0, cx,
          0,  fy, cy,
          0,    0,   1;
    Eigen::Vector3d t; t << 0.0, 0.0, 0.4;
    Eigen::Vector3d v; v << 0.2, 0.0, 0.0;
    double tag_width = 0.05;
    double s = tag_width/2;
    Eigen::Vector3d p1; p1 <<  s,  s, 0; // bottom right
    Eigen::Vector3d p2; p2 << -s,  s, 0; // bottom left

    // Got from Matlab code:
    // Top left corner
    Eigen::Vector3d h1_matlab; h1_matlab <<   137.5894, 105.0325, 0.4050;
    Eigen::Matrix3d J_h_T1_matlab;
    J_h_T1_matlab << 320,  0, 320,
                     0,  320, 240,
                     0,    0,   1;
    Eigen::Matrix3d J_h_R1_matlab;
    J_h_R1_matlab << 7.8405, -7.8405, -6.4106,
                     4.2910, -4.2910,  9.0325,
                     0.0245, -0.0245,  0.0050;
    // Top right corner
    Eigen::Vector3d h2_matlab; h2_matlab << 121.5894, 105.0325, 0.4050;
    Eigen::Matrix3d J_h_T2_matlab;
    J_h_T2_matlab << 320,  0, 320,
                     0,  320, 240,
                     0,    0,   1;
    Eigen::Matrix3d J_h_R2_matlab;
    J_h_R2_matlab << 7.8405, 7.8405, -9.5894,
                     4.2910, 4.2910, -9.0325,
                     0.0245, 0.0245, -0.0050;

    Eigen::Vector3d h1;
    Eigen::Matrix3d J_h_T1;
    Eigen::Matrix3d J_h_R1;
    Eigen::Vector3d h2;
    Eigen::Matrix3d J_h_T2;
    Eigen::Matrix3d J_h_R2;

    prc_apr->pinholeHomogeneous(K, t, v2R(v), p1, h1, J_h_T1, J_h_R1);
    prc_apr->pinholeHomogeneous(K, t, v2R(v), p2, h2, J_h_T2, J_h_R2);

    ASSERT_MATRIX_APPROX(h1, h1_matlab, 1e-3);
    ASSERT_MATRIX_APPROX(J_h_T1, J_h_T1_matlab, 1e-3);
    ASSERT_MATRIX_APPROX(J_h_R1, J_h_R1_matlab, 1e-3);
    ASSERT_MATRIX_APPROX(h2, h2_matlab, 1e-3);
    ASSERT_MATRIX_APPROX(J_h_T2, J_h_T2_matlab, 1e-3);
    ASSERT_MATRIX_APPROX(J_h_R2, J_h_R2_matlab, 1e-3);

    double sig_q = 2;
    Eigen::Matrix6d transformation_info = prc_apr->computeInformation(t, v2R(v), K, tag_width, sig_q);

    // From Matlab
//    Eigen::Matrix6d transformation_cov_matlab;
//    transformation_cov_matlab <<
//    0.0000,    0.0000,   -0.0000,    0.0000,   -0.0002,    0.0000,
//    0.0000,    0.0000,   -0.0000,    0.0002,    0.0000,    0.0000,
//   -0.0000,   -0.0000,    0.0004,   -0.0040,   -0.0000,    0.0000,
//    0.0000,    0.0002,   -0.0040,    0.1027,    0.0000,    0.0000,
//   -0.0002,    0.0000,   -0.0000,    0.0000,    0.1074,   -0.0106,
//    0.0000,    0.0000,    0.0000,    0.0000,   -0.0106,    0.0023;

    Eigen::Matrix6d transformation_info_matlab;
    transformation_info_matlab <<
    6.402960973553990,                   0,   0.000000000000000,  -0.000000000000000,   0.009809735541319,   0.001986080274985,
                    0,   6.402960973553990,   0.014610695222409,  -0.008824560412472,   0.000000000000000,   0.000000000000000,
    0.000000000000000,   0.014610695222409,   0.049088870761338,   0.001889201771982,   0.000000000000000,   0.000000000000000,
   -0.000000000000000,  -0.008824560412472,   0.001889201771982,   0.000183864607538,  -0.000000000000000,   0.000000000000000,
    0.009809735541319,   0.000000000000000,   0.000000000000000,  -0.000000000000000,   0.000183864607538,   0.000773944077821,
    0.001986080274985,   0.000000000000000,   0.000000000000000,  -0.000000000000000,   0.000773944077821,   0.007846814985446;

    transformation_info_matlab = transformation_info_matlab*100000.0;


    ASSERT_MATRIX_APPROX(transformation_info, transformation_info_matlab, 1e-3);


}





////////////////////////////////
////////////////////////////////
// Projection based Factor
////////////////////////////////
////////////////////////////////



TEST_F(ProcessorTrackerLandmarkApriltag_class, emplaceFactorProj)
{
    // dummy pose, not used in the factor, only for some part of the processor
    Vector7d pose_dummy = Vector7d::Zero(); 
    auto f1 = FeatureBase::emplace<FeatureApriltagProj>(C1, Vector8d::Zero(), Matrix8d::Identity(), tag_id_, 0.1, pose_dummy, use_rotation, decision_margin, rep_err1, rep_err2);

    LandmarkBasePtr lmk = prc_apr->emplaceLandmark(f1);
    LandmarkApriltagPtr lmk_april = std::static_pointer_cast<LandmarkApriltag>(lmk);

    auto st = prc_apr->getSensor()->getO()->getState();

    FactorBasePtr fac = prc_apr->emplaceFactor(f1, lmk);

    ASSERT_TRUE(fac->getFeature() == f1);
    ASSERT_TRUE(fac->getType() == "FactorApriltagProj");
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}


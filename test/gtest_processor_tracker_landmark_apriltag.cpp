#include "utils_gtest.h"

#include "base/wolf.h"
#include "base/logging.h"
#include "base/capture/capture_pose.h"
#include "base/processor/processor_factory.h"

#include "apriltag/processor/processor_tracker_landmark_apriltag.h"
#include "apriltag/feature/feature_apriltag.h"
#include "apriltag/landmark/landmark_apriltag.h"
using namespace Eigen;
using namespace wolf;
using std::static_pointer_cast;


////////////////////////////////////////////////////////////////
/*
 * Wrapper class to be able to have setOriginPtr() and setLastPtr() in ProcessorTrackerLandmarkApriltag
 */
WOLF_PTR_TYPEDEFS(ProcessorTrackerLandmarkApriltag_Wrapper);
class ProcessorTrackerLandmarkApriltag_Wrapper : public ProcessorTrackerLandmarkApriltag
{
    public:
        ProcessorTrackerLandmarkApriltag_Wrapper(ProcessorParamsTrackerLandmarkApriltagPtr _params_tracker_landmark_apriltag) :
            ProcessorTrackerLandmarkApriltag(_params_tracker_landmark_apriltag)
        {
            setType("TRACKER LANDMARK APRILTAG WRAPPER");
        };
        ~ProcessorTrackerLandmarkApriltag_Wrapper(){}
        void setOriginPtr(const CaptureBasePtr _origin_ptr) { origin_ptr_ = _origin_ptr; }
        void setLastPtr  (const CaptureBasePtr _last_ptr)   { last_ptr_ = _last_ptr; }
        void setIncomingPtr  (const CaptureBasePtr _incoming_ptr)   { incoming_ptr_ = _incoming_ptr; }
        unsigned int getMinFeaturesForKeyframe (){return min_features_for_keyframe_;}
        Scalar getMinTimeVote (){return min_time_vote_;}
        void setIncomingDetections(const FeatureBasePtrList _incoming_detections) { detections_incoming_ = _incoming_detections; }
        void setLastDetections(const FeatureBasePtrList _last_detections) { detections_last_ = _last_detections; }

        // for factory
        static ProcessorBasePtr create(const std::string& _unique_name, const ProcessorParamsBasePtr _params, const SensorBasePtr sensor_ptr = nullptr)
        {
            std::shared_ptr<ProcessorParamsTrackerLandmarkApriltag> prc_apriltag_params_;
            if (_params)
                prc_apriltag_params_ = std::static_pointer_cast<ProcessorParamsTrackerLandmarkApriltag>(_params);
            else
                prc_apriltag_params_ = std::make_shared<ProcessorParamsTrackerLandmarkApriltag>();

            ProcessorTrackerLandmarkApriltag_WrapperPtr prc_ptr = std::make_shared<ProcessorTrackerLandmarkApriltag_Wrapper>(prc_apriltag_params_);
            prc_ptr->setName(_unique_name);
            return prc_ptr;
        }

};
namespace wolf{
// Register in the Factories
WOLF_REGISTER_PROCESSOR("TRACKER LANDMARK APRILTAG WRAPPER", ProcessorTrackerLandmarkApriltag_Wrapper);
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
        virtual void SetUp()
        {
            wolf_root = _WOLF_ROOT_DIR;

            // configure wolf problem
            problem = Problem::create("PO 3D");
            sen = problem->installSensor("CAMERA", "camera", (Vector7s()<<0,0,0,0,0,0,1).finished(), wolf_root + "/src/examples/camera_params_canonical.yaml");
            prc     = problem->installProcessor("TRACKER LANDMARK APRILTAG WRAPPER", "apriltags_wrapper", "camera", wolf_root + "/src/examples/processor_tracker_landmark_apriltag.yaml");
            prc_apr = std::static_pointer_cast<ProcessorTrackerLandmarkApriltag_Wrapper>(prc);

            // set prior
            F1 = problem->setPrior((Vector7s()<<0,0,0,0,0,0,1).finished(), Matrix6s::Identity(), 0.0, 0.1);

            // minimal config for the processor to be operative
            C1 = std::make_shared<CapturePose>(1.0, sen, Vector7s(), Matrix6s());
            F1->addCapture(C1);
            prc_apr->setOriginPtr(C1);
            prc_apr->setLastPtr(C1);

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
        }

    public:
        ProcessorTrackerLandmarkApriltag_WrapperPtr prc_apr;
        std::string             wolf_root;
        ProblemPtr              problem;
        SensorBasePtr           sen;
        ProcessorBasePtr        prc;
        FrameBasePtr            F1;
        CaptureBasePtr          C1;
        apriltag_detection_t    det;
        Scalar                  rep_error1;
        Scalar                  rep_error2;
        bool                    use_rotation;
};
////////////////////////////////////////////////////////////////



/////////////////// TESTS START HERE ///////////////////////////
//                                                            //
TEST(ProcessorTrackerLandmarkApriltag, Constructor)
{
    std::string s1;
    std::string s2;

    ProcessorParamsTrackerLandmarkApriltagPtr params = std::make_shared<ProcessorParamsTrackerLandmarkApriltag>();
    params->tag_family_ = "tag36h11";
    ProcessorTrackerLandmarkApriltagPtr p = std::make_shared<ProcessorTrackerLandmarkApriltag>(params);
    ASSERT_TRUE(p->getTagFamily() == params->tag_family_);

    params->tag_family_ = "tag36h10";
    p = std::make_shared<ProcessorTrackerLandmarkApriltag>(params);
    ASSERT_TRUE(p->getTagFamily() == params->tag_family_);

    params->tag_family_ = "tag36artoolkit";
    p = std::make_shared<ProcessorTrackerLandmarkApriltag>(params);
    ASSERT_TRUE(p->getTagFamily() == "artoolkit"); // This tagfamily is stored differently by apriltag library

    params->tag_family_ = "tag25h9";
    p = std::make_shared<ProcessorTrackerLandmarkApriltag>(params);
    ASSERT_TRUE(p->getTagFamily() == params->tag_family_);

    params->tag_family_ = "tag25h7";
    p = std::make_shared<ProcessorTrackerLandmarkApriltag>(params);
    ASSERT_TRUE(p->getTagFamily() == params->tag_family_);

    params->tag_family_ = "wrong_family";
    WOLF_INFO("The following runtime error \"Unrecognized tag family name. Use e.g. \"tag36h11\".\" is expected and does not imply a failed test:");
    ASSERT_DEATH( { std::make_shared<ProcessorTrackerLandmarkApriltag>(params); }, "" );
}

TEST_F(ProcessorTrackerLandmarkApriltag_class, voteForKeyFrame)
{
    Scalar min_time_vote = prc_apr->getMinTimeVote();
    unsigned int min_features_for_keyframe = prc_apr->getMinFeaturesForKeyframe();
    Scalar start_ts = 2.0;

    CaptureBasePtr Ca = std::make_shared<CapturePose>(start_ts, sen, Vector7s(), Matrix6s());
    CaptureBasePtr Cb = std::make_shared<CapturePose>(start_ts + min_time_vote/2, sen, Vector7s(), Matrix6s());
    CaptureBasePtr Cc = std::make_shared<CapturePose>(start_ts + 2*min_time_vote, sen, Vector7s(), Matrix6s());
    CaptureBasePtr Cd = std::make_shared<CapturePose>(start_ts + 2.5*min_time_vote, sen, Vector7s(), Matrix6s());
    CaptureBasePtr Ce = std::make_shared<CapturePose>(start_ts + 3*min_time_vote, sen, Vector7s(), Matrix6s());

    for (int i=0; i < min_features_for_keyframe; i++){
        det.id = i;
        FeatureApriltagPtr f = std::make_shared<FeatureApriltag>((Vector7s()<<0,0,0,0,0,0,1).finished(), Matrix6s::Identity(), i, det, rep_error1, rep_error2, use_rotation);
        Ca->addFeature(f);
        Ca->addFeature(f);
        Cc->addFeature(f);
        if (i != min_features_for_keyframe-1){
            Cd->addFeature(f);
            Ce->addFeature(f);
        }
    }
    F1->addCapture(Ca);
    F1->addCapture(Cb);
    F1->addCapture(Cc);
    F1->addCapture(Cd);
    F1->addCapture(Ce);

    // CASE 1: Not enough time between origin and incoming
    prc_apr->setOriginPtr(Ca);
    prc_apr->setIncomingPtr(Cb);
    ASSERT_FALSE(prc_apr->voteForKeyFrame());

    // CASE 2: Enough time but still too many features in image to trigger a KF
    prc_apr->setOriginPtr(Ca);
    prc_apr->setLastPtr(Cb);
    prc_apr->setIncomingPtr(Cc);
    ASSERT_FALSE(prc_apr->voteForKeyFrame());

    // CASE 3: Enough time, enough features in last, not enough features in incoming
    prc_apr->setOriginPtr(Ca);
    prc_apr->setLastPtr(Cc);
    prc_apr->setIncomingPtr(Cd);
    ASSERT_TRUE(prc_apr->voteForKeyFrame());

    // CASE 4: Enough time, not enough features in last, not enough features in incoming
    prc_apr->setOriginPtr(Ca);
    prc_apr->setLastPtr(Cd);
    prc_apr->setIncomingPtr(Ce);
    ASSERT_FALSE(prc_apr->voteForKeyFrame());

}

TEST_F(ProcessorTrackerLandmarkApriltag_class, detectNewFeatures)
{
    // No detected features
    FeatureBasePtrList features_out;
    prc_apr->detectNewFeatures(1, features_out);
    ASSERT_EQ(features_out.size(), 0);

    // Some detected features TODO
    FeatureBasePtrList features_in;
    Eigen::Vector3s pos;
    Eigen::Vector3s ori; //Euler angles in rad
    Eigen::Quaternions quat;
    Eigen::Vector7s pose;
    Eigen::Matrix6s meas_cov( (prc_apr->getVarVec()).asDiagonal() );
    int tag_id;

    // feature 0
    pos << 0,2,0;
    ori << M_TORAD * 0, M_TORAD * 0, M_TORAD * 0;
    quat = e2q(ori);
    pose << pos, quat.coeffs();
    tag_id = 0;
    det.id = tag_id;
    FeatureBasePtr f0 = std::make_shared<FeatureApriltag>(pose, meas_cov, tag_id, det, rep_error1, rep_error2, use_rotation);

    // feature 1
    pos << 1,2,0;
    ori << M_TORAD * 0, M_TORAD * 0, M_TORAD * 0;
    quat = e2q(ori);
    pose << pos, quat.coeffs();
    tag_id = 1;
    det.id = tag_id;
    FeatureBasePtr f1 = std::make_shared<FeatureApriltag>(pose, meas_cov, tag_id, det, rep_error1, rep_error2, use_rotation);

    // feature 2
    pos << 0,2,1;
    ori << M_TORAD * 0, M_TORAD * 0, M_TORAD * 0;
    quat = e2q(ori);
    pose << pos, quat.coeffs();
    tag_id = 2;
    det.id = tag_id;
    FeatureBasePtr f2 = std::make_shared<FeatureApriltag>(pose, meas_cov, tag_id, det, rep_error1, rep_error2, use_rotation);

    features_in.push_back(f0);
    features_in.push_back(f0);

    // We just added twice the same feature in the list.
    prc_apr->setLastDetections(features_in);
    // at this point we have 0 detections in last, 2 detections in incoming with same id. We should keep only one in the final list of new detected features
    prc_apr->detectNewFeatures(2, features_out);
    ASSERT_EQ(features_out.size(), 1);

    //we add new different features in the list
    features_in.clear();
    features_in.push_back(f0);
    features_in.push_back(f1);
    //these features are set as the incoming detections due to processing an image
    prc_apr->setLastDetections(features_in);
    // at this point we have 0 detections in last, 2 detections in incoming with different ids, thus we should have 2 new detected features (if max_features set to >= 2)
    prc_apr->detectNewFeatures(2, features_out);
    ASSERT_EQ(features_out.size(), 2);

    // Put some of the features in the graph with createLandmark() and detect some of them as well as others with detectNewFeatures() running again.
    WOLF_WARN("call to function createLandmark() in unit test for detectNewFeatures().")
    C1->addFeature(f0);
    LandmarkBasePtr lmk0 = prc_apr->createLandmark(f0);
    C1->addFeature(f1);
    LandmarkBasePtr lmk1 = prc_apr->createLandmark(f1);

    // Add landmarks to the map
    LandmarkBasePtrList landmark_list;
    landmark_list.push_back(lmk0);
    landmark_list.push_back(lmk1);
    problem->addLandmarkList(landmark_list);
    //problem->print(4,1,1,1);

    // Add 1 one more new feature to the detection list
    features_in.push_back(f2);
    prc_apr->setLastDetections(features_in);
    // At this point we have 2 landmarks (for f0 and f1), and 3 detections (f0, f1 and f2).
    // Hence we should 1 new detected feature : f2
    features_out.clear();
    prc_apr->detectNewFeatures(2, features_out);
    ASSERT_EQ(features_out.size(), 1);
    ASSERT_EQ(std::static_pointer_cast<FeatureApriltag>(features_out.front())->getTagId(), 2);
}

TEST_F(ProcessorTrackerLandmarkApriltag_class, createLandmark)
{
    Vector7s pose_landmark((Vector7s()<<0,0,0,0,0,0,1).finished());
    det.id = 1;
    FeatureApriltagPtr f1 = std::make_shared<FeatureApriltag>(pose_landmark, Matrix6s::Identity(), 1, det, rep_error1, rep_error2, use_rotation);

    C1->addFeature(f1);
    LandmarkBasePtr lmk = prc_apr->createLandmark(f1);
    LandmarkApriltagPtr lmk_april = std::static_pointer_cast<LandmarkApriltag>(lmk);
    ASSERT_TRUE(lmk_april->getType() == "APRILTAG");
    ASSERT_MATRIX_APPROX(lmk_april->getState(), pose_landmark, 1e-6);
}

TEST_F(ProcessorTrackerLandmarkApriltag_class, createFactor)
{
    det.id = 1;
    FeatureApriltagPtr f1 = std::make_shared<FeatureApriltag>((Vector7s()<<0,0,0,0,0,0,1).finished(), Matrix6s::Identity(), 1, det, rep_error1, rep_error2, use_rotation);

    C1->addFeature(f1);
    LandmarkBasePtr lmk = prc_apr->createLandmark(f1);
    LandmarkApriltagPtr lmk_april = std::static_pointer_cast<LandmarkApriltag>(lmk);

    FactorBasePtr ctr = prc_apr->createFactor(f1, lmk);

    ASSERT_TRUE(ctr->getType() == "AUTODIFF APRILTAG");
}

TEST_F(ProcessorTrackerLandmarkApriltag_class, computeInformation)
{
    Scalar cx = 320;
    Scalar cy = 240;
    Scalar fx = 320;
    Scalar fy = 320;
    Eigen::Matrix3s K;
    K <<  fx,  0, cx,
          0,  fy, cy,
          0,    0,   1;
    Eigen::Vector3s t; t << 0.0, 0.0, 0.4;
    Eigen::Vector3s v; v << 0.2, 0.0, 0.0;
    Scalar tag_width = 0.05;
    Scalar s = tag_width/2;
    Eigen::Vector3s p1; p1 <<  s,  s, 0; // bottom right
    Eigen::Vector3s p2; p2 << -s,  s, 0; // bottom left

    // Got from Matlab code:
    // Top left corner
    Eigen::Vector3s h1_matlab; h1_matlab <<   137.5894, 105.0325, 0.4050;
    Eigen::Matrix3s J_h_T1_matlab;
    J_h_T1_matlab << 320,  0, 320,
                     0,  320, 240,
                     0,    0,   1;
    Eigen::Matrix3s J_h_R1_matlab;
    J_h_R1_matlab << 7.8405, -7.8405, -6.4106,
                     4.2910, -4.2910,  9.0325,
                     0.0245, -0.0245,  0.0050;
    // Top right corner
    Eigen::Vector3s h2_matlab; h2_matlab << 121.5894, 105.0325, 0.4050;
    Eigen::Matrix3s J_h_T2_matlab;
    J_h_T2_matlab << 320,  0, 320,
                     0,  320, 240,
                     0,    0,   1;
    Eigen::Matrix3s J_h_R2_matlab;
    J_h_R2_matlab << 7.8405, 7.8405, -9.5894,
                     4.2910, 4.2910, -9.0325,
                     0.0245, 0.0245, -0.0050;

    Eigen::Vector3s h1;
    Eigen::Matrix3s J_h_T1;
    Eigen::Matrix3s J_h_R1;
    Eigen::Vector3s h2;
    Eigen::Matrix3s J_h_T2;
    Eigen::Matrix3s J_h_R2;

    prc_apr->pinholeHomogeneous(K, t, v2R(v), p1, h1, J_h_T1, J_h_R1);
    prc_apr->pinholeHomogeneous(K, t, v2R(v), p2, h2, J_h_T2, J_h_R2);

    ASSERT_MATRIX_APPROX(h1, h1_matlab, 1e-3);
    ASSERT_MATRIX_APPROX(J_h_T1, J_h_T1_matlab, 1e-3);
    ASSERT_MATRIX_APPROX(J_h_R1, J_h_R1_matlab, 1e-3);
    ASSERT_MATRIX_APPROX(h2, h2_matlab, 1e-3);
    ASSERT_MATRIX_APPROX(J_h_T2, J_h_T2_matlab, 1e-3);
    ASSERT_MATRIX_APPROX(J_h_R2, J_h_R2_matlab, 1e-3);

    Scalar sig_q = 2;
    Eigen::Matrix6s transformation_info = prc_apr->computeInformation(t, v2R(v), K, tag_width, sig_q);

    // From Matlab
//    Eigen::Matrix6s transformation_cov_matlab;
//    transformation_cov_matlab <<
//    0.0000,    0.0000,   -0.0000,    0.0000,   -0.0002,    0.0000,
//    0.0000,    0.0000,   -0.0000,    0.0002,    0.0000,    0.0000,
//   -0.0000,   -0.0000,    0.0004,   -0.0040,   -0.0000,    0.0000,
//    0.0000,    0.0002,   -0.0040,    0.1027,    0.0000,    0.0000,
//   -0.0002,    0.0000,   -0.0000,    0.0000,    0.1074,   -0.0106,
//    0.0000,    0.0000,    0.0000,    0.0000,   -0.0106,    0.0023;

    Eigen::Matrix6s transformation_info_matlab;
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

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}


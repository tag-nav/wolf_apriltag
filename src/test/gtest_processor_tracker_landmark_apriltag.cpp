#include "utils_gtest.h"

#include "wolf.h"
#include "logging.h"

#include "processors/processor_tracker_landmark_apriltag.h"
#include "features/feature_apriltag.h"
#include "landmark_apriltag.h"
#include "capture_pose.h"
#include "processor_factory.h"

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
        Scalar getMinFeaturesForKeyframe (){return min_features_for_keyframe_;}
        Scalar getMinTimeVote (){return min_time_vote_;}
        void setIncomingDetections(const FeatureBaseList _incoming_detections) { detections_incoming_ = _incoming_detections; }

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
            F1 = problem->setPrior(Vector7s::Zero(), Matrix6s::Identity(), 0.0, 0.1);

            // minimal config for the processor to be operative
            C1 = std::make_shared<CapturePose>(1.0, sen, Vector7s(), Matrix6s());
            F1->addCapture(C1);
            prc_apr->setOriginPtr(C1);
            prc_apr->setLastPtr(C1);
        }

    public:
        std::string wolf_root;
        ProblemPtr   problem;
        SensorBasePtr    sen;
        ProcessorBasePtr prc;
        ProcessorTrackerLandmarkApriltag_WrapperPtr prc_apr;
        FrameBasePtr     F1;
        CaptureBasePtr   C1;
};
////////////////////////////////////////////////////////////////



/////////////////// TESTS START HERE ///////////////////////////
//                                                            //
TEST(ProcessorTrackerLandmarkApriltag, Constructor)
{
    ProcessorParamsTrackerLandmarkApriltagPtr params = std::make_shared<ProcessorParamsTrackerLandmarkApriltag>();
    params->tag_family_ = "tag36h11";
    params->name        = params->tag_family_;
    ProcessorTrackerLandmarkApriltagPtr p = std::make_shared<ProcessorTrackerLandmarkApriltag>(params);
    ASSERT_TRUE(p->getName() == params->tag_family_);

    params->tag_family_ = "tag36h10";
    params->name        = params->tag_family_;
    p = std::make_shared<ProcessorTrackerLandmarkApriltag>(params);
    ASSERT_TRUE(p->getName() == params->tag_family_);

    params->tag_family_ = "tag36artoolkit";
    params->name        = params->tag_family_;
    p = std::make_shared<ProcessorTrackerLandmarkApriltag>(params);
    ASSERT_TRUE(p->getName() == params->tag_family_);

    params->tag_family_ = "tag25h9";
    params->name        = params->tag_family_;
    p = std::make_shared<ProcessorTrackerLandmarkApriltag>(params);
    ASSERT_TRUE(p->getName() == params->tag_family_);

    params->tag_family_ = "tag25h7";
    params->name        = params->tag_family_;
    p = std::make_shared<ProcessorTrackerLandmarkApriltag>(params);
    ASSERT_TRUE(p->getName() == params->tag_family_);

    params->tag_family_ = "wrong_family";
    params->name        = params->tag_family_;
    WOLF_INFO("The following runtime error \"Unrecognized tag family name. Use e.g. \"tag36h11\".\" is expected and does not imply a failed test:");
    ASSERT_DEATH( { std::make_shared<ProcessorTrackerLandmarkApriltag>(params); }, "" );
}

TEST_F(ProcessorTrackerLandmarkApriltag_class, voteForKeyFrame)
{
    Scalar min_time_vote = prc_apr->getMinTimeVote();
//    TODO: use min_features_for_keyframe (here "hardcoded" to 1, needs to create features with a loop)
//    unsigned int min_features_for_keyframe = prc_apr->prc_apriltag_params_->min_features_for_keyframe;
    Scalar start_ts = 2.0;

    FeatureApriltagPtr f1 = std::make_shared<FeatureApriltag>((Vector7s()<<0,0,0,0,0,0,1).finished(), Matrix6s::Identity(), 1);
    FeatureApriltagPtr f2 = std::make_shared<FeatureApriltag>((Vector7s()<<0,0,0,0,0,0,1).finished(), Matrix6s::Identity(), 2);

    CaptureBasePtr Ca = std::make_shared<CapturePose>(start_ts, sen, Vector7s(), Matrix6s());
    Ca->addFeature(f1);
    Ca->addFeature(f2);  // not really necessary

    CaptureBasePtr Cb = std::make_shared<CapturePose>(start_ts + min_time_vote/2, sen, Vector7s(), Matrix6s());
    Cb->addFeature(f1);

    CaptureBasePtr Cc = std::make_shared<CapturePose>(start_ts + 2*min_time_vote, sen, Vector7s(), Matrix6s());
    Cc->addFeature(f1);

    CaptureBasePtr Cd = std::make_shared<CapturePose>(start_ts + 2.5*min_time_vote, sen, Vector7s(), Matrix6s());

    CaptureBasePtr Ce = std::make_shared<CapturePose>(start_ts + 3*min_time_vote, sen, Vector7s(), Matrix6s());

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
    FeatureBaseList features_out;
    prc_apr->detectNewFeatures(1, features_out);
    ASSERT_EQ(features_out.size(), 0);

    // Some detected features TODO
    FeatureBaseList features_in;
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
    FeatureBasePtr f0 = std::make_shared<FeatureApriltag>(pose, meas_cov, tag_id);

    // feature 1
    pos << 1,2,0;
    ori << M_TORAD * 0, M_TORAD * 0, M_TORAD * 0;
    quat = e2q(ori);
    pose << pos, quat.coeffs();
    tag_id = 1;
    FeatureBasePtr f1 = std::make_shared<FeatureApriltag>(pose, meas_cov, tag_id);

    // feature 2
    pos << 0,2,1;
    ori << M_TORAD * 0, M_TORAD * 0, M_TORAD * 0;
    quat = e2q(ori);
    pose << pos, quat.coeffs();
    tag_id = 2;
    FeatureBasePtr f2 = std::make_shared<FeatureApriltag>(pose, meas_cov, tag_id);

    features_in.push_back(f0);
    features_in.push_back(f0);

    // We just added twice the same feature in the list.
    prc_apr->setIncomingDetections(features_in);
    // at this point we have 0 detections in last, 2 detections in incoming with same id. We should keep only one in the final list of new detected features
    prc_apr->detectNewFeatures(2, features_out);
    ASSERT_EQ(features_out.size(), 1);

    //we add new different features in the list
    features_in.clear();
    features_in.push_back(f0);
    features_in.push_back(f1);
    //these features are set as the incoming detections due to processing an image
    prc_apr->setIncomingDetections(features_in);
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
    LandmarkBaseList landmark_list;
    landmark_list.push_back(lmk0);
    landmark_list.push_back(lmk1);
    problem->addLandmarkList(landmark_list);
    //problem->print(4,1,1,1);

    // Add 1 one more new feature to the detection list
    features_in.push_back(f2);
    prc_apr->setIncomingDetections(features_in);
    // At this point we have 2 landmarks (for f0 and f1), and 3 detections (f0, f1 and f2).
    // Hence we should 1 new detected feature : f2
    features_out.clear();
    prc_apr->detectNewFeatures(2, features_out);
    ASSERT_EQ(features_out.size(), 1);
    ASSERT_EQ(std::static_pointer_cast<FeatureApriltag>(features_out.front())->getTagId(), 2);
}

TEST_F(ProcessorTrackerLandmarkApriltag_class, createLandmark)
{
    FeatureApriltagPtr f1 = std::make_shared<FeatureApriltag>((Vector7s()<<0,0,0,0,0,0,1).finished(), Matrix6s::Identity(), 1);

    C1->addFeature(f1);
    LandmarkBasePtr lmk = prc_apr->createLandmark(f1);
    LandmarkApriltagPtr lmk_april = std::static_pointer_cast<LandmarkApriltag>(lmk);
    ASSERT_TRUE(lmk_april->getType() == "APRILTAG");
}

TEST_F(ProcessorTrackerLandmarkApriltag_class, createConstraint)
{
    FeatureApriltagPtr f1 = std::make_shared<FeatureApriltag>((Vector7s()<<0,0,0,0,0,0,1).finished(), Matrix6s::Identity(), 1);

    C1->addFeature(f1);
    LandmarkBasePtr lmk = prc_apr->createLandmark(f1);
    LandmarkApriltagPtr lmk_april = std::static_pointer_cast<LandmarkApriltag>(lmk);

    ConstraintBasePtr ctr = prc_apr->createConstraint(f1, lmk);

    ASSERT_TRUE(ctr->getType() == "AUTODIFF APRILTAG");
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}


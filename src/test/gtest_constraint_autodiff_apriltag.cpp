#include "utils_gtest.h"

#include "wolf.h"
#include "logging.h"

#include "ceres_wrapper/ceres_manager.h"
#include "processors/processor_tracker_landmark_apriltag.h"
#include "capture_image.h"
#include "constraints/constraint_autodiff_apriltag.h"
#include "processor_factory.h"

#include <apriltag.h>

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

// Use the following in case you want to initialize tests with predefines variables or methods.
class ConstraintAutodiffApriltag_class : public testing::Test{
    public:
        Vector3s    pos_camera,   pos_robot,   pos_landmark;
        Vector3s    euler_camera, euler_robot, euler_landmark;
        Quaternions quat_camera,  quat_robot,  quat_landmark;
        Vector4s    vquat_camera, vquat_robot, vquat_landmark; // quaternions as vectors
        Vector7s    pose_camera,  pose_robot,  pose_landmark;

        ProblemPtr      problem;
        CeresManagerPtr ceres_manager;

        SensorCameraPtr camera;
        ProcessorTrackerLandmarkApriltag_WrapperPtr proc_apriltag;

        SensorBasePtr   S;
        FrameBasePtr    F1;
        CaptureImagePtr C1;
        FeatureApriltagPtr  f1;
        LandmarkApriltagPtr lmk1;
        ConstraintAutodiffApriltagPtr c_tag;

        virtual void SetUp()
        {
            std::string wolf_root = _WOLF_ROOT_DIR;

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
             *   S: pos = (0,0,0), ori = (-pi/1, 0, -pi/1)
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
            euler_camera    << -M_PI_2, 0, -M_PI_2;
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
            problem = Problem::create("PO 3D");
            ceres::Solver::Options options;
            ceres_manager = std::make_shared<CeresManager>(problem, options);

            // Install sensor and processor
            S      = problem->installSensor("CAMERA", "canonical", pose_camera, wolf_root + "/src/examples/camera_params_canonical.yaml");
            camera = std::static_pointer_cast<SensorCamera>(S);

            ProcessorParamsTrackerLandmarkApriltagPtr params = std::make_shared<ProcessorParamsTrackerLandmarkApriltag>();
            // Need to set some parameters ? do it now !
            params->tag_family_ = "tag36h11";
            //params->name        = params->tag_family_;

            ProcessorBasePtr proc = problem->installProcessor("TRACKER LANDMARK APRILTAG WRAPPER", "apriltags_wrapper", camera, params);
            proc_apriltag = std::static_pointer_cast<ProcessorTrackerLandmarkApriltag_Wrapper>(proc);

            // F1 is be origin KF
            F1 = problem->setPrior(pose_robot, Matrix6s::Identity(), 0.0, 0.1);

            //create feature and landmark
            C1 = std::make_shared<CaptureImage>(1.0, camera, cv::Mat(2,2,CV_8UC1));
            F1-> addCapture(C1);
            proc_apriltag->setOriginPtr(C1);
            proc_apriltag->setLastPtr(C1);

            // the sensor is at origin as well as the robot. The measurement matches with the pose of the tag wrt camera (and also wrt robot and world)
            // FeatureApriltag(Vector7s & _measurement,Matrix6s & _meas_covariance, const int _tag_id)
            Eigen::Matrix6s meas_cov(Eigen::Matrix6s::Identity());
            meas_cov.topLeftCorner(3,3)     *= 1e-2;
            meas_cov.bottomRightCorner(3,3) *= 1e-3;
            int tag_id = 1;

            f1 = std::make_shared<FeatureApriltag>(pose_landmark, meas_cov, tag_id);
            lmk1 = std::static_pointer_cast<LandmarkApriltag>(proc_apriltag->createLandmark(f1));

            // Add the feature and the landmark in the graph as needed
            C1->addFeature(f1); // add feature to capture
            problem->addLandmark(lmk1); // add landmark to map
        }
};


TEST_F(ConstraintAutodiffApriltag_class, Constructor)
{
    ConstraintAutodiffApriltagPtr constraint = std::make_shared<ConstraintAutodiffApriltag>(
            S,
            F1,
            lmk1,
            f1,
            false,
            CTR_ACTIVE
    );

    ASSERT_TRUE(constraint->getType() == "AUTODIFF APRILTAG");
}

TEST_F(ConstraintAutodiffApriltag_class, Check_tree)
{
    ConstraintAutodiffApriltagPtr constraint = std::make_shared<ConstraintAutodiffApriltag>(
            S,
            F1,
            lmk1,
            f1,
            false,
            CTR_ACTIVE
    );

    ConstraintAutodiffApriltagPtr ctr0 = std::static_pointer_cast<ConstraintAutodiffApriltag>(f1->addConstraint(constraint));
    lmk1->addConstrainedBy(constraint);
    //check is returning true even without the lines below....
    WOLF_WARN("I think the lines below are needed... to be checked !")
    F1->addConstrainedBy(constraint);
    f1->addConstrainedBy(constraint);

    ASSERT_TRUE(problem->check(0));
}

TEST_F(ConstraintAutodiffApriltag_class, solve_f1_P_perturbated)
{
    ConstraintAutodiffApriltagPtr constraint = std::make_shared<ConstraintAutodiffApriltag>(
            S,
            F1,
            lmk1,
            f1,
            false,
            CTR_ACTIVE
    );

    ConstraintAutodiffApriltagPtr ctr0 = std::static_pointer_cast<ConstraintAutodiffApriltag>(f1->addConstraint(constraint));
    lmk1->addConstrainedBy(constraint);
    F1->addConstrainedBy(constraint);
    f1->addConstrainedBy(constraint);

    // unfix F1, perturbate state
    F1->unfix();
    Vector3s p0 = Vector3s::Random() * 0.25;
//    WOLF_DEBUG("Perturbation: ")
//    WOLF_DEBUG(p0.transpose());
    Vector7s x0(pose_robot);

    x0.head<3>() += p0;
    WOLF_DEBUG("State before perturbation: ");
    WOLF_DEBUG(F1->getState().transpose());
    F1->setState(x0);
//    WOLF_DEBUG("State after perturbation: ");
//    WOLF_DEBUG(F1->getState().transpose());

//    solve
    std::string report = ceres_manager->solve(SolverManager::ReportVerbosity::QUIET); // 0: nothing, 1: BriefReport, 2: FullReport
//    WOLF_DEBUG("State after solve: ");
//    WOLF_DEBUG(F1->getState().transpose());
    ASSERT_MATRIX_APPROX(F1->getState(), pose_robot, 1e-6);

}

TEST_F(ConstraintAutodiffApriltag_class, solve_f1_O_perturbated)
{
    ConstraintAutodiffApriltagPtr constraint = std::make_shared<ConstraintAutodiffApriltag>(
            S,
            F1,
            lmk1,
            f1,
            false,
            CTR_ACTIVE
    );

    ConstraintAutodiffApriltagPtr ctr0 = std::static_pointer_cast<ConstraintAutodiffApriltag>(f1->addConstraint(constraint));
    lmk1->addConstrainedBy(constraint);
    F1->addConstrainedBy(constraint);
    f1->addConstrainedBy(constraint);

    // unfix F1, perturbate state
    F1->unfix();
    Vector3s e0 = euler_robot + Vector3s::Random() * 0.25;
    Quaternions e0_quat     = e2q(e0);
    Vector4s e0_vquat = e0_quat.coeffs();
//    WOLF_DEBUG("Perturbation: ")
//    WOLF_DEBUG(e0.transpose());
    Vector7s x0(pose_robot);

    x0.tail<4>() = e0_vquat;
    WOLF_DEBUG("State before perturbation: ");
    WOLF_DEBUG(F1->getState().transpose());
    F1->setState(x0);
//    WOLF_DEBUG("State after perturbation: ");
//    WOLF_DEBUG(F1->getState().transpose());

//    solve
    std::string report = ceres_manager->solve(SolverManager::ReportVerbosity::QUIET); // 0: nothing, 1: BriefReport, 2: FullReport
//    WOLF_DEBUG("State after solve: ");
//    WOLF_DEBUG(F1->getState().transpose());
    ASSERT_MATRIX_APPROX(F1->getState(), pose_robot, 1e-6);

}

//[Class methods]

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}


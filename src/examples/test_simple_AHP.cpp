/**
 * \file test_simple_AHP.cpp
 *
 *  Created on: 2 nov. 2016
 *      \author: jtarraso
 */

#include "wolf.h"
#include "frame_base.h"
#include "sensor_camera.h"
#include "pinholeTools.h"
#include "rotations.h"
#include "capture_image.h"
#include "landmark_AHP.h"
#include "constraint_AHP.h"
#include "ceres_wrapper/ceres_manager.h"

/**
 * This test simulates the following situation:
 *   - A kf at the origin, we use it as anchor: kf1
 *   - A kf at the origin, we use it to project lmks onto the anchor frame: kf2
 *   - A kf at 1m to the right of the origin, kf3
 *   - A kf at 1m to the left of the origin, kf4
 *   - A lmk at 1m distance in front of the anchor: L1
 *     - we use this lmk to project it onto kf2, kf3 and kf4 and obtain measured pixels p0, p1 and p2
 *   - A lmk initialized at kf1, with measurement p0, at a distance of 2m: L2
 *     - we project the pixels p3 and p4: we observe that they do not match p1 and p2
 *     - we use the measurements p1 and p2 to solve the optimization problem on L2: L2 should converge to L1.
 *   - This is a sketch of the situation:
 *     - X, Y are the axes in world frame
 *     - x, z are the axes in anchor camera frame. We have that X=z, Y=-x.
 *     - Axes Z and y are perpendicular to the drawing; they have no effect.
 *
 *                X,z
 *                 ^
 *                 |
 *              L2 * 2
 *                .|.
 *               . | .
 *              .  |  .
 *             .   |   .
 *            . L1 * 1  .
 *           .   . | .   .
 *          .  .   |   .  .
 *      p4 . .     |     . . p3
 *        .. p2    |    p1 ..
 *  Y <--+---------+---------+
 * -x   +1         0        -1
 *      kf4      kf1        kf3
 *               kf2
 *
 *      camera: (uo,vo) = (320,240)
 *              (au,av) = (320,320)
 *
 *      projection geometry:
 *
 *     1:1  2:1  1:0  2:1  1:1
 *      0   160  320  480  640
 *      +----+----+----+----+
 *                |
 *                |
 *                | 320
 *                |
 *                *
 *
 *
 *      projected pixels:
 *      p0 = (320,240) // at optical axis or relation 1:0
 *      p1 = ( 0 ,240) // at 45 deg or relation 1:1
 *      p2 = (640,240) // at 45 deg or relation 1:1
 *      p3 = (160,240) // at a relation 2:1
 *      p4 = (480,240) // at a relation 2:1
 *
 */
int main(int argc, char** argv)
{
    using namespace wolf;
    using namespace Eigen;

    /* 1. crear 2 kf, fixed
     * 2. crear 1 sensor
     * 3. crear 1 lmk1
     * 4. projectar lmk sobre sensor a fk1
     * 5. projectar lmk sobre sensor a kf4
     * 6. // esborrar lmk lmk_ptr->remove() no cal
     * 7. crear nous kf
     * 8. crear captures
     * 9. crear features amb les mesures de 4 i 5
     * 10. crear lmk2 des de kf3
     * 11. crear constraint des del kf4 a lmk2, amb ancora al kf3
     * 12. solve
     * 13. lmk1 == lmk2 ?
     */

    // ============================================================================================================
    /* 1 */
    ProblemPtr problem = Problem::create(FRM_PO_3D);
    // One anchor frame to define the lmk, and a copy to make a constraint
    FrameBasePtr kf_1 = problem->emplaceFrame(KEY_FRAME,(Vector7s()<<0,0,0,0,0,0,1).finished(), TimeStamp(0));
    FrameBasePtr kf_2 = problem->emplaceFrame(KEY_FRAME,(Vector7s()<<0,0,0,0,0,0,1).finished(), TimeStamp(0));
    // and two other frames to observe the lmk
    FrameBasePtr kf_3 = problem->emplaceFrame(KEY_FRAME,(Vector7s()<<0,-1,0,0,0,0,1).finished(), TimeStamp(0));
    FrameBasePtr kf_4 = problem->emplaceFrame(KEY_FRAME,(Vector7s()<<0,+1,0,0,0,0,1).finished(), TimeStamp(0));

    kf_1->fix();
    kf_2->fix();
    kf_3->fix();
    kf_4->fix();
    // ============================================================================================================

    // ============================================================================================================
    /* 2 */
    std::string wolf_root = _WOLF_ROOT_DIR;
    SensorBasePtr sensor_ptr = problem->installSensor("CAMERA", "PinHole", (Vector7s()<<0,0,0,-0.5,0.5,-0.5,0.5).finished(), wolf_root + "/src/examples/camera_params_ueye_sim.yaml");
    SensorCamera::Ptr camera = std::static_pointer_cast<SensorCamera>(sensor_ptr);
    // ============================================================================================================

    // ============================================================================================================
    /* 3 */
    Eigen::Vector3s lmk_dir = {0,0,1}; // in the optical axis of the anchor camera at kf1
    std::cout << std::endl << "lmk: " << lmk_dir.transpose() << std::endl;
    lmk_dir.normalize();
    Eigen::Vector4s lmk_hmg_c;
    Scalar distance = 1.0; // from anchor at kf1
    lmk_hmg_c = {lmk_dir(0),lmk_dir(1),lmk_dir(2),(1/distance)};
//    std::cout << "lmk hmg in C frame: " << lmk_hmg_c.transpose() << std::endl;
    // ============================================================================================================


    // Captures------------------
    cv::Mat cv_image;
    cv_image.zeros(2,2,0);
    CaptureImagePtr image_0 = std::make_shared<CaptureImage>(TimeStamp(0), camera, cv_image);
    CaptureImagePtr image_1 = std::make_shared<CaptureImage>(TimeStamp(1), camera, cv_image);
    CaptureImagePtr image_2 = std::make_shared<CaptureImage>(TimeStamp(2), camera, cv_image);
    kf_2->addCapture(image_0);
    kf_3->addCapture(image_1);
    kf_4->addCapture(image_2);

    // Features-----------------
    cv::Mat desc;

    cv::KeyPoint kp_0;
    std::shared_ptr<FeaturePointImage> feat_0 = std::make_shared<FeaturePointImage>(kp_0, desc, Eigen::Matrix2s::Identity());
    image_0->addFeature(feat_0);

    cv::KeyPoint kp_1;
    std::shared_ptr<FeaturePointImage> feat_1 = std::make_shared<FeaturePointImage>(kp_1, desc, Eigen::Matrix2s::Identity());
    image_1->addFeature(feat_1);

    cv::KeyPoint kp_2;
    std::shared_ptr<FeaturePointImage> feat_2 = std::make_shared<FeaturePointImage>(kp_2, desc, Eigen::Matrix2s::Identity());
    image_2->addFeature(feat_2);

    // Landmark--------------------
    std::shared_ptr<LandmarkAHP> lmk_1 = std::make_shared<LandmarkAHP>(lmk_hmg_c, kf_1, camera, desc);
    problem->addLandmark(lmk_1);
    lmk_1->setStatus(LANDMARK_FIXED);
    std::cout << "Landmark 1: " << lmk_1->point().transpose() << std::endl;

    // Constraints------------------
    ConstraintAHPPtr ctr_0 = ConstraintAHP::create(feat_0, lmk_1 );
    feat_0->addConstraint(ctr_0);
    ConstraintAHPPtr ctr_1 = ConstraintAHP::create(feat_1, lmk_1 );
    feat_1->addConstraint(ctr_1);
    ConstraintAHPPtr ctr_2 = ConstraintAHP::create(feat_2, lmk_1 );
    feat_2->addConstraint(ctr_2);

    // Projections----------------------------
    Eigen::VectorXs pix_0 = ctr_0->expectation();
    kp_0 = cv::KeyPoint(pix_0(0), pix_0(1), 0);
    feat_0->setKeypoint(kp_0);

    Eigen::VectorXs pix_1 = ctr_1->expectation();
    kp_1 = cv::KeyPoint(pix_1(0), pix_1(1), 0);
    feat_1->setKeypoint(kp_1);

    Eigen::VectorXs pix_2 = ctr_2->expectation();
    kp_2 = cv::KeyPoint(pix_2(0), pix_2(1), 0);
    feat_2->setKeypoint(kp_2);

    std::cout << "pixel 0: " << pix_0.transpose() << std::endl;
    std::cout << "pixel 1: " << pix_1.transpose() << std::endl;
    std::cout << "pixel 2: " << pix_2.transpose() << std::endl;
    //
    //======== up to here the initial projections ==============

    std::cout << "\n";

    //======== now we want to estimate a new lmk ===============
    //
    // Features -----------------
    std::shared_ptr<FeaturePointImage> feat_3 = std::make_shared<FeaturePointImage>(kp_1, desc, Eigen::Matrix2s::Identity());
    image_1->addFeature(feat_3);

    std::shared_ptr<FeaturePointImage> feat_4 = std::make_shared<FeaturePointImage>(kp_2, desc, Eigen::Matrix2s::Identity());
    image_2->addFeature(feat_4);


    // New landmark with measured pixels from kf2 (anchor) kf3 and kf4 (measurements)
    Scalar unknown_distance = 2; // the real distance is 1
    Matrix3s K = camera->getIntrinsicMatrix();
    Vector3s pix_0_hmg;
    pix_0_hmg << pix_0, 1;
    Eigen::Vector3s dir_0 = K.inverse() * pix_0_hmg;
    Eigen::Vector4s pnt_hmg_0;
    pnt_hmg_0 << dir_0, 1/unknown_distance;
    LandmarkAHPPtr lmk_2( std::make_shared<LandmarkAHP>(pnt_hmg_0, kf_2, camera, desc) );
    problem->addLandmark(lmk_2);
    std::cout << "Landmark 2: " << lmk_2->point().transpose() << std::endl;

    // New constraints from kf3 and kf4
    ConstraintAHPPtr ctr_3 = ConstraintAHP::create(feat_3, lmk_2 );
    feat_3->addConstraint(ctr_3);
    ConstraintAHPPtr ctr_4 = ConstraintAHP::create(feat_4, lmk_2 );
    feat_4->addConstraint(ctr_4);

    Eigen::Vector2s pix_3 = ctr_3->expectation();
    Eigen::Vector2s pix_4 = ctr_4->expectation();

    std::cout << "pix 3: " << pix_3.transpose() << std::endl;
    std::cout << "pix 4: " << pix_4.transpose() << std::endl;

    // Wolf tree status ----------------------
    problem->print(3);
//    problem->check();






    // ========== solve ==================================================================================================
    /* 12 */
    // Ceres wrapper
    ceres::Solver::Options ceres_options;
    ceres_options.minimizer_type = ceres::TRUST_REGION; //ceres::TRUST_REGION;LINE_SEARCH
    ceres_options.max_line_search_step_contraction = 1e-3;
    //    ceres_options.minimizer_progress_to_stdout = false;
    //    ceres_options.line_search_direction_type = ceres::LBFGS;
    //    ceres_options.max_num_iterations = 100;
    google::InitGoogleLogging(argv[0]);

    CeresManager ceres_manager(problem, ceres_options);


    ceres::Solver::Summary summary = ceres_manager.solve();
    std::cout << summary.FullReport() << std::endl;

    // Test of convergence over the lmk params
    bool pass = (lmk_2->point() - lmk_1->point()).isMuchSmallerThan(1,Constants::EPS);

    std::cout << "Landmark 2 below should have converged to Landmark 1:" << std::endl;
    std::cout << "Landmark 1: " << lmk_1->point().transpose() << std::endl;
    std::cout << "Landmark 2: " << lmk_2->point().transpose() << std::endl;
    std::cout << "Landmark convergence test " << (pass ? "PASSED" : "FAILED") << std::endl;
    std::cout << std::endl;

    if (!pass)
        return -1;
    return 0;

}

















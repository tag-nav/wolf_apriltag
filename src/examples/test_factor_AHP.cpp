#include "base/math/pinhole_tools.h"
#include "base/landmark/landmark_AHP.h"
#include "base/factor/factor_AHP.h"
#include "base/state_block/state_block.h"
#include "base/state_block/state_quaternion.h"
#include "base/sensor/sensor_camera.h"
#include "base/capture/capture_image.h"

int main()
{
    using namespace wolf;

    std::cout << std::endl << "==================== factor AHP test ======================" << std::endl;

    TimeStamp t = 1;

    //=====================================================
    // Environment variable for configuration files
    std::string wolf_root = _WOLF_ROOT_DIR;
    std::cout << wolf_root << std::endl;
    //=====================================================

    // Wolf problem
    ProblemPtr wolf_problem_ptr_ = Problem::create("PO 3D");

    /* Do this while there aren't extrinsic parameters on the yaml */
    Eigen::Vector7s extrinsic_cam;
    extrinsic_cam.setRandom();
//    extrinsic_cam[0] = 0; //px
//    extrinsic_cam[1] = 0; //py
//    extrinsic_cam[2] = 0; //pz
//    extrinsic_cam[3] = 0; //qx
//    extrinsic_cam[4] = 0; //qy
//    extrinsic_cam[5] = 0; //qz
//    extrinsic_cam[6] = 1; //qw
    extrinsic_cam.tail<4>().normalize();
    std::cout << "========extrinsic_cam: " << extrinsic_cam.transpose() << std::endl;
    const Eigen::VectorXs extr = extrinsic_cam;
    /* Do this while there aren't extrinsic parameters on the yaml */

    SensorBasePtr sensor_ptr = wolf_problem_ptr_->installSensor("CAMERA", "PinHole", extr, wolf_root + "/src/examples/camera_params_ueye_radial_dist.yaml");
    SensorCameraPtr camera_ptr_ = std::static_pointer_cast<SensorCamera>(sensor_ptr);

    // PROCESSOR
    // one-liner API
    ProcessorBasePtr processor_ptr = wolf_problem_ptr_->installProcessor("IMAGE LANDMARK", "ORB", "PinHole", wolf_root + "/src/examples/processor_image_feature.yaml");

    // create the current frame
    Eigen::Vector7s frame_pos_ori;
    frame_pos_ori.setRandom();
//    frame_pos_ori[0] = 0; //px
//    frame_pos_ori[1] = 0; //py
//    frame_pos_ori[2] = 0; //pz
//    frame_pos_ori[3] = 0; //qx
//    frame_pos_ori[4] = 0; //qy
//    frame_pos_ori[5] = 0; //qz
//    frame_pos_ori[6] = 1; //qw
    frame_pos_ori.tail<4>().normalize();
    const Eigen::VectorXs frame_val = frame_pos_ori;

    FrameBasePtr last_frame = std::make_shared<FrameBase>(t,std::make_shared<StateBlock>(frame_val.head(3)), std::make_shared<StateQuaternion>(frame_val.tail(4)));
    std::cout << "Last frame" << std::endl;
    wolf_problem_ptr_->getTrajectory()->addFrame(last_frame);

    // Capture
    CaptureImagePtr image_ptr;
    t.setToNow();
    cv::Mat frame; //puede que necesite una imagen

    image_ptr = std::make_shared< CaptureImage>(t, camera_ptr_, frame);
    last_frame->addCapture(image_ptr);

    // create the feature
    cv::KeyPoint kp; kp.pt = {10,20};
    cv::Mat desc;

    FeaturePointImagePtr feat_point_image_ptr = std::make_shared<FeaturePointImage>(kp, 0, desc, Eigen::Matrix2s::Identity());
    image_ptr->addFeature(feat_point_image_ptr);

    FrameBasePtr anchor_frame = std::make_shared< FrameBase>(t,std::make_shared<StateBlock>(frame_val.head(3)), std::make_shared<StateQuaternion>(frame_val.tail(4)));
    //FrameBasePtr anchor_frame = wolf_problem_ptr_->getTrajectory()->getLastFrame();

    // create the landmark
    Eigen::Vector2s point2D;
    point2D[0] = feat_point_image_ptr->getKeypoint().pt.x;
    point2D[1] = feat_point_image_ptr->getKeypoint().pt.y;
    std::cout << "point2D: " << point2D.transpose() << std::endl;

    Scalar distance = 2; // arbitrary value
    Eigen::Vector4s vec_homogeneous;

    Eigen::VectorXs correction_vec = (std::static_pointer_cast<SensorCamera>(image_ptr->getSensor()))->getCorrectionVector();
    std::cout << "correction vector: " << correction_vec << std::endl;
    std::cout << "distortion vector: " << (std::static_pointer_cast<SensorCamera>(image_ptr->getSensor()))->getDistortionVector() << std::endl;
    point2D = pinhole::depixellizePoint(image_ptr->getSensor()->getIntrinsic()->getState(),point2D);
    std::cout << "point2D depixellized: " << point2D.transpose() << std::endl;
    point2D = pinhole::undistortPoint((std::static_pointer_cast<SensorCamera>(image_ptr->getSensor()))->getCorrectionVector(),point2D);
    std::cout << "point2D undistorted: " << point2D.transpose() << std::endl;

    Eigen::Vector3s point3D;
    point3D.head(2) = point2D;
    point3D(2) = 1;

    point3D.normalize();
    std::cout << "point3D normalized: " << point3D.transpose() << std::endl;

    vec_homogeneous = {point3D(0),point3D(1),point3D(2),1/distance};
    std::cout << "vec_homogeneous: " << vec_homogeneous.transpose() << std::endl;

    LandmarkAHPPtr landmark = std::make_shared<LandmarkAHP>(vec_homogeneous, anchor_frame, image_ptr->getSensor(), feat_point_image_ptr->getDescriptor());

    std::cout << "Landmark AHP created" << std::endl;

    // Create the factor
    FactorAHPPtr factor_ptr = std::make_shared<FactorAHP>(feat_point_image_ptr, std::static_pointer_cast<LandmarkAHP>(landmark), processor_ptr);

    feat_point_image_ptr->addFactor(factor_ptr);
    std::cout << "Factor AHP created" << std::endl;

    Eigen::Vector2s point2D_proj = point3D.head<2>()/point3D(2);
    std::cout << "point2D projected: " << point2D_proj.transpose() << std::endl;

    Eigen::Vector2s point2D_dist;
    point2D_dist =  pinhole::distortPoint((std::static_pointer_cast<SensorCamera>(image_ptr->getSensor()))->getDistortionVector(),point2D_proj);
    std::cout << "point2D distorted: " << point2D_dist.transpose() << std::endl;

    Eigen::Vector2s point2D_pix;
    point2D_pix = pinhole::pixellizePoint(image_ptr->getSensor()->getIntrinsic()->getState(),point2D_dist);
    std::cout << "point2D pixellized: " << point2D_pix.transpose() << std::endl;

    Eigen::Vector2s expectation;
    Eigen::Vector3s current_frame_p = last_frame->getP()->getState();
    Eigen::Vector4s current_frame_o = last_frame->getO()->getState();
    Eigen::Vector3s anchor_frame_p = landmark->getAnchorFrame()->getP()->getState();
    Eigen::Vector4s anchor_frame_o = landmark->getAnchorFrame()->getO()->getState();
    Eigen::Vector4s landmark_ = landmark->getP()->getState();

    factor_ptr ->expectation(current_frame_p.data(), current_frame_o.data(),
            anchor_frame_p.data(), anchor_frame_o.data(),
            landmark_.data(), expectation.data());
//    current_frame p; current_frame o; anchor_frame p; anchor_frame o; homogeneous_vector landmark, residual

    std::cout << "expectation computed" << std::endl;
    std::cout << "expectation = " << expectation[0] << "   " << expectation[1] << std::endl;

    return 0;

}

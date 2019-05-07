// Testing things for the 3D image odometry

//Wolf includes
#include "core/sensor/sensor_camera.h"
#include "core/capture/capture_image.h"
#include "core/processor/processor_tracker_feature_image.h"
#include "core/ceres_wrapper/ceres_manager.h"

// Vision utils includes
#include <vision_utils.h>
#include <sensors.h>
#include <common_class/buffer.h>
#include <common_class/frame.h>

//std includes
#include <ctime>
#include <iostream>
#include <string>

int main(int argc, char** argv)
{
    using namespace wolf;
    using std::shared_ptr;
    using std::make_shared;
    using std::static_pointer_cast;

    //ProcessorImageFeature test
    std::cout << std::endl << " ========= ProcessorImageFeature test ===========" << std::endl << std::endl;

    // Sensor or sensor recording
    vision_utils::SensorCameraPtr sen_ptr = vision_utils::askUserSource(argc, argv);
    if (sen_ptr==NULL)
    	return 0;

    unsigned int buffer_size = 8;
    vision_utils::Buffer<vision_utils::FramePtr> frame_buff(buffer_size);
    frame_buff.add( vision_utils::setFrame(sen_ptr->getImage(), 0) );

    unsigned int img_width  = frame_buff.back()->getImage().cols;
    unsigned int img_height = frame_buff.back()->getImage().rows;
    std::cout << "Image size: " << img_width << "x" << img_height << std::endl;

    // graphics
    cv::namedWindow("Feature tracker");    // Creates a window for display.
    cv::moveWindow("Feature tracker", 0, 0);
    cv::startWindowThread();

    CaptureImagePtr image_ptr;

    TimeStamp t = 1;

    std::string wolf_root = _WOLF_ROOT_DIR;
    std::cout << "Wolf root: " << wolf_root << std::endl;

    ProblemPtr wolf_problem_ = Problem::create("PO", 3);

    //=====================================================
    // Method 1: Use data generated here for sensor and processor
    //=====================================================

//        // SENSOR
//        Eigen::Vector4s k = {320,240,320,320};
//        SensorCameraPtr camera_ptr = std::make_shared<SensorCamera>(std::make_shared<StateBlock>(Eigen::Vector3s::Zero()),
//                                                  std::make_shared<StateBlock>(Eigen::Vector3s::Zero()),
//                                                  std::make_shared<StateBlock>(k,false),img_width,img_height);
//
//        wolf_problem_->getHardware()->addSensor(camera_ptr);
//
//        // PROCESSOR
//        ProcessorParamsImage tracker_params;
//        tracker_params.matcher.min_normalized_score = 0.75;
//        tracker_params.matcher.similarity_norm = cv::NORM_HAMMING;
//        tracker_params.matcher.roi_width = 30;
//        tracker_params.matcher.roi_height = 30;
//        tracker_params.active_search.grid_width = 12;
//        tracker_params.active_search.grid_height = 8;
//        tracker_params.active_search.separation = 1;
//        tracker_params.max_new_features =0;
//        tracker_params.min_features_for_keyframe = 20;
//
//        DetectorDescriptorParamsOrb orb_params;
//        orb_params.type = DD_ORB;
//
//        DetectorDescriptorParamsBrisk brisk_params;
//        brisk_params.type = DD_BRISK;
//
//        // select the kind of detector-descriptor parameters
//        tracker_params.detector_descriptor_params_ptr = std::make_shared<DetectorDescriptorParamsOrb>(orb_params); // choose ORB
////        tracker_params.detector_descriptor_params_ptr = std::make_shared<DetectorDescriptorParamsBrisk>(brisk_params); // choose BRISK
//
//        std::cout << tracker_params.detector_descriptor_params_ptr->type << std::endl;
//
//        ProcessorTrackerTrifocalTensorPtr prc_image = std::make_shared<ProcessorImageFeature>(tracker_params);
////        camera_ptr->addProcessor(prc_image);
//        std::cout << "sensor & processor created and added to wolf problem" << std::endl;
    //=====================================================

    //=====================================================
    // Method 2: Use factory to create sensor and processor
    //=====================================================

    /* Do this while there aren't extrinsic parameters on the yaml */
    Eigen::Vector7s extrinsic_cam;
    extrinsic_cam[0] = 0; //px
    extrinsic_cam[1] = 0; //py
    extrinsic_cam[2] = 0; //pz
    extrinsic_cam[3] = 0; //qx
    extrinsic_cam[4] = 0; //qy
    extrinsic_cam[5] = 0; //qz
    extrinsic_cam[6] = 1; //qw
    std::cout << "========extrinsic_cam: " << extrinsic_cam.transpose() << std::endl;
    const Eigen::VectorXs extr = extrinsic_cam;
    /* Do this while there aren't extrinsic parameters on the yaml */

    // SENSOR
    // one-liner API
    SensorBasePtr sensor_ptr = wolf_problem_->installSensor("CAMERA", "PinHole", Eigen::VectorXs::Zero(7), wolf_root + "/src/examples/camera_params_ueye_sim.yaml");
    SensorCameraPtr camera_ptr = static_pointer_cast<SensorCamera>(sensor_ptr);
    camera_ptr->setImgWidth(img_width);
    camera_ptr->setImgHeight(img_height);

    // PROCESSOR
    // one-liner API
    ProcessorTrackerFeatureImagePtr prc_img_ptr = std::static_pointer_cast<ProcessorTrackerFeatureImage>( wolf_problem_->installProcessor("IMAGE FEATURE", "ORB", "PinHole", wolf_root + "/src/examples/processor_image_feature.yaml") );
    std::cout << "sensor & processor created and added to wolf problem" << std::endl;
    //=====================================================

//    // Ceres wrapper
//    ceres::Solver::Options ceres_options;
//    ceres_options.minimizer_type = ceres::TRUST_REGION; //ceres::TRUST_REGION;LINE_SEARCH
//    ceres_options.max_line_search_step_contraction = 1e-3;
//    // ceres_options.minimizer_progress_to_stdout = false;
//    // ceres_options.line_search_direction_type = ceres::LBFGS;
//    // ceres_options.max_num_iterations = 100;
//    google::InitGoogleLogging(argv[0]);
//    CeresManager ceres_manager(wolf_problem_, ceres_options);

    for(int f = 0; f<10000; ++f)
    {
        frame_buff.add( vision_utils::setFrame(sen_ptr->getImage(), f) );

    	std::cout << "\n=============== Frame #: " << f << " ===============" << std::endl;

        t.setToNow();
        clock_t t1 = clock();

        // Preferred method with factory objects:
        image_ptr = make_shared<CaptureImage>(t, camera_ptr, frame_buff.back()->getImage());

        camera_ptr->process(image_ptr);

        std::cout << "Time: " << ((double) clock() - t1) / CLOCKS_PER_SEC << "s" << std::endl;

        wolf_problem_->print();

        cv::Mat image = frame_buff.back()->getImage().clone();
        prc_img_ptr->drawFeatures(image);
        prc_img_ptr->drawRoi(image,prc_img_ptr->detector_roi_,cv::Scalar(0.0,255.0, 255.0));   //active search roi
        prc_img_ptr->drawRoi(image,prc_img_ptr->tracker_roi_, cv::Scalar(255.0, 0.0, 255.0));  //tracker roi
        prc_img_ptr->drawTarget(image,prc_img_ptr->tracker_target_);
        cv::imshow("Feature tracker", image);
        cv::waitKey(1);

//        if((f%100) == 0)
//        {
//            std::string summary = ceres_manager.solve(2);
//            std::cout << summary << std::endl;
//
//            std::cout << "Last key frame pose: "
//                      << wolf_problem_->getLastKeyFrame()->getP()->getState().transpose() << std::endl;
//            std::cout << "Last key frame orientation: "
//                      << wolf_problem_->getLastKeyFrame()->getO()->getState().transpose() << std::endl;
//
//            cv::waitKey(0);
//        }
    }
}

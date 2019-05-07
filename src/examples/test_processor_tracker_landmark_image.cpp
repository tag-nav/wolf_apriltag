//std
#include <iostream>

#include "core/processor/processor_tracker_landmark_image.h"

//Wolf
#include "core/common/wolf.h"
#include "core/problem/problem.h"
#include "core/state_block/state_block.h"
#include "core/processor/processor_odom_3D.h"
#include "core/sensor/sensor_odom_3D.h"
#include "core/sensor/sensor_camera.h"
#include "core/capture/capture_image.h"
#include "core/capture/capture_pose.h"
#include "core/ceres_wrapper/ceres_manager.h"

// Vision utils includes
#include <vision_utils.h>
#include <sensors.h>
#include <common_class/buffer.h>
#include <common_class/frame.h>

using Eigen::Vector3s;
using Eigen::Vector4s;
using Eigen::Vector6s;
using Eigen::Vector7s;

using namespace wolf;

void cleanupMap(const ProblemPtr& _problem, const TimeStamp& _t, Scalar _dt_max,
                                      SizeEigen _min_factors)
{
    std::list<LandmarkBasePtr> lmks_to_remove;
    for (auto lmk : _problem->getMap()->getLandmarkList())
    {
        TimeStamp t0 = std::static_pointer_cast<LandmarkAHP>(lmk)->getAnchorFrame()->getTimeStamp();
        if (_t - t0 > _dt_max)
            if (lmk->getConstrainedByList().size() <= _min_factors)
                lmks_to_remove.push_back(lmk);
    }

    for (auto lmk : lmks_to_remove)
    {
        WOLF_DEBUG("Clean up L" , lmk->id() );
        lmk->remove();
    }
}

Eigen::MatrixXs computeDataCovariance(const VectorXs& _data)
{
    Scalar k = 0.5;
    Scalar dist = _data.head<3>().norm();
    if ( dist == 0 ) dist = 1.0;
    WOLF_DEBUG("dist: ", dist, "; sigma: ", sqrt(k* (dist + 0.1)) );
    return k * (dist + 0.1) * Matrix6s::Identity();
}

int main(int argc, char** argv)
{
    std::cout << std::endl << "==================== processor image landmark test ======================" << std::endl;

    // Sensor or sensor recording
    vision_utils::SensorCameraPtr sen_ptr = vision_utils::askUserSource(argc, argv);
    if (sen_ptr==NULL)
        return 0;

    unsigned int buffer_size = 10;
    vision_utils::Buffer<vision_utils::FramePtr> frame_buff(buffer_size);
    frame_buff.add( vision_utils::setFrame(sen_ptr->getImage(), 0) );

    unsigned int img_width  = frame_buff.back()->getImage().cols;
    unsigned int img_height = frame_buff.back()->getImage().rows;
    std::cout << "Image size: " << img_width << "x" << img_height << std::endl;

    //=====================================================
    // Environment variable for configuration files
    std::string wolf_root = _WOLF_ROOT_DIR;
    std::cout << wolf_root << std::endl;
    //=====================================================

    //=====================================================
    // Wolf problem
    ProblemPtr problem = Problem::create("PO", 3);

    // ODOM SENSOR AND PROCESSOR
    SensorBasePtr sensor_base        = problem->installSensor("ODOM 3D", "odom", (Vector7s()<<0,0,0,0,0,0,1).finished(), wolf_root + "/src/examples/sensor_odom_3D.yaml");
    SensorOdom3DPtr sensor_odom      = std::static_pointer_cast<SensorOdom3D>(sensor_base);
    ProcessorBasePtr prcocessor_base = problem->installProcessor("ODOM 3D", "odometry integrator", "odom",               wolf_root + "/src/examples/processor_odom_3D.yaml");

    // CAMERA SENSOR AND PROCESSOR
    sensor_base            = problem->installSensor("CAMERA", "PinHole", (Vector7s()<<0,0,0,0,0,0,1).finished(), wolf_root + "/src/examples/camera_params_ueye_sim.yaml");
    SensorCameraPtr camera = std::static_pointer_cast<SensorCamera>(sensor_base);
    camera->setImgWidth(img_width);
    camera->setImgHeight(img_height);
    ProcessorTrackerLandmarkImagePtr prc_img_ptr = std::static_pointer_cast<ProcessorTrackerLandmarkImage>( problem->installProcessor("IMAGE LANDMARK", "ORB", "PinHole", wolf_root + "/src/examples/processor_image_feature.yaml") );
    //=====================================================

    //=====================================================
    // Origin Key Frame is fixed
    TimeStamp t = 0;
    FrameBasePtr origin_frame = problem->emplaceFrame(KEY, (Vector7s()<<1,0,0,0,0,0,1).finished(), t);
    problem->getProcessorMotion()->setOrigin(origin_frame);
    origin_frame->fix();

    std::cout << "t: " << 0 << "  \t\t\t x = ( " << problem->getCurrentState().transpose() << ")" << std::endl;
    std::cout << "--------------------------------------------------------------" << std::endl;
    //=====================================================

    //=====================================================
    // running CAPTURES preallocated
    CaptureImagePtr image;
    Vector6s data(Vector6s::Zero()); // will integrate this data repeatedly
    CaptureMotionPtr cap_odo = std::make_shared<CaptureMotion>("IMAGE", t, sensor_odom, data, 7, 6, nullptr);
    //=====================================================

    //=====================================================
    // Ceres wrapper
    ceres::Solver::Options ceres_options;
    //    ceres_options.minimizer_type = ceres::TRUST_REGION; //ceres::TRUST_REGION;LINE_SEARCH
    //    ceres_options.max_line_search_step_contraction = 1e-3;
    //    ceres_options.minimizer_progress_to_stdout = false;
    //    ceres_options.line_search_direction_type = ceres::LBFGS;
    ceres_options.max_num_iterations = 10;
    google::InitGoogleLogging(argv[0]);

    CeresManager ceres_manager(problem, ceres_options);
    //=====================================================

    //=====================================================
    // graphics
    cv::namedWindow("Feature tracker");    // Creates a window for display.
    cv::moveWindow("Feature tracker", 0, 0);
    cv::startWindowThread();
    //=====================================================

    //=====================================================
    // main loop
    unsigned int number_of_KFs = 0;
    Scalar dt = 0.04;

    for(int frame_count = 0; frame_count<10000; ++frame_count)
    {
        t += dt;

        // Image ---------------------------------------------

        frame_buff.add( vision_utils::setFrame(sen_ptr->getImage(), frame_count) );

        // Preferred method with factory objects:
        image = std::make_shared<CaptureImage>(t, camera, frame_buff.back()->getImage());

        WOLF_DEBUG(__LINE__);

        /* process */
        camera->process(image);

        WOLF_DEBUG(__LINE__);

        // Odometry --------------------------------------------

        cap_odo->setTimeStamp(t);

        // previous state
        FrameBasePtr prev_key_fr_ptr = problem->getLastKeyFrame();
//        Eigen::Vector7s x_prev = problem->getCurrentState();
        Eigen::Vector7s x_prev = prev_key_fr_ptr->getState();

        // before the previous state
        FrameBasePtr prev_prev_key_fr_ptr = nullptr;
        Vector7s x_prev_prev;
        Vector7s dx;
        for (auto f_it = problem->getTrajectory()->getFrameList().rbegin(); f_it != problem->getTrajectory()->getFrameList().rend(); f_it++)
            if ((*f_it) == prev_key_fr_ptr)
            {
                f_it++;
                if (f_it != problem->getTrajectory()->getFrameList().rend())
                {
                    prev_prev_key_fr_ptr = (*f_it);
                }
                break;
            }

        // compute delta state, and odometry data
        if (!prev_prev_key_fr_ptr)
        {
            // we have just one state --> odometry data is zero
            data.setZero();
        }
        else
        {
            x_prev_prev = prev_prev_key_fr_ptr->getState();

            // define local variables on top of existing vectors to avoid memory allocation
            Eigen::Vector3s     p_prev_prev(x_prev_prev.data());
            Eigen::Quaternions  q_prev_prev(x_prev_prev.data() + 3);
            Eigen::Vector3s     p_prev(x_prev.data());
            Eigen::Quaternions  q_prev(x_prev.data() + 3);
            Eigen::Vector3s     dp(dx.data());
            Eigen::Quaternions  dq(dx.data() + 3);

            // delta state PQ
            dp = q_prev_prev.conjugate() * (p_prev - p_prev_prev);
            dq = q_prev_prev.conjugate() * q_prev;

            // odometry data
            data.head<3>() = dp;
            data.tail<3>() = q2v(dq);
        }

        Matrix6s data_cov = computeDataCovariance(data);

        cap_odo->setData(data);
        cap_odo->setDataCovariance(data_cov);

        sensor_odom->process(cap_odo);

//        problem->print(2,1,0,0);

//        std::cout << "prev prev ts: " << t_prev_prev.get() << "; x: " << x_prev_prev.transpose() << std::endl;
//        std::cout << "prev      ts: " << t_prev.get() << "; x: " << x_prev.transpose() << std::endl;
//        std::cout << "current   ts: " << t.get() << std::endl;
//        std::cout << "          dt: " << t_prev - t_prev_prev << "; dx: " << dx.transpose() << std::endl;

        // Cleanup map ---------------------------------------

        cleanupMap(problem, t, 2, 5); // dt, min_ctr

        // Solve -----------------------------------------------
        // solve only when new KFs are added
        if (problem->getTrajectory()->getFrameList().size() > number_of_KFs)
        {
            number_of_KFs = problem->getTrajectory()->getFrameList().size();
            std::string summary = ceres_manager.solve(SolverManager::ReportVerbosity::BRIEF);// 0: nothing, 1: BriefReport, 2: FullReport
            std::cout << summary << std::endl;
        }

        // Finish loop -----------------------------------------
        cv::Mat image_graphics = frame_buff.back()->getImage().clone();
        prc_img_ptr->drawTrackerRoi(image_graphics, cv::Scalar(255.0, 0.0, 255.0)); //tracker roi
        prc_img_ptr->drawRoi(image_graphics, prc_img_ptr->detector_roi_, cv::Scalar(0.0,255.0, 255.0)); //active search roi
        prc_img_ptr->drawLandmarks(image_graphics);
        prc_img_ptr->drawFeaturesFromLandmarks(image_graphics);
        cv::imshow("Feature tracker", image_graphics);
        cv::waitKey(1);

        std::cout << "=================================================================================================" << std::endl;
    }

    // problem->print(2);
    problem.reset();

    return 0;
}


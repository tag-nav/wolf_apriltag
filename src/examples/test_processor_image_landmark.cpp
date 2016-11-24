/**
 * \file test_processor_tracker_image_landmark.cpp
 *
 *  Created on: Apr 12, 2016
 *      \author: jtarraso
 */

//std
#include <iostream>

//Wolf
#include "wolf.h"
#include "problem.h"
#include "state_block.h"
#include "processor_image_landmark.h"
#include "capture_fix.h"
#include "processor_odom_3D.h"
#include "sensor_odom_3D.h"
#include "ceres_wrapper/ceres_manager.h"

using Eigen::Vector3s;
using Eigen::Vector4s;
using Eigen::Vector6s;
using Eigen::Vector7s;

using namespace wolf;

void cleanupMap(const ProblemPtr& wolf_problem_ptr_, const TimeStamp& t, Scalar dt_max,
                                      Size min_constraints)
{
    std::list<LandmarkBasePtr> lmks_to_remove;
    for (auto lmk : wolf_problem_ptr_->getMapPtr()->getLandmarkList())
    {
        TimeStamp t0 = std::static_pointer_cast<LandmarkAHP>(lmk)->getAnchorFrame()->getTimeStamp();
        if (t - t0 > dt_max)
        {
            unsigned int nbr_ctr = lmk->getConstrainedByList().size();
            if (nbr_ctr <= min_constraints)
            {
                lmks_to_remove.push_back(lmk);
            }
        }
    }
    for (auto lmk : lmks_to_remove)
    {
        std::cout << "clean up L" << lmk->id() << std::endl;
        lmk->remove();
    }
}

int main(int argc, char** argv)
{
    std::cout << std::endl << "==================== processor image landmark test ======================" << std::endl;

    //=====================================================
    // Parse arguments
    cv::VideoCapture capture;
    const char * filename;
    if (argc == 1)
    {
//        filename = "/home/jtarraso/Videos/House_interior.mp4";
//        filename = "/home/jtarraso/Vídeos/gray1.mp4";
        filename = "/home/jtarraso/Escritorio/video_test2/sim_video.mpg";
        capture.open(filename);
    }
    else if (std::string(argv[1]) == "0")
    {
        //camera
        filename = "0";
        capture.open(0);
    }
    else
    {
        filename = argv[1];
        capture.open(filename);
    }
    std::cout << "Input video file: " << filename << std::endl;
    if(!capture.isOpened()) std::cout << "failed" << std::endl; else std::cout << "succeded" << std::endl;
    capture.set(CV_CAP_PROP_POS_MSEC, 6000);
    //=====================================================


    //=====================================================
    // Properties of video sequence
    unsigned int img_width  = capture.get(CV_CAP_PROP_FRAME_WIDTH);
    unsigned int img_height = capture.get(CV_CAP_PROP_FRAME_HEIGHT);
    std::cout << "Image size: " << img_width << "x" << img_height << std::endl;
    unsigned int buffer_size = 10;
    std::vector<cv::Mat> frame(buffer_size);
    //=====================================================


    //=====================================================
    // Environment variable for configuration files
    std::string wolf_root = _WOLF_ROOT_DIR;
    std::cout << wolf_root << std::endl;
    //=====================================================



    //=====================================================
    // Wolf problem
    ProblemPtr wolf_problem_ptr_ = Problem::create(FRM_PO_3D);

    // ODOM SENSOR AND PROCESSOR
    SensorBasePtr sen_ptr = wolf_problem_ptr_->installSensor("ODOM 3D", "odom", (Vector7s()<<0,0,0,0,0,0,1).finished(), wolf_root + "/src/examples/sensor_odom_3D.yaml");
    ProcessorBasePtr prc_ptr = wolf_problem_ptr_->installProcessor("ODOM 3D", "odometry integrator", "odom",            wolf_root + "/src/examples/processor_odom_3D.yaml");
    SensorOdom3D::Ptr sen_odo_ptr = std::static_pointer_cast<SensorOdom3D>(sen_ptr);

    // CAMERA SENSOR AND PROCESSOR
    SensorBasePtr sensor_ptr = wolf_problem_ptr_->installSensor("CAMERA", "PinHole", (Vector7s()<<0,0,0,0,0,0,1).finished(), wolf_root + "/src/examples/camera_params_ueye_sim.yaml");
    SensorCamera::Ptr camera_ptr = std::static_pointer_cast<SensorCamera>(sensor_ptr);
    camera_ptr->setImgWidth(img_width);
    camera_ptr->setImgHeight(img_height);
    wolf_problem_ptr_->installProcessor("IMAGE LANDMARK", "ORB", "PinHole", wolf_root + "/src/examples/processor_image_ORB.yaml");

    //=====================================================


    //=====================================================
    // Origin Key Frame
    TimeStamp t = 0;
    FrameBasePtr origin_frame = wolf_problem_ptr_->emplaceFrame(KEY_FRAME, (Vector7s()<<1,0,0,0,0,0,1).finished(), t);
    wolf_problem_ptr_->getProcessorMotionPtr()->setOrigin(origin_frame);
    origin_frame->fix();

    std::cout << "t: " << 0 << "  \t\t\t x = ( " << wolf_problem_ptr_->getCurrentState().transpose() << ")" << std::endl;
    std::cout << "--------------------------------------------------------------" << std::endl;
    //=====================================================


    //=====================================================
    // running CAPTURES preallocated
    CaptureImagePtr image_ptr;
    Vector6s data(Vector6s::Zero()); // will integrate this data repeatedly
    CaptureMotionPtr cap_odo = std::make_shared<CaptureMotion>(t, sen_odo_ptr, data);
    //=====================================================



    //=====================================================
    // Ceres wrapper
    ceres::Solver::Options ceres_options;
    ceres_options.minimizer_type = ceres::TRUST_REGION; //ceres::TRUST_REGION;LINE_SEARCH
    ceres_options.max_line_search_step_contraction = 1e-3;
    //    ceres_options.minimizer_progress_to_stdout = false;
    //    ceres_options.line_search_direction_type = ceres::LBFGS;
    //    ceres_options.max_num_iterations = 100;
    google::InitGoogleLogging(argv[0]);

    CeresManager ceres_manager(wolf_problem_ptr_, ceres_options);
    //=====================================================


    //=====================================================
    // graphics
    cv::namedWindow("Feature tracker");    // Creates a window for display.
    cv::moveWindow("Feature tracker", 0, 0);
    //=====================================================


    //=====================================================
    // main loop
    unsigned int f  = 1;
    capture >> frame[f % buffer_size];

    Scalar dt = 0.04;

    while(!(frame[f % buffer_size].empty()))
    {

        t += dt;

        // Image ---------------------------------------------

        // Preferred method with factory objects:
        image_ptr = std::make_shared<CaptureImage>(t, camera_ptr, frame[f % buffer_size]);

        /* process */
        camera_ptr->process(image_ptr);



        // Odometry --------------------------------------------

        cap_odo->setTimeStamp(t);

        // previous state and TS
        TimeStamp t_prev_prev;
        Vector7s x_prev_prev;
        Eigen::VectorXs x_prev(7);
        TimeStamp t_prev;
        Vector7s dx;
        wolf_problem_ptr_->getCurrentState(x_prev, t_prev);

        // before the previous state
        FrameBasePtr prev_key_fr_ptr = wolf_problem_ptr_->getLastKeyFramePtr();
        FrameBasePtr prev_prev_key_fr_ptr = nullptr;
        for (auto f_it = wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().rbegin(); f_it != wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().rend(); f_it++)
            if ((*f_it) == prev_key_fr_ptr)
            {
                f_it++;
                if (f_it != wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().rend())
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
            t_prev_prev = prev_prev_key_fr_ptr->getTimeStamp();
            x_prev_prev = prev_prev_key_fr_ptr->getState();

            // some maps to avoid local variables
            Eigen::Map<Eigen::Vector3s>     p_prev_prev(x_prev_prev.data());
            Eigen::Map<Eigen::Quaternions>  q_prev_prev(x_prev_prev.data() + 3);
            Eigen::Map<Eigen::Vector3s>     p_prev(x_prev.data());
            Eigen::Map<Eigen::Quaternions>  q_prev(x_prev.data() + 3);
            Eigen::Map<Eigen::Vector3s>     dp(dx.data());
            Eigen::Map<Eigen::Quaternions>  dq(dx.data() + 3);

            // delta state PQ
//            Eigen::Vector3s dp = q_prev_prev.conjugate() * (p_prev - p_prev_prev);
//            Eigen::Quaternions dq = q_prev_prev.conjugate() * q_prev;
//
//            dx.head<3>() = dp;
//            dx.tail<4>() = dq.coeffs();

            dp = q_prev_prev.conjugate() * (p_prev - p_prev_prev);
            dq = q_prev_prev.conjugate() * q_prev;

            // odometry data
            data.head<3>() = dp;
            data.tail<3>() = q2v(dq);
        }


        cap_odo->setData(data);

        sen_odo_ptr->process(cap_odo);

//        wolf_problem_ptr_->print(2,1,0,0);

//        std::cout << "prev prev ts: " << t_prev_prev.get() << "; x: " << x_prev_prev.transpose() << std::endl;
//        std::cout << "prev      ts: " << t_prev.get() << "; x: " << x_prev.transpose() << std::endl;
//        std::cout << "current   ts: " << t.get() << std::endl;
//        std::cout << "          dt: " << t_prev - t_prev_prev << "; dx: " << dx.transpose() << std::endl;


        // Cleanup map ---------------------------------------

        cleanupMap(wolf_problem_ptr_, t, 2, 5); // dt, min_ctr


        // Solve -----------------------------------------------

        ceres::Solver::Summary summary = ceres_manager.solve();
        std::cout << summary.BriefReport() << std::endl;



        // Finish loop -----------------------------------------

        cv::waitKey(10);

        std::cout << "=================================================================================================" << std::endl;

        f++;
        capture >> frame[f % buffer_size];
    }

    // wolf_problem_ptr_->print(2);
    wolf_problem_ptr_.reset();

    return 0;
}


//std includes
#include "core/sensor/sensor_GPS_fix.h"
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <memory>
#include <random>
#include <typeinfo>
#include <ctime>
#include <queue>

// Eigen includes
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

//Ceres includes
#include "glog/logging.h"

//Wolf includes
#include "core/problem/problem.h"
#include "core/processor/processor_tracker_landmark_polyline.h"
#include "core/processor/processor_odom_2D.h"
#include "core/sensor/sensor_laser_2D.h"
#include "core/sensor/sensor_odom_2D.h"
#include "core/ceres_wrapper/ceres_manager.h"

// laserscanutils
#include "laser_scan_utils/line_finder_iterative.h"
#include "laser_scan_utils/laser_scan.h"

//C includes for sleep, time and main args
#include "unistd.h"

#include "core/capture/capture_pose.h"
//faramotics includes
#include "faramotics/dynamicSceneRender.h"
#include "faramotics/rangeScan2D.h"
#include "btr-headers/pose3d.h"

namespace wolf {
class FaramoticsRobot
{
    public:

        Cpose3d viewPoint, devicePose, laser1Pose, laser2Pose, estimated_vehicle_pose, estimated_laser_1_pose, estimated_laser_2_pose;
        vector < Cpose3d > devicePoses;
        vector<float> scan1, scan2;
        string modelFileName;
        CrangeScan2D* myScanner;
        CdynamicSceneRender* myRender;
        Eigen::Vector3s ground_truth_pose_;
        Eigen::Vector4s laser_1_pose_, laser_2_pose_;

        FaramoticsRobot(int argc, char** argv, const Eigen::Vector4s& _laser_1_pose, const Eigen::Vector4s& _laser_2_pose) :
            modelFileName("/home/jvallve/iri-lab/faramotics/models/campusNordUPC.obj"),
            laser_1_pose_(_laser_1_pose),
            laser_2_pose_(_laser_2_pose)
        {
            devicePose.setPose(2, 8, 0.2, 0, 0, 0);
            viewPoint.setPose(devicePose);
            viewPoint.moveForward(10);
            viewPoint.rt.setEuler(viewPoint.rt.head() + M_PI / 2, viewPoint.rt.pitch() + 30. * M_PI / 180., viewPoint.rt.roll());
            viewPoint.moveForward(-15);
            //glut initialization
            faramotics::initGLUT(argc, argv);
            //create a viewer for the 3D model and scan points
            myRender = new CdynamicSceneRender(1200, 700, 90 * M_PI / 180, 90 * 700.0 * M_PI / (1200.0 * 180.0), 0.2, 100);
            myRender->loadAssimpModel(modelFileName, true); //with wireframe
            //create scanner and load 3D model
            myScanner = new CrangeScan2D(HOKUYO_UTM30LX_180DEG);  //HOKUYO_UTM30LX_180DEG or LEUZE_RS4
            myScanner->loadAssimpModel(modelFileName);
        }

        //function travel around
        Eigen::Vector3s motionCampus(unsigned int ii, double& displacement_, double& rotation_)
        {
            if (ii <= 120){         displacement_ = 0.1;    rotation_ = 0; }
            else if (ii <= 170) {   displacement_ = 0.2;    rotation_ = 1.8 * M_PI / 180; }
            else if (ii <= 220) {   displacement_ = 0;      rotation_ =-1.8 * M_PI / 180; }
            else if (ii <= 310) {   displacement_ = 0.1;    rotation_ = 0; }
            else if (ii <= 487) {   displacement_ = 0.1;    rotation_ =-1.0 * M_PI / 180; }
            else if (ii <= 600) {   displacement_ = 0.2;    rotation_ = 0; }
            else if (ii <= 700) {   displacement_ = 0.1;    rotation_ =-1.0 * M_PI / 180; }
            else if (ii <= 780) {   displacement_ = 0;      rotation_ =-1.0 * M_PI / 180; }
            else {                  displacement_ = 0.3;    rotation_ = 0; }

            devicePose.moveForward(displacement_);
            devicePose.rt.setEuler(devicePose.rt.head() + rotation_, devicePose.rt.pitch(), devicePose.rt.roll());

            // laser 1
            laser1Pose.setPose(devicePose);
            laser1Pose.moveForward(laser_1_pose_(0));
            // laser 2
            laser2Pose.setPose(devicePose);
            laser2Pose.moveForward(laser_2_pose_(0));
            laser2Pose.rt.setEuler(laser2Pose.rt.head() + laser_2_pose_(3), laser2Pose.rt.pitch(), laser2Pose.rt.roll());

            devicePoses.push_back(devicePose);

            ground_truth_pose_ << devicePose.pt(0), devicePose.pt(1), devicePose.rt.head();
            return ground_truth_pose_;
        }

        ~FaramoticsRobot()
        {
            std::cout << "deleting render and scanner.." << std::endl;
            delete myRender;
            delete myScanner;
            std::cout << "deleted!" << std::endl;
        }

        //compute scans
        vector<float> computeScan(const int scan_id)
        {
            if (scan_id == 1)
            {
                scan1.clear();
                myScanner->computeScan(laser1Pose, scan1);
                return scan1;
            }
            else
            {
                scan2.clear();
                myScanner->computeScan(laser2Pose, scan2);
                return scan2;
            }
        }

        void render(const FeatureBasePtrList& feature_list, int laser, const LandmarkBasePtrList& landmark_list, const Eigen::Vector3s& estimated_pose)
        {
            // detected corners
            std::cout << "   drawCorners: " << feature_list.size() << std::endl;
            std::vector<double> corner_vector;
            corner_vector.reserve(2*feature_list.size());
            for (auto corner : feature_list)
            {
                std::cout << "       corner " << corner->id() << std::endl;
                corner_vector.push_back(corner->getMeasurement(0));
                corner_vector.push_back(corner->getMeasurement(1));
            }
            myRender->drawCorners(laser == 1 ? laser1Pose : laser2Pose, corner_vector);

            // landmarks
            std::cout << "   drawLandmarks: " << landmark_list.size() << std::endl;
            std::vector<double> landmark_vector;
            landmark_vector.reserve(3*landmark_list.size());
            for (auto landmark : landmark_list)
            {
                Scalar* position_ptr = landmark->getP()->get();
                landmark_vector.push_back(*position_ptr); //x
                landmark_vector.push_back(*(position_ptr + 1)); //y
                landmark_vector.push_back(0.2); //z
            }
            myRender->drawLandmarks(landmark_vector);

            // draw localization and sensors
            estimated_vehicle_pose.setPose(estimated_pose(0), estimated_pose(1), 0.2, estimated_pose(2), 0, 0);
            estimated_laser_1_pose.setPose(estimated_vehicle_pose);
            estimated_laser_1_pose.moveForward(laser_1_pose_(0));
            estimated_laser_2_pose.setPose(estimated_vehicle_pose);
            estimated_laser_2_pose.moveForward(laser_2_pose_(0));
            estimated_laser_2_pose.rt.setEuler(estimated_laser_2_pose.rt.head() + laser_2_pose_(3), estimated_laser_2_pose.rt.pitch(), estimated_laser_2_pose.rt.roll());
            myRender->drawPoseAxisVector( { estimated_vehicle_pose, estimated_laser_1_pose, estimated_laser_2_pose });

            //Set view point and render the scene
            //locate visualization view point, somewhere behind the device
    //      viewPoint.setPose(devicePose);
    //      viewPoint.rt.setEuler( viewPoint.rt.head(), viewPoint.rt.pitch()+20.*M_PI/180., viewPoint.rt.roll() );
    //      viewPoint.moveForward(-5);
            myRender->setViewPoint(viewPoint);
            myRender->drawPoseAxis(devicePose);
            myRender->drawScan(laser == 1 ? laser1Pose : laser2Pose, laser == 1 ? scan1 : scan2, 180. * M_PI / 180., 90. * M_PI / 180.); //draw scan
            myRender->render();
        }
};
}

int main(int argc, char** argv)
{
    using namespace wolf;

    std::cout << "\n============================================================\n";
    std::cout << "========== 2D Robot with odometry and 2 LIDARs =============\n";

    // USER INPUT ============================================================================================
    if (argc != 2 || atoi(argv[1]) < 1 )
    {
        std::cout << "Please call me with: [./test_ceres_manager NI PRINT], where:" << std::endl;
        std::cout << "     - NI is the number of iterations (NI > 0)" << std::endl;
        std::cout << "EXIT due to bad user input" << std::endl << std::endl;
        return -1;
    }

    unsigned int n_execution = (unsigned int) atoi(argv[1]); //number of iterations of the whole execution

    // INITIALIZATION ============================================================================================
    //init random generators
    Scalar odom_std_factor = 0.5;
    Scalar gps_std = 1;
    std::default_random_engine generator(1);
    std::normal_distribution<Scalar> distribution_odom(0.0, odom_std_factor); //odometry noise
    std::normal_distribution<Scalar> distribution_gps(0.0, gps_std); //GPS noise

    //variables
    Eigen::Vector2s odom_data;
    Eigen::Vector2s gps_fix_reading;
    Eigen::VectorXs ground_truth(n_execution * 3); //all true poses
    Eigen::Vector3s ground_truth_pose; //last true pose
    Eigen::VectorXs odom_trajectory(n_execution * 3); //open loop trajectory
    Eigen::VectorXs mean_times = Eigen::VectorXs::Zero(7);
    clock_t t1, t2;
    Scalar dt = 0.05;
    TimeStamp ts(0);

    // Wolf Tree initialization
    Eigen::Vector3s odom_pose = Eigen::Vector3s::Zero();
    Eigen::Vector3s gps_pose = Eigen::Vector3s::Zero();
    Eigen::Vector4s laser_1_pose, laser_2_pose; //xyz + theta
    laser_1_pose << 1.2, 0, 0, 0; //laser 1
    laser_2_pose << -1.2, 0, 0, M_PI; //laser 2

    Problem problem(FRM_PO_2D);
    SensorOdom2D* odom_sensor = new SensorOdom2D(std::make_shared<StateBlock>(odom_pose.head(2), true), std::make_shared<StateBlock>(odom_pose.tail(1), true), odom_std_factor, odom_std_factor);
    SensorGPSFix* gps_sensor = new SensorGPSFix(std::make_shared<StateBlock>(gps_pose.head(2), true), std::make_shared<StateBlock>(gps_pose.tail(1), true), gps_std);
    SensorLaser2D* laser_1_sensor = new SensorLaser2D(std::make_shared<StateBlock>(laser_1_pose.head(2), true), std::make_shared<StateBlock>(laser_1_pose.tail(1), true), laserscanutils::LaserScanParams({M_PI/2,-M_PI/2, -M_PI/720,0.01,0.2,100,0.01,0.01}));
    SensorLaser2D* laser_2_sensor = new SensorLaser2D(std::make_shared<StateBlock>(laser_2_pose.head(2), true), std::make_shared<StateBlock>(laser_2_pose.tail(1), true), laserscanutils::LaserScanParams({M_PI/2,-M_PI/2, -M_PI/720,0.01,0.2,100,0.01,0.01}));

    wolf::ProcessorParamsPolyline laser_processor_params;
    laser_processor_params.line_finder_params = laserscanutils::LineFinderIterativeParams({0.1, 5, 1, 2});
    laser_processor_params.new_features_th = 3;
    laser_processor_params.loop_frames_th = 10;
    laser_processor_params.time_tolerance = 0.1;
    laser_processor_params.position_error_th = 0.5;
    ProcessorTrackerLandmarkPolyline* laser_1_processor = new ProcessorTrackerLandmarkPolyline(laser_processor_params);
    ProcessorTrackerLandmarkPolyline* laser_2_processor = new ProcessorTrackerLandmarkPolyline(laser_processor_params);
    ProcessorOdom2D* odom_processor = new ProcessorOdom2D(1,1,100);
    odom_sensor->addProcessor(odom_processor);
    laser_1_sensor->addProcessor(laser_1_processor);
    laser_2_sensor->addProcessor(laser_2_processor);
    problem.addSensor(odom_sensor);
    problem.addSensor(gps_sensor);
    problem.addSensor(laser_1_sensor);
    problem.addSensor(laser_2_sensor);
    problem.setProcessorMotion(odom_processor);

    CaptureMotion* odom_capture = new CaptureMotion(ts,odom_sensor, odom_data, Eigen::Matrix2s::Identity() * odom_std_factor * odom_std_factor, nullptr);

    // Simulated robot
    FaramoticsRobot robot(argc, argv, laser_1_pose, laser_2_pose);

    // Initial pose
    ground_truth_pose << 2, 8, 0;
    ground_truth.head(3) = ground_truth_pose;
    odom_trajectory.head(3) = ground_truth_pose;

    // Origin Key Frame
    FrameBasePtr origin_frame = problem.createFrame(KEY_FRAME, ground_truth_pose, ts);

    // Prior covariance
    CapturePose* initial_covariance = new CapturePose(ts, gps_sensor, ground_truth_pose, Eigen::Matrix3s::Identity() * 0.1);
    origin_frame->addCapture(initial_covariance);
    initial_covariance->process();

    odom_processor->setOrigin(origin_frame);

    // Ceres wrapper
    ceres::Solver::Options ceres_options;
    ceres_options.minimizer_type = ceres::TRUST_REGION; //ceres::TRUST_REGION;LINE_SEARCH
    ceres_options.max_line_search_step_contraction = 1e-3;
    //    ceres_options.minimizer_progress_to_stdout = false;
    //    ceres_options.line_search_direction_type = ceres::LBFGS;
    //    ceres_options.max_num_iterations = 100;
    google::InitGoogleLogging(argv[0]);

    CeresManager ceres_manager(&problem, ceres_options);
    std::ofstream log_file, landmark_file;  //output file

    //std::cout << "START TRAJECTORY..." << std::endl;
    // START TRAJECTORY ============================================================================================
    for (unsigned int step = 1; step < n_execution; step++)
    {
        // timestamp
        ts = TimeStamp(step*dt);

        //get init time
        t2 = clock();

        // ROBOT MOVEMENT ---------------------------
        //std::cout << "ROBOT MOVEMENT..." << std::endl;
        // moves the device position
        t1 = clock();
        ground_truth_pose = robot.motionCampus(step, odom_data(0), odom_data(1));
        ground_truth.segment(step * 3, 3) = ground_truth_pose;

        // ODOMETRY DATA -------------------------------------
        // noisy odometry
        odom_data(0) += distribution_odom(generator) * (odom_data(0) == 0 ? 1e-6 : odom_data(0));
        odom_data(1) += distribution_odom(generator) * (odom_data(1) == 0 ? 1e-6 : odom_data(1));
        // process odometry
        odom_capture->setTimeStamp(TimeStamp(ts));
        odom_capture->setData(odom_data);
        odom_processor->process(odom_capture);
        // odometry integration
        odom_trajectory.segment(step * 3, 3) = problem.getCurrentState();

        // LIDAR DATA ---------------------------
        if (step % 3 == 0)
        {
            std::cout << "--PROCESS LIDAR 1 DATA..." << laser_1_sensor->id() << std::endl;
            laser_1_processor->process(new CaptureLaser2D(ts, laser_1_sensor, robot.computeScan(1)));
            std::cout << "--PROCESS LIDAR 2 DATA..." << laser_2_sensor->id() << std::endl;
            laser_2_processor->process(new CaptureLaser2D(ts, laser_2_sensor, robot.computeScan(2)));
        }

        // GPS DATA ---------------------------
        if (step % 5 == 0)
        {
            // compute GPS
            gps_fix_reading  = ground_truth_pose.head<2>();
            gps_fix_reading(0) += distribution_gps(generator);
            gps_fix_reading(1) += distribution_gps(generator);
            // process data
            //(new CaptureGPSFix(ts, &gps_sensor, gps_fix_reading, gps_std * Eigen::MatrixXs::Identity(3,3)));
        }
        mean_times(0) += ((double) clock() - t1) / CLOCKS_PER_SEC;

        // SOLVE OPTIMIZATION ---------------------------
        std::cout << "SOLVING..." << std::endl;
        t1 = clock();
        std::string summary = ceres_manager.solve(2);// 0: nothing, 1: BriefReport, 2: FullReport
        std::cout << summary << std::endl;
        mean_times(3) += ((double) clock() - t1) / CLOCKS_PER_SEC;

        // COMPUTE COVARIANCES ---------------------------
        std::cout << "COMPUTING COVARIANCES..." << std::endl;
        t1 = clock();
        //ceres_manager.computeCovariances();
        mean_times(4) += ((double) clock() - t1) / CLOCKS_PER_SEC;

        // DRAWING STUFF ---------------------------
        std::cout << "RENDERING..." << std::endl;
        t1 = clock();
//        if (step % 3 == 0)
//            robot.render(laser_1_processor->getLast() == nullptr ? FeatureBasePtrList({}) : *laser_1_processor->getLast()->getFeatureList(), 1, *problem.getMap()->getLandmarkList(), problem.getCurrentState());
            //robot.render(laser_2_processor->getLast() == nullptr ? FeatureBasePtrList({}) : *laser_2_processor->getLast()->getFeatureList(), 2, *problem.getMap()->getLandmarkList(), problem.getCurrentState());
        mean_times(5) += ((double) clock() - t1) / CLOCKS_PER_SEC;

        // TIME MANAGEMENT ---------------------------
        double total_t = ((double) clock() - t2) / CLOCKS_PER_SEC;
        mean_times(6) += total_t;
        if (total_t < 0.5)
            usleep(500000 - 1e6 * total_t);

//		std::cout << "\nTree after step..." << std::endl;
    }

    // DISPLAY RESULTS ============================================================================================
    mean_times /= n_execution;
    std::cout << "\nSIMULATION AVERAGE LOOP DURATION [s]" << std::endl;
    std::cout << "  data generation:    " << mean_times(0) << std::endl;
    std::cout << "  wolf managing:      " << mean_times(1) << std::endl;
    std::cout << "  ceres managing:     " << mean_times(2) << std::endl;
    std::cout << "  ceres optimization: " << mean_times(3) << std::endl;
    std::cout << "  ceres covariance:   " << mean_times(4) << std::endl;
    std::cout << "  results drawing:    " << mean_times(5) << std::endl;
    std::cout << "  loop time:          " << mean_times(6) << std::endl;

    std::cout << "\nTree before deleting..." << std::endl;

        std::cout << "Press any key for ending... " << std::endl << std::endl;
    std::getchar();

    std::cout << " ========= END ===========" << std::endl << std::endl;

    //exit
    return 0;
}

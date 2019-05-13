//std includes
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
#include "wolf_manager.h"
#include "core/sensor/sensor_laser_2D.h"
#include "core/ceres_wrapper/ceres_manager.h"

//C includes for sleep, time and main args
#include "unistd.h"

//faramotics includes
#include "faramotics/dynamicSceneRender.h"
#include "faramotics/rangeScan2D.h"
#include "btr-headers/pose3d.h"

//laser_scan_utils
#include "iri-algorithms/laser_scan_utils/corner_detector.h"
#include "iri-algorithms/laser_scan_utils/entities.h"

namespace wolf {
//function travel around
void motionCampus(unsigned int ii, Cpose3d & pose, double& displacement_, double& rotation_)
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

    pose.moveForward(displacement_);
    pose.rt.setEuler(pose.rt.head() + rotation_, pose.rt.pitch(), pose.rt.roll());
}

int main(int argc, char** argv)
{
    using namespace wolf;

    std::cout << "\n ========= 2D Robot with odometry and 2 LIDARs ===========\n";

    // USER INPUT ============================================================================================
    if (argc != 3 || atoi(argv[1]) < 1 || atoi(argv[2]) < 1 )
    {
        std::cout << "Please call me with: [./test_ceres_manager NI PRINT], where:" << std::endl;
        std::cout << "     - NI is the number of iterations (NI > 0)" << std::endl;
        std::cout << "     - WS is the window size (WS > 0)" << std::endl;
        std::cout << "EXIT due to bad user input" << std::endl << std::endl;
        return -1;
    }

    unsigned int n_execution = (unsigned int) atoi(argv[1]); //number of iterations of the whole execution
    unsigned int window_size = (unsigned int) atoi(argv[2]);

    // INITIALIZATION ============================================================================================
    //init random generators
    Scalar odom_std_factor = 0.5;
    Scalar gps_std = 1;
    std::default_random_engine generator(1);
    std::normal_distribution<Scalar> distribution_odom(0.0, odom_std_factor); //odometry noise
    std::normal_distribution<Scalar> distribution_gps(0.0, gps_std); //GPS noise

    //init google log
    //google::InitGoogleLogging(argv[0]);

    // Faramotics stuff
    Cpose3d viewPoint, devicePose, laser1Pose, laser2Pose, estimated_vehicle_pose, estimated_laser_1_pose, estimated_laser_2_pose;
    vector < Cpose3d > devicePoses;
    vector<float> scan1, scan2;
    string modelFileName;

    //model and initial view point
    modelFileName = "/home/jvallve/iri-lab/faramotics/models/campusNordUPC.obj";
    //modelFileName = "/home/acoromin/dev/br/faramotics/models/campusNordUPC.obj";
    //modelFileName = "/home/andreu/dev/faramotics/models/campusNordUPC.obj";
    devicePose.setPose(2, 8, 0.2, 0, 0, 0);
    viewPoint.setPose(devicePose);
    viewPoint.moveForward(10);
    viewPoint.rt.setEuler(viewPoint.rt.head() + M_PI / 2, viewPoint.rt.pitch() + 30. * M_PI / 180., viewPoint.rt.roll());
    viewPoint.moveForward(-15);
    //glut initialization
    faramotics::initGLUT(argc, argv);

    //create a viewer for the 3D model and scan points
    CdynamicSceneRender* myRender = new CdynamicSceneRender(1200, 700, 90 * M_PI / 180, 90 * 700.0 * M_PI / (1200.0 * 180.0), 0.2, 100);
    myRender->loadAssimpModel(modelFileName, true); //with wireframe
    //create scanner and load 3D model
    CrangeScan2D* myScanner = new CrangeScan2D(HOKUYO_UTM30LX_180DEG);	//HOKUYO_UTM30LX_180DEG or LEUZE_RS4
    myScanner->loadAssimpModel(modelFileName);

    //variables
    Eigen::Vector3s odom_reading;
    Eigen::Vector2s gps_fix_reading;
    Eigen::VectorXs pose_odom(3); //current odometry integred pose
    Eigen::VectorXs ground_truth(n_execution * 3); //all true poses
    Eigen::VectorXs odom_trajectory(n_execution * 3); //open loop trajectory
    Eigen::VectorXs mean_times = Eigen::VectorXs::Zero(7);
    clock_t t1, t2;

    // Wolf manager initialization
    Eigen::Vector3s odom_pose = Eigen::Vector3s::Zero();
    Eigen::Vector3s gps_pose = Eigen::Vector3s::Zero();
    Eigen::Vector4s laser_1_pose, laser_2_pose; //xyz + theta
    laser_1_pose << 1.2, 0, 0, 0; //laser 1
    laser_2_pose << -1.2, 0, 0, M_PI; //laser 2
    SensorOdom2D odom_sensor(std::make_shared<StateBlock>(odom_pose.head(2)), std::make_shared<StateBlock>(odom_pose.tail(1)), odom_std_factor, odom_std_factor);
    SensorGPSFix gps_sensor(std::make_shared<StateBlock>(gps_pose.head(2)), std::make_shared<StateBlock>(gps_pose.tail(1)), gps_std);
    SensorLaser2D laser_1_sensor(std::make_shared<StateBlock>(laser_1_pose.head(2)), std::make_shared<StateBlock>(laser_1_pose.tail(1)));
    SensorLaser2D laser_2_sensor(std::make_shared<StateBlock>(laser_2_pose.head(2)), std::make_shared<StateBlock>(laser_2_pose.tail(1)));
    laser_1_sensor.addProcessor(new ProcessorLaser2D());
    laser_2_sensor.addProcessor(new ProcessorLaser2D());

    // Initial pose
    pose_odom << 2, 8, 0;
    ground_truth.head(3) = pose_odom;
    odom_trajectory.head(3) = pose_odom;

    WolfManager* wolf_manager_ceres = new WolfManager(FRM_PO_2D, &odom_sensor, pose_odom, Eigen::Matrix3s::Identity() * 0.01, window_size, 0.3);
    WolfManager* wolf_manager_wolf = new WolfManager(FRM_PO_2D, &odom_sensor, pose_odom, Eigen::Matrix3s::Identity() * 0.01, window_size, 0.3);
    
    // Ceres wrapper
    ceres::Solver::Options ceres_options;
    ceres_options.minimizer_type = ceres::TRUST_REGION; //ceres::TRUST_REGION;LINE_SEARCH
    ceres_options.max_line_search_step_contraction = 1e-3;
    //    ceres_options.minimizer_progress_to_stdout = false;
    //    ceres_options.line_search_direction_type = ceres::LBFGS;
    //    ceres_options.max_num_iterations = 100;
    CeresManager* ceres_manager_ceres = new CeresManager(wolf_manager_ceres->getProblem(), false);
    CeresManager* ceres_manager_wolf = new CeresManager(wolf_manager_wolf->getProblem(), true);
    std::ofstream log_file, landmark_file;  //output file

    //std::cout << "START TRAJECTORY..." << std::endl;
    // START TRAJECTORY ============================================================================================
    for (unsigned int step = 1; step < n_execution; step++)
    {
        //get init time
        t2 = clock();

        // ROBOT MOVEMENT ---------------------------
        //std::cout << "ROBOT MOVEMENT..." << std::endl;
        // moves the device position
        t1 = clock();
        motionCampus(step, devicePose, odom_reading(0), odom_reading(2));
        odom_reading(1) = 0;
        devicePoses.push_back(devicePose);

        // SENSOR DATA ---------------------------
        //std::cout << "SENSOR DATA..." << std::endl;
        // store groundtruth
        ground_truth.segment(step * 3, 3) << devicePose.pt(0), devicePose.pt(1), devicePose.rt.head();

        // compute odometry
        odom_reading(0) += distribution_odom(generator) * (odom_reading(0) == 0 ? 1e-6 : odom_reading(0));
        odom_reading(1) += distribution_odom(generator) * 1e-6;
        odom_reading(2) += distribution_odom(generator) * (odom_reading(2) == 0 ? 1e-6 : odom_reading(2));

        // odometry integration
        pose_odom(0) = pose_odom(0) + odom_reading(0) * cos(pose_odom(2)) - odom_reading(1) * sin(pose_odom(2));
        pose_odom(1) = pose_odom(1) + odom_reading(0) * sin(pose_odom(2)) + odom_reading(1) * cos(pose_odom(2));
        pose_odom(2) = pose_odom(2) + odom_reading(1);
        odom_trajectory.segment(step * 3, 3) = pose_odom;

        // compute GPS
        gps_fix_reading << devicePose.pt(0), devicePose.pt(1);
        gps_fix_reading(0) += distribution_gps(generator);
        gps_fix_reading(1) += distribution_gps(generator);

        //compute scans
        scan1.clear();
        scan2.clear();
        // scan 1
        laser1Pose.setPose(devicePose);
        laser1Pose.moveForward(laser_1_pose(0));
        myScanner->computeScan(laser1Pose, scan1);
        // scan 2
        laser2Pose.setPose(devicePose);
        laser2Pose.moveForward(laser_2_pose(0));
        laser2Pose.rt.setEuler(laser2Pose.rt.head() + M_PI, laser2Pose.rt.pitch(), laser2Pose.rt.roll());
        myScanner->computeScan(laser2Pose, scan2);

        mean_times(0) += ((double) clock() - t1) / CLOCKS_PER_SEC;

        // ADD CAPTURES ---------------------------
        std::cout << "ADD CAPTURES..." << std::endl;
        t1 = clock();
        // adding new sensor captures
        wolf_manager_ceres->addCapture(new CaptureOdom2D(TimeStamp(),TimeStamp(), &odom_sensor, odom_reading));		//, odom_std_factor * Eigen::MatrixXs::Identity(2,2)));
        wolf_manager_ceres->addCapture(new CaptureGPSFix(TimeStamp(), &gps_sensor, gps_fix_reading, gps_std * Eigen::MatrixXs::Identity(3,3)));
        wolf_manager_ceres->addCapture(new CaptureLaser2D(TimeStamp(), &laser_1_sensor, scan1));
        wolf_manager_ceres->addCapture(new CaptureLaser2D(TimeStamp(), &laser_2_sensor, scan2));
        wolf_manager_wolf->addCapture(new CaptureOdom2D(TimeStamp(),TimeStamp(), &odom_sensor, odom_reading));       //, odom_std_factor * Eigen::MatrixXs::Identity(2,2)));
        wolf_manager_wolf->addCapture(new CaptureGPSFix(TimeStamp(), &gps_sensor, gps_fix_reading, gps_std * Eigen::MatrixXs::Identity(3,3)));
        wolf_manager_wolf->addCapture(new CaptureLaser2D(TimeStamp(), &laser_1_sensor, scan1));
        wolf_manager_wolf->addCapture(new CaptureLaser2D(TimeStamp(), &laser_2_sensor, scan2));

        // updating problem
        wolf_manager_ceres->update();
        wolf_manager_wolf->update();
        mean_times(1) += ((double) clock() - t1) / CLOCKS_PER_SEC;

        // UPDATING CERES ---------------------------
        std::cout << "UPDATING CERES..." << std::endl;
        t1 = clock();
        // update state units and factors in ceres
        ceres_manager_ceres->update();
        ceres_manager_wolf->update();
        mean_times(2) += ((double) clock() - t1) / CLOCKS_PER_SEC;

        // SOLVE OPTIMIZATION ---------------------------
        std::cout << "SOLVING..." << std::endl;
        t1 = clock();
        ceres::Solver::Summary summary_ceres = ceres_manager_ceres->solve(ceres_options);
        ceres::Solver::Summary summary_wolf = ceres_manager_wolf->solve(ceres_options);
        std::cout << "CERES AUTO DIFF" << std::endl;
        std::cout << "Jacobian evaluation: " << summary_ceres.jacobian_evaluation_time_in_seconds << std::endl;
        std::cout << "Total time: " << summary_ceres.total_time_in_seconds << std::endl;
        std::cout << "WOLF AUTO DIFF" << std::endl;
        std::cout << "Jacobian evaluation: " << summary_wolf.jacobian_evaluation_time_in_seconds << std::endl;
        std::cout << "Total time: " << summary_wolf.total_time_in_seconds << std::endl;
        mean_times(3) += ((double) clock() - t1) / CLOCKS_PER_SEC;

        std::cout << "CERES AUTO DIFF solution:" << std::endl;
        std::cout << wolf_manager_ceres->getVehiclePose().transpose() << std::endl;
        std::cout << "WOLF AUTO DIFF solution:" << std::endl;
        std::cout << wolf_manager_wolf->getVehiclePose().transpose() << std::endl;

        // COMPUTE COVARIANCES ---------------------------
        std::cout << "COMPUTING COVARIANCES..." << std::endl;
        t1 = clock();
        ceres_manager_ceres->computeCovariances(ALL_MARGINALS);
        ceres_manager_wolf->computeCovariances(ALL_MARGINALS);
        Eigen::MatrixXs marginal_ceres(3,3), marginal_wolf(3,3);
        wolf_manager_ceres->getProblem()->getCovarianceBlock(wolf_manager_ceres->getProblem()->getTrajectory()->getLastFrame()->getP(),
                                                                wolf_manager_ceres->getProblem()->getTrajectory()->getLastFrame()->getP(),
                                                                marginal_ceres, 0, 0);
        wolf_manager_ceres->getProblem()->getCovarianceBlock(wolf_manager_ceres->getProblem()->getTrajectory()->getLastFrame()->getP(),
                                                                wolf_manager_ceres->getProblem()->getTrajectory()->getLastFrame()->getO(),
                                                                marginal_ceres, 0, 2);
        wolf_manager_ceres->getProblem()->getCovarianceBlock(wolf_manager_ceres->getProblem()->getTrajectory()->getLastFrame()->getO(),
                                                                wolf_manager_ceres->getProblem()->getTrajectory()->getLastFrame()->getO(),
                                                                marginal_ceres, 2, 2);
        wolf_manager_wolf->getProblem()->getCovarianceBlock(wolf_manager_wolf->getProblem()->getTrajectory()->getLastFrame()->getP(),
                                                               wolf_manager_wolf->getProblem()->getTrajectory()->getLastFrame()->getP(),
                                                               marginal_wolf, 0, 0);
        wolf_manager_wolf->getProblem()->getCovarianceBlock(wolf_manager_wolf->getProblem()->getTrajectory()->getLastFrame()->getP(),
                                                               wolf_manager_wolf->getProblem()->getTrajectory()->getLastFrame()->getO(),
                                                               marginal_wolf, 0, 2);
        wolf_manager_wolf->getProblem()->getCovarianceBlock(wolf_manager_wolf->getProblem()->getTrajectory()->getLastFrame()->getO(),
                                                               wolf_manager_wolf->getProblem()->getTrajectory()->getLastFrame()->getO(),
                                                               marginal_wolf, 2, 2);
        std::cout << "CERES AUTO DIFF covariance:" << std::endl;
        std::cout << marginal_ceres << std::endl;
        std::cout << "WOLF AUTO DIFF covariance:" << std::endl;
        std::cout << marginal_wolf << std::endl;
        mean_times(4) += ((double) clock() - t1) / CLOCKS_PER_SEC;

        // DRAWING STUFF ---------------------------
        t1 = clock();
        // draw detected corners
//        std::list < laserscanutils::Corner > corner_list;
//        std::vector<double> corner_vector;
//        CaptureLaser2D last_scan(TimeStamp(), &laser_1_sensor, scan1);
//        last_scan.extractCorners(corner_list);
//        for (std::list<laserscanutils::Corner>::iterator corner_it = corner_list.begin(); corner_it != corner_list.end(); corner_it++)
//        {
//            corner_vector.push_back(corner_it->pt_(0));
//            corner_vector.push_back(corner_it->pt_(1));
//        }
//        myRender->drawCorners(laser1Pose, corner_vector);

//        // draw landmarks
//        std::vector<double> landmark_vector;
//        for (auto landmark_it = wolf_manager->getProblem()->getMap()->getLandmarkList().begin(); landmark_it != wolf_manager->getProblem()->getMap()->getLandmarkList().end(); landmark_it++)
//        {
//            Scalar* position_ptr = (*landmark_it)->getP()->get();
//            landmark_vector.push_back(*position_ptr); //x
//            landmark_vector.push_back(*(position_ptr + 1)); //y
//            landmark_vector.push_back(0.2); //z
//        }
//        myRender->drawLandmarks(landmark_vector);
//
//        // draw localization and sensors
//        estimated_vehicle_pose.setPose(wolf_manager->getVehiclePose()(0), wolf_manager->getVehiclePose()(1), 0.2, wolf_manager->getVehiclePose()(2), 0, 0);
//        estimated_laser_1_pose.setPose(estimated_vehicle_pose);
//        estimated_laser_1_pose.moveForward(laser_1_pose(0));
//        estimated_laser_2_pose.setPose(estimated_vehicle_pose);
//        estimated_laser_2_pose.moveForward(laser_2_pose(0));
//        estimated_laser_2_pose.rt.setEuler(estimated_laser_2_pose.rt.head() + M_PI, estimated_laser_2_pose.rt.pitch(), estimated_laser_2_pose.rt.roll());
//        myRender->drawPoseAxisVector( { estimated_vehicle_pose, estimated_laser_1_pose, estimated_laser_2_pose });
//
//        //Set view point and render the scene
//        //locate visualization view point, somewhere behind the device
////		viewPoint.setPose(devicePose);
////		viewPoint.rt.setEuler( viewPoint.rt.head(), viewPoint.rt.pitch()+20.*M_PI/180., viewPoint.rt.roll() );
////		viewPoint.moveForward(-5);
//        myRender->setViewPoint(viewPoint);
//        myRender->drawPoseAxis(devicePose);
//        myRender->drawScan(laser1Pose, scan1, 180. * M_PI / 180., 90. * M_PI / 180.); //draw scan
//        myRender->render();
//        mean_times(5) += ((double) clock() - t1) / CLOCKS_PER_SEC;

        // TIME MANAGEMENT ---------------------------
        double dt = ((double) clock() - t2) / CLOCKS_PER_SEC;
        mean_times(6) += dt;
        if (dt < 0.1)
            usleep(100000 - 1e6 * dt);

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

//	std::cout << "\nTree before deleting..." << std::endl;

//    // Draw Final result -------------------------
//    std::vector<double> landmark_vector;
//    for (auto landmark_it = wolf_manager->getProblem()->getMap()->getLandmarkList().begin(); landmark_it != wolf_manager->getProblem()->getMap()->getLandmarkList().end(); landmark_it++)
//    {
//        Scalar* position_ptr = (*landmark_it)->getP()->get();
//        landmark_vector.push_back(*position_ptr); //x
//        landmark_vector.push_back(*(position_ptr + 1)); //y
//        landmark_vector.push_back(0.2); //z
//    }
//    myRender->drawLandmarks(landmark_vector);
////	viewPoint.setPose(devicePoses.front());
////	viewPoint.moveForward(10);
////	viewPoint.rt.setEuler( viewPoint.rt.head()+M_PI/4, viewPoint.rt.pitch()+20.*M_PI/180., viewPoint.rt.roll() );
////	viewPoint.moveForward(-10);
//    myRender->setViewPoint(viewPoint);
//    myRender->render();

    // Print Final result in a file -------------------------
    // Vehicle poses
//    int i = 0;
//    Eigen::VectorXs state_poses(n_execution * 3);
//    for (auto frame_it = wolf_manager->getProblem()->getTrajectory()->getFrameList().begin(); frame_it != wolf_manager->getProblem()->getTrajectory()->getFrameList().end(); frame_it++)
//    {
//        state_poses.segment(i, 3) << *(*frame_it)->getP()->get(), *((*frame_it)->getP()->get() + 1), *(*frame_it)->getO()->get();
//        i += 3;
//    }
//
//    // Landmarks
//    i = 0;
//    Eigen::VectorXs landmarks(wolf_manager->getProblem()->getMap()->getLandmarkList().size() * 2);
//    for (auto landmark_it = wolf_manager->getProblem()->getMap()->getLandmarkList().begin(); landmark_it != wolf_manager->getProblem()->getMap()->getLandmarkList().end(); landmark_it++)
//    {
//        Eigen::Map<Eigen::Vector2s> landmark((*landmark_it)->getP()->get());
//        landmarks.segment(i, 2) = landmark;
//        i += 2;
//    }
//
//    // Print log files
//    std::string filepath = getenv("HOME") + std::string("/Desktop/log_file_2.txt");
//    log_file.open(filepath, std::ofstream::out); //open log file
//
//    if (log_file.is_open())
//    {
//        log_file << 0 << std::endl;
//        for (unsigned int ii = 0; ii < n_execution; ii++)
//            log_file << state_poses.segment(ii * 3, 3).transpose() << "\t" << ground_truth.segment(ii * 3, 3).transpose() << "\t" << (state_poses.segment(ii * 3, 3) - ground_truth.segment(ii * 3, 3)).transpose() << "\t" << odom_trajectory.segment(ii * 3, 3).transpose() << std::endl;
//        log_file.close(); //close log file
//        std::cout << std::endl << "Result file " << filepath << std::endl;
//    }
//    else
//        std::cout << std::endl << "Failed to write the log file " << filepath << std::endl;
//
//    std::string filepath2 = getenv("HOME") + std::string("/Desktop/landmarks_file_2.txt");
//    landmark_file.open(filepath2, std::ofstream::out); //open log file
//
//    if (landmark_file.is_open())
//    {
//        for (unsigned int ii = 0; ii < landmarks.size(); ii += 2)
//            landmark_file << landmarks.segment(ii, 2).transpose() << std::endl;
//        landmark_file.close(); //close log file
//        std::cout << std::endl << "Landmark file " << filepath << std::endl;
//    }
//    else
//        std::cout << std::endl << "Failed to write the landmark file " << filepath << std::endl;
//
//    std::cout << "Press any key for ending... " << std::endl << std::endl;
//    std::getchar();

    delete myRender;
    delete myScanner;
    delete wolf_manager_ceres;
    delete wolf_manager_wolf;
    std::cout << "wolf deleted" << std::endl;
    delete ceres_manager_ceres;
    delete ceres_manager_wolf;
    std::cout << "ceres_manager deleted" << std::endl;

    std::cout << " ========= END ===========" << std::endl << std::endl;

    //exit
    return 0;
}

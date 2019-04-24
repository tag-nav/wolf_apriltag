/*
 * test_iQR_wolf.cpp
 *
 *  Created on: Jun 17, 2015
 *      Author: jvallve
 */

//std includes
#include <cstdlib>
#include <string>
#include <iostream>
#include <fstream>
#include <memory>
#include <random>
#include <typeinfo>
#include <ctime>
#include <queue>

//Wolf includes
#include "base/state_block/state_block.h"
#include "base/factor/factor_base.h"
#include "base/sensor/sensor_laser_2D.h"
#include "wolf_manager.h"

// wolf solver
#include "solver/qr_solver.h"

//C includes for sleep, time and main args
#include "unistd.h"

//faramotics includes
#include "faramotics/dynamicSceneRender.h"
#include "faramotics/rangeScan2D.h"
#include "btr-headers/pose3d.h"

//Ceres includes
#include "glog/logging.h"
#include "base/ceres_wrapper/ceres_manager.h"

//laser_scan_utils
#include "iri-algorithms/laser_scan_utils/corner_detector.h"
#include "iri-algorithms/laser_scan_utils/entities.h"

//function travel around
void motionCampus(unsigned int ii, Cpose3d & pose, double& displacement_, double& rotation_)
{
    if (ii <= 120)
    {
        displacement_ = 0.1;
        rotation_ = 0;
    }
    else if ((ii > 120) && (ii <= 170))
    {
        displacement_ = 0.2;
        rotation_ = 1.8 * M_PI / 180;
    }
    else if ((ii > 170) && (ii <= 220))
    {
        displacement_ = 0;
        rotation_ = -1.8 * M_PI / 180;
    }
    else if ((ii > 220) && (ii <= 310))
    {
        displacement_ = 0.1;
        rotation_ = 0;
    }
    else if ((ii > 310) && (ii <= 487))
    {
        displacement_ = 0.1;
        rotation_ = -1. * M_PI / 180;
    }
    else if ((ii > 487) && (ii <= 600))
    {
        displacement_ = 0.2;
        rotation_ = 0;
    }
    else if ((ii > 600) && (ii <= 700))
    {
        displacement_ = 0.1;
        rotation_ = -1. * M_PI / 180;
    }
    else if ((ii > 700) && (ii <= 780))
    {
        displacement_ = 0;
        rotation_ = -1. * M_PI / 180;
    }
    else
    {
        displacement_ = 0.3;
        rotation_ = 0.0 * M_PI / 180;
    }

    pose.moveForward(displacement_);
    pose.rt.setEuler(pose.rt.head() + rotation_, pose.rt.pitch(), pose.rt.roll());
}

//main
int main(int argc, char *argv[])
{
    using namespace Eigen;
    using namespace wolf;

    // USER INPUT ============================================================================================
    if (argc != 3 || atoi(argv[1]) < 1 || atoi(argv[1]) > 1100 || atoi(argv[2]) < 0 || atoi(argv[2]) > 2)
    {
        std::cout << "Please call me with: [./test_ceres_manager NI MODE], where:" << std::endl;
        std::cout << "     - NI is the number of iterations (0 < NI < 1100)" << std::endl;
        std::cout << "     - MODE is the solver mode (0 batch (no ordering), 1 batch (ordering), 2 incremental" << std::endl;
        std::cout << "EXIT due to bad user input" << std::endl << std::endl;
        return -1;
    }
    bool complex_angle = false;
    unsigned int solving_mode = (unsigned int) atoi(argv[2]); //solving mode
    unsigned int n_execution = (unsigned int) atoi(argv[1]); //number of iterations of the whole execution

    // INITIALIZATION ============================================================================================

    //init random generators
    Scalar odom_std_factor = 0.1;
    Scalar gps_std = 10;
    std::default_random_engine generator(1);
    std::normal_distribution<Scalar> gaussian_distribution(0.0, 1);

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
    CrangeScan2D* myScanner = new CrangeScan2D(HOKUYO_UTM30LX_180DEG);  //HOKUYO_UTM30LX_180DEG or LEUZE_RS4
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
    SensorLaser2D laser_1_sensor(std::make_shared<StateBlock>(laser_1_pose.head(2)), std::make_shared<StateBlock>(laser_1_pose.tail(1)), laserscanutils::LaserScanParams({M_PI/2,-M_PI/2, -M_PI/720,0.01,0.2,100,0.01,0.01}));
    SensorLaser2D laser_2_sensor(std::make_shared<StateBlock>(laser_2_pose.head(2)), std::make_shared<StateBlock>(laser_2_pose.tail(1)), laserscanutils::LaserScanParams({M_PI/2,-M_PI/2, -M_PI/720,0.01,0.2,100,0.01,0.01}));

    // Initial pose
    pose_odom << 2, 8, 0;
    ground_truth.head(3) = pose_odom;
    odom_trajectory.head(3) = pose_odom;

    WolfManager* wolf_manager_QR = new WolfManager(FRM_PO_2D, &odom_sensor, pose_odom, Eigen::Matrix3s::Identity() * 0.01, n_execution*10, 0.01);
    WolfManager* wolf_manager_ceres = new WolfManager(FRM_PO_2D, &odom_sensor, pose_odom, Eigen::Matrix3s::Identity() * 0.01, n_execution*10, 0.01);

    // Ceres initialization
    ceres::Solver::Options ceres_options;
    ceres_options.minimizer_type = ceres::TRUST_REGION; //ceres::TRUST_REGION;LINE_SEARCH
    ceres_options.max_line_search_step_contraction = 1e-3;
    //    ceres_options.minimizer_progress_to_stdout = false;
    //    ceres_options.line_search_direction_type = ceres::LBFGS;
    //    ceres_options.max_num_iterations = 100;
    CeresManager* ceres_manager = new CeresManager(wolf_manager_ceres->getProblem(), ceres_options);
    std::ofstream log_file, landmark_file;  //output file

    // Own solver
    SolverQR solver_(wolf_manager_QR->getProblem());

    std::cout << "STARTING INCREMENTAL QR TEST" << std::endl << std::endl;
    std::cout << "\n ========= 2D Robot with odometry and 2 LIDARs ===========\n";
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
        odom_reading(0) += gaussian_distribution(generator) * odom_std_factor * (odom_reading(0) == 0 ? 1e-6 : odom_reading(0));
        odom_reading(1) += gaussian_distribution(generator) * odom_std_factor * 1e-6;
        odom_reading(2) += gaussian_distribution(generator) * odom_std_factor * (odom_reading(2) == 0 ? 1e-6 : odom_reading(2));

        // odometry integration
        pose_odom(0) = pose_odom(0) + odom_reading(0) * cos(pose_odom(2)) - odom_reading(1) * sin(pose_odom(2));
        pose_odom(1) = pose_odom(1) + odom_reading(0) * sin(pose_odom(2)) + odom_reading(1) * cos(pose_odom(2));
        pose_odom(2) = pose_odom(2) + odom_reading(1);
        odom_trajectory.segment(step * 3, 3) = pose_odom;

        // compute GPS
        gps_fix_reading << devicePose.pt(0), devicePose.pt(1);
        gps_fix_reading(0) += gaussian_distribution(generator) * gps_std;
        gps_fix_reading(1) += gaussian_distribution(generator) * gps_std;

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
        //std::cout << "ADD CAPTURES..." << std::endl;
        // adding new sensor captures
        wolf_manager_QR->addCapture(new CaptureOdom2D(TimeStamp(), TimeStamp(), &odom_sensor, odom_reading));       //, odom_std_factor * Eigen::MatrixXs::Identity(2,2)));
        wolf_manager_QR->addCapture(new CaptureGPSFix(TimeStamp(), &gps_sensor, gps_fix_reading, gps_std * gps_std * Eigen::MatrixXs::Identity(3,3)));
        wolf_manager_ceres->addCapture(new CaptureOdom2D(TimeStamp(), TimeStamp(), &odom_sensor, odom_reading));       //, odom_std_factor * Eigen::MatrixXs::Identity(2,2)));
        wolf_manager_ceres->addCapture(new CaptureGPSFix(TimeStamp(), &gps_sensor, gps_fix_reading, gps_std * gps_std * Eigen::MatrixXs::Identity(3,3)));
        //wolf_manager->addCapture(new CaptureLaser2D(TimeStamp(), &laser_1_sensor, scan1));
        //wolf_manager->addCapture(new CaptureLaser2D(TimeStamp(), &laser_2_sensor, scan2));
        // updating problem
        wolf_manager_QR->update();
        wolf_manager_ceres->update();

        // UPDATING SOLVER ---------------------------
        //std::cout << "UPDATING..." << std::endl;
        // update state units and factors in ceres
        solver_.update();

        // PRINT PROBLEM
        //solver_.printProblem();

        // SOLVE OPTIMIZATION ---------------------------
        //std::cout << "SOLVING..." << std::endl;
        ceres::Solver::Summary summary = ceres_manager->solve();
        solver_.solve(solving_mode);

        std::cout << "========================= RESULTS " << step << ":" << std::endl;
        //solver_.printResults();
        std::cout << "QR vehicle pose    " << wolf_manager_QR->getVehiclePose().transpose() << std::endl;
        std::cout << "ceres vehicle pose " << wolf_manager_ceres->getVehiclePose().transpose() << std::endl;

        // COMPUTE COVARIANCES ---------------------------
        //std::cout << "COMPUTING COVARIANCES..." << std::endl;
        // TODO

        // DRAWING STUFF ---------------------------
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

        // draw landmarks
        std::vector<double> landmark_vector;
        for (auto landmark_it = wolf_manager_QR->getProblem()->getMap()->getLandmarkList().begin(); landmark_it != wolf_manager_QR->getProblem()->getMap()->getLandmarkList().end(); landmark_it++)
        {
            Scalar* position_ptr = (*landmark_it)->getP()->get();
            landmark_vector.push_back(*position_ptr); //x
            landmark_vector.push_back(*(position_ptr + 1)); //y
            landmark_vector.push_back(0.2); //z
        }
        myRender->drawLandmarks(landmark_vector);

        // draw localization and sensors
        estimated_vehicle_pose.setPose(wolf_manager_QR->getVehiclePose()(0), wolf_manager_QR->getVehiclePose()(1), 0.2, wolf_manager_QR->getVehiclePose()(2), 0, 0);
        estimated_laser_1_pose.setPose(estimated_vehicle_pose);
        estimated_laser_1_pose.moveForward(laser_1_pose(0));
        estimated_laser_2_pose.setPose(estimated_vehicle_pose);
        // instead of laser 2 we draw ceres solution
        //estimated_laser_2_pose.moveForward(laser_2_pose(0));
        //estimated_laser_2_pose.rt.setEuler(estimated_laser_2_pose.rt.head() + M_PI, estimated_laser_2_pose.rt.pitch(), estimated_laser_2_pose.rt.roll());
        estimated_laser_2_pose.setPose(wolf_manager_ceres->getVehiclePose()(0), wolf_manager_ceres->getVehiclePose()(1), 0.2, wolf_manager_ceres->getVehiclePose()(2), 0, 0);

        myRender->drawPoseAxisVector( { estimated_vehicle_pose, estimated_laser_1_pose, estimated_laser_2_pose });

        //Set view point and render the scene
        //locate visualization view point, somewhere behind the device
        myRender->setViewPoint(viewPoint);
        myRender->drawPoseAxis(devicePose);
        myRender->drawScan(laser1Pose, scan1, 180. * M_PI / 180., 90. * M_PI / 180.); //draw scan
        myRender->render();

        // TIME MANAGEMENT ---------------------------
        double dt = ((double) clock() - t2) / CLOCKS_PER_SEC;
        mean_times(6) += dt;
        if (dt < 0.1)
            usleep(100000 - 1e6 * dt);

//      std::cout << "\nTree after step..." << std::endl;
//      wolf_manager->getProblem()->print();
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

//  std::cout << "\nTree before deleting..." << std::endl;
//  wolf_manager->getProblem()->print();

    // Draw Final result -------------------------
    std::cout << "Drawing final results..." << std::endl;
    std::vector<double> landmark_vector;
    for (auto landmark_it = wolf_manager_QR->getProblem()->getMap()->getLandmarkList().begin(); landmark_it != wolf_manager_QR->getProblem()->getMap()->getLandmarkList().end(); landmark_it++)
    {
        Scalar* position_ptr = (*landmark_it)->getP()->get();
        landmark_vector.push_back(*position_ptr); //x
        landmark_vector.push_back(*(position_ptr + 1)); //y
        landmark_vector.push_back(0.2); //z
    }
    myRender->drawLandmarks(landmark_vector);
//  viewPoint.setPose(devicePoses.front());
//  viewPoint.moveForward(10);
//  viewPoint.rt.setEuler( viewPoint.rt.head()+M_PI/4, viewPoint.rt.pitch()+20.*M_PI/180., viewPoint.rt.roll() );
//  viewPoint.moveForward(-10);
    myRender->setViewPoint(viewPoint);
    myRender->render();

    // Print Final result in a file -------------------------
    std::cout << "Printing results in a file..." << std::endl;
    // Vehicle poses
    std::cout << "Vehicle poses..." << std::endl;
    int i = 0;
    Eigen::VectorXs state_poses(wolf_manager_QR->getProblem()->getTrajectory()->getFrameList().size() * 3);
    for (auto frame_it = wolf_manager_QR->getProblem()->getTrajectory()->getFrameList().begin(); frame_it != wolf_manager_QR->getProblem()->getTrajectory()->getFrameList().end(); frame_it++)
    {
        if (complex_angle)
            state_poses.segment(i, 3) << *(*frame_it)->getP()->get(), *((*frame_it)->getP()->get() + 1), atan2(*(*frame_it)->getO()->get(), *((*frame_it)->getO()->get() + 1));
        else
            state_poses.segment(i, 3) << *(*frame_it)->getP()->get(), *((*frame_it)->getP()->get() + 1), *(*frame_it)->getO()->get();
        i += 3;
    }

    // Landmarks
    std::cout << "Landmarks..." << std::endl;
    i = 0;
    Eigen::VectorXs landmarks(wolf_manager_QR->getProblem()->getMap()->getLandmarkList().size() * 2);
    for (auto landmark_it = wolf_manager_QR->getProblem()->getMap()->getLandmarkList().begin(); landmark_it != wolf_manager_QR->getProblem()->getMap()->getLandmarkList().end(); landmark_it++)
    {
        Eigen::Map<Eigen::Vector2s> landmark((*landmark_it)->getP()->get());
        landmarks.segment(i, 2) = landmark;
        i += 2;
    }

    // Print log files
    std::string filepath = getenv("HOME") + (complex_angle ? std::string("/Desktop/log_file_3.txt") : std::string("/Desktop/log_file_2.txt"));
    log_file.open(filepath, std::ofstream::out); //open log file

    if (log_file.is_open())
    {
        log_file << 0 << std::endl;
        for (unsigned int ii = 0; ii < n_execution; ii++)
            log_file << state_poses.segment(ii * 3, 3).transpose() << "\t" << ground_truth.segment(ii * 3, 3).transpose() << "\t" << (state_poses.segment(ii * 3, 3) - ground_truth.segment(ii * 3, 3)).transpose() << "\t" << odom_trajectory.segment(ii * 3, 3).transpose() << std::endl;
        log_file.close(); //close log file
        std::cout << std::endl << "Result file " << filepath << std::endl;
    }
    else
        std::cout << std::endl << "Failed to write the log file " << filepath << std::endl;

    std::string filepath2 = getenv("HOME") + (complex_angle ? std::string("/Desktop/landmarks_file_3.txt") : std::string("/Desktop/landmarks_file_2.txt"));
    landmark_file.open(filepath2, std::ofstream::out); //open log file

    if (landmark_file.is_open())
    {
        for (unsigned int ii = 0; ii < landmarks.size(); ii += 2)
            landmark_file << landmarks.segment(ii, 2).transpose() << std::endl;
        landmark_file.close(); //close log file
        std::cout << std::endl << "Landmark file " << filepath << std::endl;
    }
    else
        std::cout << std::endl << "Failed to write the landmark file " << filepath << std::endl;

    std::cout << "Press any key for ending... " << std::endl << std::endl;
    std::getchar();

    delete myRender;
    delete myScanner;
    delete wolf_manager_QR;
    delete wolf_manager_ceres;
    std::cout << "wolf deleted" << std::endl;

    std::cout << " ========= END ===========" << std::endl << std::endl;

    //exit
    return 0;
}


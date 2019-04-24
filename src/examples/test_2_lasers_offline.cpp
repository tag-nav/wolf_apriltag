//std includes
#include "base/sensor/sensor_GPS_fix.h"
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
#include "base/problem/problem.h"
#include "base/processor/processor_tracker_landmark_corner.h"
#include "base/processor/processor_odom_2D.h"
#include "base/sensor/sensor_laser_2D.h"
#include "base/sensor/sensor_odom_2D.h"
#include "base/ceres_wrapper/ceres_manager.h"

// laserscanutils
#include "laser_scan_utils/line_finder_iterative.h"
#include "laser_scan_utils/laser_scan.h"

//C includes for sleep, time and main args
#include "unistd.h"

#include "base/capture/capture_pose.h"
//faramotics includes
#include "faramotics/dynamicSceneRender.h"
#include "faramotics/rangeScan2D.h"
#include "btr-headers/pose3d.h"

void extractVector(std::ifstream& text_file, Eigen::VectorXs& vector, wolf::Scalar& ts)
{
    std::string line;
    std::getline(text_file, line);
    std::stringstream line_stream(line);
    line_stream >> ts;
    for (auto i = 0; i < vector.size(); i++)
        line_stream >> vector(i);
}

void extractScan(std::ifstream& text_file, std::vector<float>& scan, wolf::Scalar& ts)
{
    std::string line;
    std::getline(text_file, line);
    std::stringstream line_stream(line);
    line_stream >> ts;
    for (unsigned int i = 0; i < scan.size(); i++)
        line_stream >> scan[i];
}

int main(int argc, char** argv)
{
    using namespace wolf;

    std::cout << "\n==================================================================\n";
    std::cout << "========== 2D Robot with offline odometry and 2 LIDARs =============\n";

    // USER INPUT ============================================================================================
    if (argc != 2 || atoi(argv[1]) < 1 )
    {
        std::cout << "Please call me with: [./test_ceres_manager NI PRINT], where:" << std::endl;
        std::cout << "     - NI is the number of iterations (NI > 0)" << std::endl;
        std::cout << "EXIT due to bad user input" << std::endl << std::endl;
        return -1;
    }

    // FILES INPUT ============================================================================================
    std::ifstream laser_1_file, laser_2_file, odom_file, groundtruth_file;
    laser_1_file.open("simulated_laser_1.txt", std::ifstream::in);
    laser_2_file.open("simulated_laser_2.txt", std::ifstream::in);
    odom_file.open("simulated_odom.txt", std::ifstream::in);
    groundtruth_file.open("simulated_groundtruth.txt", std::ifstream::in);

    if (!(laser_1_file.is_open() && laser_2_file.is_open() && odom_file.is_open() && groundtruth_file.is_open()))
    {
        std::cout << "Error opening simulated data files. Remember to run test_faramotics_simulation before this test!" << std::endl;
        return -1;
    }
    else
        std::cout << "Simulated data files opened correctly..." << std::endl;

    unsigned int n_execution = (unsigned int) atoi(argv[1]); //number of iterations of the whole execution

    // INITIALIZATION ============================================================================================
    //init random generators
    Scalar odom_std_factor = 0.5;
    std::default_random_engine generator(1);
    std::normal_distribution<Scalar> distribution_odom(0.0, odom_std_factor); //odometry noise

    //variables
    std::string line;
    Eigen::VectorXs odom_data = Eigen::VectorXs::Zero(2);
    Eigen::VectorXs ground_truth(n_execution * 3); //all true poses
    Eigen::VectorXs ground_truth_pose(3); //last true pose
    Eigen::VectorXs odom_trajectory(n_execution * 3); //open loop trajectory
    Eigen::VectorXs mean_times = Eigen::VectorXs::Zero(6);
    clock_t t1, t2;
    Scalar timestamp;
    TimeStamp ts(0);

    // Wolf initialization
    Eigen::VectorXs odom_pose = Eigen::VectorXs::Zero(3);
    //Eigen::VectorXs gps_position = Eigen::VectorXs::Zero(2);
    Eigen::VectorXs laser_1_params(9), laser_2_params(9);
    Eigen::VectorXs laser_1_pose(4), laser_2_pose(4); //xyz + theta
    Eigen::VectorXs laser_1_pose2D(3), laser_2_pose2D(3); //xy + theta

    // odometry intrinsics
    IntrinsicsOdom2D odom_intrinsics;
    odom_intrinsics.k_disp_to_disp = odom_std_factor;
    odom_intrinsics.k_rot_to_rot = odom_std_factor;

    // laser 1 extrinsics and intrinsics
    extractVector(laser_1_file, laser_1_params, timestamp);
    extractVector(laser_1_file, laser_1_pose, timestamp);
    laser_1_pose2D.head<2>() = laser_1_pose.head<2>();
    laser_1_pose2D(2) = laser_1_pose(3);
    std::vector<float> scan1(laser_1_params(8)); // number of ranges in a scan
    IntrinsicsLaser2D laser_1_intrinsics;
    laser_1_intrinsics.scan_params = laserscanutils::LaserScanParams({laser_1_params(0), laser_1_params(1), laser_1_params(2), laser_1_params(3), laser_1_params(4), laser_1_params(5), laser_1_params(6), laser_1_params(7)});

    ProcessorParamsLaser laser_1_processor_params;
    laser_1_processor_params.line_finder_params_ = laserscanutils::LineFinderIterativeParams({0.1, 5, 1, 2});
    laser_1_processor_params.new_corners_th = 10;

    // laser 2 extrinsics and intrinsics
    extractVector(laser_2_file, laser_2_params, timestamp);
    extractVector(laser_2_file, laser_2_pose, timestamp);
    laser_2_pose2D.head<2>() = laser_2_pose.head<2>();
    laser_2_pose2D(2) = laser_2_pose(3);
    std::vector<float> scan2(laser_2_params(8));
    IntrinsicsLaser2D laser_2_intrinsics;
    laser_2_intrinsics.scan_params = laserscanutils::LaserScanParams({laser_2_params(0), laser_2_params(1), laser_2_params(2), laser_2_params(3), laser_2_params(4), laser_2_params(5), laser_2_params(6), laser_2_params(7)});

    ProcessorParamsLaser laser_2_processor_params;
    laser_2_processor_params.line_finder_params_ = laserscanutils::LineFinderIterativeParams({0.1, 5, 1, 2});
    laser_2_processor_params.new_corners_th = 10;

    Problem problem(FRM_PO_2D);
    SensorOdom2D* odom_sensor = (SensorOdom2D*)problem.installSensor("ODOM 2D", "odometer", odom_pose, &odom_intrinsics);
    ProcessorParamsOdom2D odom_params;
    odom_params.cov_det = 1;
    odom_params.dist_traveled_th_ = 5;
    odom_params.elapsed_time_th_ = 10;
    ProcessorOdom2D* odom_processor = (ProcessorOdom2D*)problem.installProcessor("ODOM 2D", "main odometry", odom_sensor, &odom_params);
    //SensorBasePtr gps_sensor = problem.installSensor("GPS FIX", "GPS fix", gps_position);
    SensorBasePtr laser_1_sensor = problem.installSensor("LASER 2D", "front laser", laser_1_pose2D, &laser_1_intrinsics);
    SensorBasePtr laser_2_sensor = problem.installSensor("LASER 2D", "rear laser", laser_2_pose2D, &laser_2_intrinsics);
    problem.installProcessor("LASER 2D", "front laser processor", laser_1_sensor, &laser_1_processor_params);
    problem.installProcessor("LASER 2D", "rear laser processor", laser_2_sensor, &laser_2_processor_params);

    std::cout << "Wolf tree setted correctly!" << std::endl;

    CaptureMotion* odom_capture = new CaptureMotion(ts, odom_sensor, odom_data, Eigen::Matrix2s::Identity() * odom_std_factor * odom_std_factor, nullptr);

    // Initial pose
    ground_truth_pose << 2, 8, 0;
    ground_truth.head(3) = ground_truth_pose;
    odom_trajectory.head(3) = ground_truth_pose;

    // Origin Key Frame with covariance
    problem.setPrior(ground_truth_pose, Eigen::Matrix3s::Identity() * 0.1, ts, 0.1);

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

    std::cout << "START TRAJECTORY..." << std::endl;
    // START TRAJECTORY ============================================================================================
    for (unsigned int step = 1; step < n_execution; step++)
    {
        //get init time
        t2 = clock();

        // GROUNDTRUTH ---------------------------
        //std::cout << "GROUND TRUTH..." << std::endl;
        t1 = clock();
        extractVector(groundtruth_file, ground_truth_pose, timestamp);
        ground_truth.segment(step * 3, 3) = ground_truth_pose;

        // timestamp
        ts = TimeStamp(timestamp);

        // ODOMETRY DATA -------------------------------------
        //std::cout << "ODOMETRY DATA..." << std::endl;
        extractVector(odom_file, odom_data, timestamp);
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
        extractScan(laser_1_file, scan1, timestamp);
        extractScan(laser_2_file, scan2, timestamp);
        if (step % 3 == 0)
        {
            std::cout << "--PROCESS LIDAR 1 DATA..." << std::endl;
            CaptureLaser2D* new_scan_1 = new CaptureLaser2D(ts, laser_1_sensor, scan1);
            new_scan_1->process();
            std::cout << "--PROCESS LIDAR 2 DATA..." << std::endl;
            CaptureLaser2D* new_scan_2 = new CaptureLaser2D(ts, laser_2_sensor, scan2);
            new_scan_2->process();
        }
        mean_times(0) += ((double) clock() - t1) / CLOCKS_PER_SEC;

        // SOLVE OPTIMIZATION ---------------------------
        std::cout << "SOLVING..." << std::endl;
        t1 = clock();
        ceres::Solver::Summary summary = ceres_manager.solve();
        //std::cout << summary.FullReport() << std::endl;
        mean_times(3) += ((double) clock() - t1) / CLOCKS_PER_SEC;

        // COMPUTE COVARIANCES ---------------------------
        std::cout << "COMPUTING COVARIANCES..." << std::endl;
        t1 = clock();
        ceres_manager.computeCovariances();
        mean_times(4) += ((double) clock() - t1) / CLOCKS_PER_SEC;

        // TIME MANAGEMENT ---------------------------
        double total_t = ((double) clock() - t2) / CLOCKS_PER_SEC;
        mean_times(5) += total_t;
        if (total_t < 0.2)
            usleep(200000 - 1e6 * total_t);

//		std::cout << "\nTree after step..." << std::endl;
    }

    // DISPLAY RESULTS ============================================================================================
    mean_times /= n_execution;
    std::cout << "\nSIMULATION AVERAGE LOOP DURATION [s]" << std::endl;
    std::cout << "  data reading:    " << mean_times(0) << std::endl;
    std::cout << "  wolf managing:      " << mean_times(1) << std::endl;
    std::cout << "  ceres managing:     " << mean_times(2) << std::endl;
    std::cout << "  ceres optimization: " << mean_times(3) << std::endl;
    std::cout << "  ceres covariance:   " << mean_times(4) << std::endl;
    std::cout << "  loop time:          " << mean_times(5) << std::endl;

    //	std::cout << "\nTree before deleting..." << std::endl;

    // Print Final result in a file -------------------------
    // Vehicle poses
    int i = 0;
    Eigen::VectorXs state_poses = Eigen::VectorXs::Zero(n_execution * 3);
    for (auto frame : *(problem.getTrajectory()->getFrameList()))
    {
        state_poses.segment(i, 3) << frame->getP()->getVector(), frame->getO()->getVector();
        i += 3;
    }

    // Landmarks
    i = 0;
    Eigen::VectorXs landmarks = Eigen::VectorXs::Zero(problem.getMap()->getLandmarkList()->size() * 2);
    for (auto landmark : *(problem.getMap()->getLandmarkList()))
    {
        landmarks.segment(i, 2) = landmark->getP()->getVector();
        i += 2;
    }

    // Print log files
    std::string filepath = getenv("HOME") + std::string("/Desktop/log_file_2.txt");
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

    std::string filepath2 = getenv("HOME") + std::string("/Desktop/landmarks_file_2.txt");
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

    std::cout << "Problem:" << std::endl;
    std::cout << "Frames: " << problem.getTrajectory()->getFrameList().size() << std::endl;
    std::cout << "Landmarks: " << problem.getMap()->getLandmarkList()->size() << std::endl;
    std::cout << "Sensors: " << problem.getHardware()->getSensorList()->size() << std::endl;

    std::cout << " ========= END ===========" << std::endl << std::endl;

    //exit
    return 0;
}

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

//C includes for sleep, time and main args
#include "unistd.h"

// wolf
#include "core/common/wolf.h"
#include "core/feature/feature_base.h"
#include "core/landmark/landmark_base.h"
#include "core/state_block/state_block.h"

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
            //std::cout << "   drawCorners: " << feature_list.size() << std::endl;
            std::vector<double> corner_vector;
            corner_vector.reserve(2*feature_list.size());
            for (auto corner : feature_list)
            {
                //std::cout << "       corner " << corner->id() << std::endl;
                corner_vector.push_back(corner->getMeasurement(0));
                corner_vector.push_back(corner->getMeasurement(1));
            }
            myRender->drawCorners(laser == 1 ? laser1Pose : laser2Pose, corner_vector);

            // landmarks
            //std::cout << "   drawLandmarks: " << landmark_list.size() << std::endl;
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

    unsigned int n_execution = 900; //number of iterations of the whole execution

    // VARIABLES ============================================================================================
    Eigen::Vector2s odom_data;
    Eigen::Vector3s ground_truth;
    Scalar dt = 0.05;

    // Laser params
    Eigen::Vector4s laser_1_pose, laser_2_pose; //xyz + theta
    laser_1_pose << 1.2, 0, 0, 0; //laser 1
    laser_2_pose << -1.2, 0, 0, M_PI; //laser 2
    Eigen::VectorXs laser_params(8);
    laser_params << M_PI/2, -M_PI/2, -M_PI/720, 0.01, 0.2, 100, 0.01, 0.01;
    std::vector<float> scan1, scan2;

    // Simulated robot
    FaramoticsRobot robot(argc, argv, laser_1_pose, laser_2_pose);

    // Initial pose
    ground_truth << 2, 8, 0;

    //output file
    std::ofstream laser_1_file, laser_2_file, odom_file, groundtruth_file;
    groundtruth_file.open("simulated_groundtruth.txt", std::ofstream::out); //open log file
    odom_file.open("simulated_odom.txt", std::ofstream::out); //open log file
    laser_1_file.open("simulated_laser_1.txt", std::ofstream::out); //open log file
    laser_2_file.open("simulated_laser_2.txt", std::ofstream::out); //open log file

    // write laser params
    laser_1_file << 0 << " " << laser_params.transpose() << " " << robot.myScanner->getNumPoints() << std::endl;
    laser_2_file << 0 << " " << laser_params.transpose() << " " << robot.myScanner->getNumPoints() << std::endl;
    laser_1_file << 0 << " " << laser_1_pose.transpose() << std::endl;
    laser_2_file << 0 << " " << laser_2_pose.transpose() << std::endl;

    // origin frame groundtruth
    groundtruth_file << 0 << " " << ground_truth.transpose() << std::endl;

    //std::cout << "START TRAJECTORY..." << std::endl;
    // START TRAJECTORY ============================================================================================
    for (unsigned int step = 1; step < n_execution; step++)
    {
        // ROBOT MOVEMENT ---------------------------
        ground_truth = robot.motionCampus(step, odom_data(0), odom_data(1));

        // LIDAR DATA ---------------------------
        scan1 = robot.computeScan(1);
        scan2 = robot.computeScan(2);

        // writing files ---------------------------
        groundtruth_file << step*dt << " " << ground_truth.transpose() << std::endl;
        odom_file << step*dt << " " << odom_data.transpose() << std::endl;
        laser_1_file << step*dt << " ";
        for (auto range : scan1)
            laser_1_file << range << " ";
        laser_1_file << std::endl;
        laser_2_file << step*dt << " ";
        for (auto range : scan1)
            laser_2_file << range << " ";
        laser_2_file << std::endl;
    }

    groundtruth_file.close(); //close log file
    odom_file.close(); //close log file
    laser_1_file.close(); //close log file
    laser_2_file.close(); //close log file

    std::cout << " ========= END ===========" << std::endl << std::endl;

    //exit
    return 0;
}

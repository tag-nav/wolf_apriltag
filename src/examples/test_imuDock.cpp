/**
 * \file test_imuDock.cpp
 *
 *  Created on: July 18, 2017
 *      \author: Dinesh Atchuthan
 */

#include "core/processor/processor_IMU.h"
#include "core/sensor/sensor_IMU.h"
#include "core/common/wolf.h"
#include "core/problem/problem.h"
#include "core/ceres_wrapper/ceres_manager.h"
#include "core/sensor/sensor_odom_3D.h"
#include "core/processor/processor_odom_3D.h"

//Factors headers
#include "core/factor/factor_fix_bias.h"

//std
#include <iostream>
#include <fstream>
#include "core/factor/factor_pose_3D.h"

#define OUTPUT_RESULTS
//#define ADD_KF3

/*                              OFFLINE VERSION
    In this test, we use the experimental conditions needed for Humanoids 2017.
    IMU data are acquired using the docking station. 

    Factors are (supposing to KeyFrames, stateblocks or first Keyframe are noted *1 and second Keyframes's are *2) :
    invar       : P1, V1, V2
    var         : Q1,B1,P2,Q2,B2
    factors : Odometry factor between KeyFrames
                  IMU factor
                  FixBias factor --> makes the problem observable (adding a big covariance on all part but a smaller one on yaw)
                  Fix3D factor

    What we expect  : Estimate biases (this will strongly depend on the actual trajectory of the IMU)
                      Estimate the position and orienttion of the IMU (check with the standard deviation using covariance matrix)

    Representation of the application:

                                    Imu
                        KF1----------◼----------KF2
                   /----P1----------\ /----------P2             invar       : P1, V1, V2
        Abs|------◼                 ◼                          var         : Q1,B1,P2,Q2,B2
                   \----Q1----------/ \----------Q2
                        V1          Odom
        Abs|------◼-----B1
*/
int main(int argc, char** argv)
{
    //#################################################### INITIALIZATION
    using namespace wolf;

    //___get input file for imu data___
    std::ifstream imu_data_input;
    const char * filename_imu;
    if (argc < 02)
    {
        WOLF_ERROR("Missing 1 input argument (path to imu data file).")
        return 1; //return with error
    }
    else
    {
        filename_imu = argv[1];

        imu_data_input.open(filename_imu);
        WOLF_INFO("imu file : ", filename_imu)
    }

    // ___Check if the file is correctly opened___
    if(!imu_data_input.is_open()){
        WOLF_ERROR("Failed to open data file ! Exiting")
        return 1;
    }

    #ifdef OUTPUT_RESULTS
        //define output file
        std::ofstream output_results_before, output_results_after, checking;
        output_results_before.open("imu_dock_beforeOptim.dat");
        output_results_after.open("imu_dock_afterOptim.dat");
        checking.open("KF_pose_stdev.dat");
    #endif

    // ___initialize variabes that will be used through the code___
    Eigen::VectorXs problem_origin(16);
    Eigen::Vector7s imu_pose((Eigen::Vector7s()<<0,0,0,0,0,0,1).finished()), odom_pose((Eigen::Vector7s()<<0,0,0,0,0,0,1).finished());
    problem_origin << 0,0,0, 0,0,0,1, 0,0,0, 0,0,0, 0,0,0;
    
    //Create vectors to store data and time
    Eigen::Vector6s data_imu, data_odom;
    Scalar clock;
    TimeStamp ts(0), ts_output(0); //will be used to store the data timestamps and set timestamps in captures

    // ___Define expected values___
    Eigen::Vector7s expected_KF1_pose((Eigen::Vector7s()<<0,0,0,0,0,0,1).finished()), expected_KF2_pose((Eigen::Vector7s()<<0,-0.06,0,0,0,0,11).finished());

    //#################################################### SETTING PROBLEM
    std::string wolf_root = _WOLF_ROOT_DIR;

    // ___Create the WOLF Problem + define origin of the problem___
    ProblemPtr problem = Problem::create("PQVBB 3D");
    CeresManager* ceres_manager = new CeresManager(problem);
 
    // ___Configure Ceres if needed___

    // ___Create sensors + processors___
    SensorIMUPtr sensorIMU = std::static_pointer_cast<SensorIMU>(problem->installSensor("IMU", "Main IMU", imu_pose, wolf_root + "/src/examples/sensor_imu.yaml"));
    ProcessorIMUPtr processorIMU = std::static_pointer_cast<ProcessorIMU>(problem->installProcessor("IMU", "IMU pre-integrator", "Main IMU", wolf_root + "/src/examples/processor_imu.yaml"));
    
    SensorOdom3DPtr sensorOdom = std::static_pointer_cast<SensorOdom3D>(problem->installSensor("ODOM 3D", "odom", odom_pose, wolf_root + "/src/examples/sensor_odom_3D_HQ.yaml"));
    ProcessorOdom3DPtr processorOdom = std::static_pointer_cast<ProcessorOdom3D>(problem->installProcessor("ODOM 3D", "odom", "odom", wolf_root + "/src/examples/processor_odom_3D.yaml"));
    
    // ___set origin of processors to the problem's origin___
    FrameIMUPtr KF1 = std::static_pointer_cast<FrameIMU>(processorIMU->setOrigin(problem_origin, ts)); // XXX JS: setting ts to zero, and then reading clock from data, is inconsistent.
    processorOdom->setOrigin(KF1);

    //#################################################### PROCESS DATA
    // ___process IMU and odometry___

    //Create captures
    CaptureIMUPtr imu_ptr = std::make_shared<CaptureIMU>(ts, sensorIMU, data_imu, Matrix6s::Identity(), Vector6s::Zero()); //ts is set at 0
    CaptureMotionPtr mot_ptr = std::make_shared<CaptureMotion>(ts, sensorOdom, data_odom, 7, 6, nullptr);

    //while we do not reach the end of file, read IMU input (ts, Ax, Ay, Az, Wx, Wy, Wz) and process data through capture
    while(!imu_data_input.eof())
    {
        //read
        imu_data_input >> clock >> data_imu[0] >> data_imu[1] >> data_imu[2] >> data_imu[3] >> data_imu[4] >> data_imu[5];

        //set capture
        ts.set(clock);
        imu_ptr->setTimeStamp(ts);
        imu_ptr->setData(data_imu);

        //process
        sensorIMU->process(imu_ptr);
    }

    //All IMU data have been processed, close the file
    imu_data_input.close();

    //A KeyFrame should have been created (depending on time_span in processors). get the last KeyFrame
    // XXX JS: in my  opinion, we should control the KF creation better, not using time span. Is it possible?
    #ifdef ADD_KF3
        //Add a KeyFrame just before the motion actually starts (we did not move yet)
        data_odom << 0,0,0, 0,0,0;
        TimeStamp t_middle(0.307585);
        mot_ptr->setTimeStamp(t_middle);
        mot_ptr->setData(data_odom);
        sensorOdom->process(mot_ptr);

        //Also add a keyframe at the end of the motion
        data_odom << 0,-0.06,0, 0,0,0;
        ts.set(0.981573); //comment this if you want the last KF to be created at last imu's ts
        mot_ptr->setTimeStamp(ts);
        mot_ptr->setData(data_odom);
        sensorOdom->process(mot_ptr);

        FrameIMUPtr KF2 = std::static_pointer_cast<FrameIMU>(problem->getTrajectory()->closestKeyFrameToTimeStamp(t_middle));
        FrameIMUPtr KF3 = std::static_pointer_cast<FrameIMU>(problem->getTrajectory()->closestKeyFrameToTimeStamp(ts));
    #else
        //now impose final odometry using last timestamp of imu
        data_odom << 0,-0.06,0, 0,0,0;
        mot_ptr->setTimeStamp(ts);
        mot_ptr->setData(data_odom);
        sensorOdom->process(mot_ptr);

        FrameIMUPtr KF2 = std::static_pointer_cast<FrameIMU>(problem->getTrajectory()->closestKeyFrameToTimeStamp(ts));
    #endif

    //#################################################### OPTIMIZATION PART
    // ___Create needed factors___

    //Add Fix3D factor on first KeyFrame (with large covariance except for yaw)
    Eigen::MatrixXs featureFix_cov(6,6);
    featureFix_cov = Eigen::MatrixXs::Identity(6,6);
    featureFix_cov.topLeftCorner(3,3) *= 1e-8; // position variances (it's fixed anyway)
    featureFix_cov(3,3) = pow( .02  , 2); // roll variance
    featureFix_cov(4,4) = pow( .02  , 2); // pitch variance
    featureFix_cov(5,5) = pow( .01 , 2); // yaw variance
    CaptureBasePtr cap_fix = KF1->addCapture(std::make_shared<CaptureMotion>(0, nullptr, problem_origin.head(7), 7, 6, nullptr));
    FeatureBasePtr featureFix = cap_fix->addFeature(std::make_shared<FeatureBase>("ODOM 3D", problem_origin.head(7), featureFix_cov));
    FactorFix3DPtr fac_fix = std::static_pointer_cast<FactorPose3D>(featureFix->addFactor(std::make_shared<FactorPose3D>(featureFix)));

    Eigen::MatrixXs featureFixBias_cov(6,6);
    featureFixBias_cov = Eigen::MatrixXs::Identity(6,6); 
    featureFixBias_cov.topLeftCorner(3,3) *= sensorIMU->getAbInitialStdev() * sensorIMU->getAbInitialStdev();
    featureFixBias_cov.bottomRightCorner(3,3) *= sensorIMU->getWbInitialStdev() * sensorIMU->getWbInitialStdev();
    CaptureBasePtr cap_fixbias = KF1->addCapture(std::make_shared<CaptureMotion>(0, nullptr, problem_origin.tail(6), featureFixBias_cov, 6, 6, nullptr));
    //create a FeatureBase to factor biases
    FeatureBasePtr featureFixBias = cap_fixbias->addFeature(std::make_shared<FeatureBase>("FIX BIAS", problem_origin.tail(6), featureFixBias_cov));
    FactorFixBiasPtr fac_fixBias = std::static_pointer_cast<FactorFixBias>(featureFixBias->addFactor(std::make_shared<FactorFixBias>(featureFixBias)));

    // ___Fix/Unfix stateblocks___
    KF1->getP()->fix();
    KF1->getO()->unfix();
    KF1->getV()->fix();
    KF1->getAccBias()->unfix();
    KF1->getGyroBias()->unfix();

    #ifdef ADD_KF3
        KF2->getP()->fix();
        KF2->getO()->unfix();
        KF2->getV()->fix();
        KF2->getAccBias()->unfix();
        KF2->getGyroBias()->unfix();

        KF3->getP()->unfix();
        KF3->getO()->unfix();
        KF3->getV()->fix();
        KF3->getAccBias()->unfix();
        KF3->getGyroBias()->unfix();
    #else
        KF2->getP()->unfix();
        KF2->getO()->unfix();
        KF2->getV()->fix();
        KF2->getAccBias()->unfix();
        KF2->getGyroBias()->unfix();
    #endif

    #ifdef OUTPUT_RESULTS
        // ___OUTPUT___
        /* Produce output file for matlab visualization
         * first output : estimated trajectory BEFORE optimization (getting the states each millisecond)
         */

        unsigned int time_iter(0);
        Scalar ms(0.001);
        ts_output.set(0);
        while(ts_output.get() < ts.get() + ms)
        {
            output_results_before << ts_output.get() << "\t" << problem->getState(ts_output).transpose() << std::endl;
            time_iter++;
            ts_output.set(time_iter * ms);
        }
    #endif

    // ___Solve + compute covariances___
    problem->print(4,0,1,0);
    std::string report = ceres_manager->solve(SolverManager::ReportVerbosity::BRIEF); // 0: nothing, 1: BriefReport, 2: FullReport
    ceres_manager->computeCovariances(SolverManager::CovarianceBlocksToBeComputed::ALL_MARGINALS);
    problem->print(1,0,1,0);

    //#################################################### RESULTS PART

    // ___Get standard deviation from covariances___
    #ifdef ADD_KF3
        Eigen::MatrixXs cov_KF1(16,16), cov_KF2(16,16), cov_KF3(16,16);

        problem->getFrameCovariance(KF1, cov_KF1);
        problem->getFrameCovariance(KF2, cov_KF2);
        problem->getFrameCovariance(KF3, cov_KF3);

        Eigen::Matrix<wolf::Scalar, 16, 1> stdev_KF1, stdev_KF2, stdev_KF3;

        stdev_KF1 = cov_KF1.diagonal().array().sqrt();
        stdev_KF2 = cov_KF2.diagonal().array().sqrt();
        stdev_KF3 = cov_KF3.diagonal().array().sqrt();

        WOLF_DEBUG("stdev KF1 : ", stdev_KF1.transpose());
        WOLF_DEBUG("stdev KF2 : ", stdev_KF2.transpose());
        WOLF_DEBUG("stdev KF3 : ", stdev_KF3.transpose());
    #else
        Eigen::MatrixXs cov_KF1(16,16), cov_KF2(16,16);

        problem->getFrameCovariance(KF1, cov_KF1);
        problem->getFrameCovariance(KF2, cov_KF2);

        Eigen::Matrix<wolf::Scalar, 16, 1> stdev_KF1, stdev_KF2;

        stdev_KF1 = cov_KF1.diagonal().array().sqrt();
        stdev_KF2 = cov_KF2.diagonal().array().sqrt();

        WOLF_DEBUG("stdev KF1 : \n", stdev_KF1.transpose());
        WOLF_DEBUG("stdev KF2 : \n", stdev_KF2.transpose());
    #endif
    

    #ifdef OUTPUT_RESULTS
        // ___OUTPUT___
        /* Produce output file for matlab visualization
         * Second output:   KF2 position standard deviation computed
         *                  estimated trajectory AFTER optimization 
         *                  + get KF2 timestamp + state just in case the loop is not working as expected
         */

        //estimated trajectort
        time_iter = 0;
        ts_output.set(0);
        while(ts_output.get() < ts.get() + ms)
        {
            output_results_after << ts_output.get() << "\t" << problem->getState(ts_output).transpose() << std::endl;
            time_iter++;
            ts_output.set(time_iter * ms);
        }

        //finally, output the timestamp, state and stdev associated to KFs
        #ifdef ADD_KF3
            checking << KF2->getTimeStamp().get() << "\t" << KF2->getState().transpose() << "\t" << stdev_KF2.transpose() << std::endl;
            checking << KF3->getTimeStamp().get() << "\t" << KF3->getState().transpose() << "\t" << stdev_KF3.transpose() << std::endl;
        #else
            checking << KF2->getTimeStamp().get() << "\t" << KF2->getState().transpose() << "\t" << stdev_KF2.transpose() << std::endl;
        #endif
    #endif
    
    // ___Are expected values in the range of estimated +/- 2*stdev ?___

    #ifdef OUTPUT_RESULTS
        output_results_before.close();
        output_results_after.close();
        checking.close();
    #endif

    return 0;
}

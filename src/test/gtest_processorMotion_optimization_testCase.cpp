/**
 * \file gtest_ceres.cpp
 *
 *  Created on: Jan 18, 2017
 *      \author: Dinesh Atchuthan
 */


#include "utils_gtest.h"

#include "wolf.h"
#include "logging.h"

#include "processor_odom_3D.h"
#include "processor_imu.h"
#include "wolf.h"
#include "problem.h"
#include "ceres_wrapper/ceres_manager.h"
#include "state_quaternion.h"
#include "sensor_imu.h"
#include "rotations.h"

// wolf yaml
#include "../yaml/yaml_conversion.h"
// yaml-cpp library
#include <../yaml-cpp/yaml.h>

#include <iostream>
#include <fstream>

using namespace Eigen;
using namespace std;
using namespace wolf;

//Global variables

//used in pure_translation test
char * filename_motion_imu_data;
char * filename_motion_odom;
unsigned int number_of_KF = 2; //determine the number of final keyframes that will be created (except origin, so n>=1) in some of processorIMU tests

class ProcessorIMU_Odom_tests : public testing::Test
{
    public:
        wolf::TimeStamp t;
        Eigen::Vector6s data_;
        wolf::Scalar dt;
        SensorIMUPtr sen_imu;
        SensorOdom3DPtr sen_odom3D;
        ProblemPtr wolf_problem_ptr_;
        CeresManager* ceres_manager_wolf_diff;
        ProcessorBasePtr processor_ptr_;
        ProcessorIMUPtr processor_ptr_imu;
        ProcessorOdom3DPtr processor_ptr_odom3D;


    virtual void SetUp()
    {
        using std::shared_ptr;
        using std::make_shared;
        using std::static_pointer_cast;

        //===================================================== SETTING PROBLEM
        std::string wolf_root = _WOLF_ROOT_DIR;
        ASSERT_TRUE(number_of_KF>0) << "number_of_KF (number of Keyframe created) must be int >0";

        // WOLF PROBLEM
        wolf_problem_ptr_ = Problem::create(FRM_PQVBB_3D);
        Eigen::VectorXs x0(16);
        x0 << 0,0,0,  0,0,0,1,  0,0,0,  0,0,.00,  0,0,.00;
        t.set(0);

        // CERES WRAPPER
        ceres::Solver::Options ceres_options;
        ceres_options.minimizer_type = ceres::TRUST_REGION; //ceres::TRUST_REGION;ceres::LINE_SEARCH
        ceres_options.max_line_search_step_contraction = 1e-3;
        ceres_options.max_num_iterations = 1e4;
        ceres_manager_wolf_diff = new CeresManager(wolf_problem_ptr_, ceres_options, true);


        // SENSOR + PROCESSOR IMU
        //We want a processorIMU with a specific max_time_span (1s) forour test
        SensorBasePtr sen0_ptr = wolf_problem_ptr_->installSensor("IMU", "Main IMU", (Vector7s()<<0,0,0,0,0,0,1).finished(), wolf_root + "/src/examples/sensor_imu.yaml");
        ProcessorIMUParamsPtr prc_imu_params = std::make_shared<ProcessorIMUParams>();
        prc_imu_params->max_time_span = 1;
        prc_imu_params->max_buff_length = 1000000000; //make it very high so that this condition will not pass
        prc_imu_params->dist_traveled = 1000000000;
        prc_imu_params->angle_turned = 1000000000;

        processor_ptr_ = wolf_problem_ptr_->installProcessor("IMU", "IMU pre-integrator", sen0_ptr, prc_imu_params);
        sen_imu = std::static_pointer_cast<SensorIMU>(sen0_ptr);
        processor_ptr_imu = std::static_pointer_cast<ProcessorIMU>(processor_ptr_);


        // SENSOR + PROCESSOR ODOM 3D
        SensorBasePtr sen1_ptr = wolf_problem_ptr_->installSensor("ODOM 3D", "odom", (Vector7s()<<0,0,0,0,0,0,1).finished(), wolf_root + "/src/examples/sensor_odom_3D.yaml");
        ProcessorOdom3DParamsPtr prc_odom3D_params = std::make_shared<ProcessorOdom3DParams>();
        prc_odom3D_params->max_time_span = 1;
        prc_odom3D_params->max_buff_length = 1000000000; //make it very high so that this condition will not pass
        prc_odom3D_params->dist_traveled = 1000000000;
        prc_odom3D_params->angle_turned = 1000000000;

        ProcessorBasePtr processor_ptr_odom = wolf_problem_ptr_->installProcessor("ODOM 3D", "odom", sen1_ptr, prc_odom3D_params);
        sen_odom3D = std::static_pointer_cast<SensorOdom3D>(sen1_ptr);
        processor_ptr_odom3D = std::static_pointer_cast<ProcessorOdom3D>(processor_ptr_odom);

        //ORIGIN MUST BE SET IN THE TEST

    //===================================================== END{SETTING PROBLEM}
    }

    virtual void TearDown()
    {
        // code here will be called just after the test completes
        // ok to through exceptions from here if need be
        /*
            You can do deallocation of resources in TearDown or the destructor routine. 
                However, if you want exception handling you must do it only in the TearDown code because throwing an exception 
                from the destructor results in undefined behavior.
            The Google assertion macros may throw exceptions in platforms where they are enabled in future releases. 
                Therefore, it's a good idea to use assertion macros in the TearDown code for better maintenance.
        */
    }
};

class ProcessorIMU_Odom_tests_details : public testing::Test
{
    /* In this scenario, we simulate the integration of a perfect IMU that is not moving and we add an odometry and IMU constraint.
     * 
     * Finally, we can represent the graph as :
     *
     *  KF0 ---- constraintIMU ---- KF1
     *     \____constraintOdom3D___/
     */

    public:

        ProblemPtr wolf_problem_ptr_;
        CeresManager* ceres_manager_wolf_diff;
        Eigen::VectorXs initial_origin_state;
        Eigen::VectorXs initial_final_state;
        FrameIMUPtr origin_KF;
        FrameIMUPtr last_KF;
        std::vector<StateBlockPtr> originStateBlock_vec;
        std::vector<StateBlockPtr> finalStateBlock_vec;
        std::vector<StateBlockPtr> allStateBlocks;
        Eigen::VectorXs perturbated_origin_state;
        Eigen::VectorXs perturbated_final_state;
        ceres::Solver::Summary summary;

    virtual void SetUp()
    {
        using std::shared_ptr;
        using std::make_shared;
        using std::static_pointer_cast;

        //===================================================== SETTING PROBLEM
        std::string wolf_root = _WOLF_ROOT_DIR;
        ASSERT_TRUE(number_of_KF>0) << "number_of_KF (number of Keyframe created) must be int >0";

        // WOLF PROBLEM
        wolf_problem_ptr_ = Problem::create(FRM_PQVBB_3D);
        Eigen::VectorXs x0(16);
        x0 << 0,0,0,  0,0,0,1,  0,0,0,  0,0,.00,  0,0,.00;
        TimeStamp t(0);

        // CERES WRAPPER
        ceres::Solver::Options ceres_options;
        ceres_options.minimizer_type = ceres::TRUST_REGION; //ceres::TRUST_REGION;ceres::LINE_SEARCH
        ceres_options.max_line_search_step_contraction = 1e-3;
        ceres_options.max_num_iterations = 1e4;
        ceres_manager_wolf_diff = new CeresManager(wolf_problem_ptr_, ceres_options, true);


        // SENSOR + PROCESSOR IMU
        //We want a processorIMU with a specific max_time_span (1s) forour test
        SensorBasePtr sen0_ptr = wolf_problem_ptr_->installSensor("IMU", "Main IMU", (Vector7s()<<0,0,0,0,0,0,1).finished(), wolf_root + "/src/examples/sensor_imu.yaml");
        ProcessorIMUParamsPtr prc_imu_params = std::make_shared<ProcessorIMUParams>();
        prc_imu_params->max_time_span = 1;
        prc_imu_params->max_buff_length = 1000000000; //make it very high so that this condition will not pass
        prc_imu_params->dist_traveled = 1000000000;
        prc_imu_params->angle_turned = 1000000000;
        prc_imu_params->voting_active = true;

        ProcessorBasePtr processor_ptr_ = wolf_problem_ptr_->installProcessor("IMU", "IMU pre-integrator", sen0_ptr, prc_imu_params);
        SensorIMUPtr sen_imu = std::static_pointer_cast<SensorIMU>(sen0_ptr);
        ProcessorIMUPtr processor_ptr_imu = std::static_pointer_cast<ProcessorIMU>(processor_ptr_);


        // SENSOR + PROCESSOR ODOM 3D
        SensorBasePtr sen1_ptr = wolf_problem_ptr_->installSensor("ODOM 3D", "odom", (Vector7s()<<0,0,0,0,0,0,1).finished(), wolf_root + "/src/examples/sensor_odom_3D.yaml");
        ProcessorOdom3DParamsPtr prc_odom3D_params = std::make_shared<ProcessorOdom3DParams>();
        prc_odom3D_params->max_time_span = 1;
        prc_odom3D_params->max_buff_length = 1000000000; //make it very high so that this condition will not pass
        prc_odom3D_params->dist_traveled = 1000000000;
        prc_odom3D_params->angle_turned = 1000000000;

        ProcessorBasePtr processor_ptr_odom = wolf_problem_ptr_->installProcessor("ODOM 3D", "odom", sen1_ptr, prc_odom3D_params);
        SensorOdom3DPtr sen_odom3D = std::static_pointer_cast<SensorOdom3D>(sen1_ptr);
        ProcessorOdom3DPtr processor_ptr_odom3D = std::static_pointer_cast<ProcessorOdom3D>(processor_ptr_odom);

        //set processorMotions
        FrameBasePtr setOrigin_KF = processor_ptr_imu->setOrigin(x0, t);
        processor_ptr_odom3D->setOrigin(setOrigin_KF);

        wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->fix();
        //There should be 3 captures at origin_frame : CaptureOdom, captureIMU
        EXPECT_EQ((wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front())->getCaptureList().size(),2);
        ASSERT_TRUE(wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->isKey()) << "origin_frame is not a KeyFrame..." << std::endl;

    //===================================================== END{SETTING PROBLEM}

    //===================================================== PROCESS DATA
    // PROCESS IMU DATA

        Eigen::Vector6s data;
        data << 0.00, 0.000, -wolf::gravity()(2), 0.0, 0.0, 0.0;
        Scalar dt = t.get();
        TimeStamp ts(0.001);
        wolf::CaptureIMUPtr imu_ptr = std::make_shared<CaptureIMU>(ts, sen_imu, data);

        while( (dt-t.get()) < (std::static_pointer_cast<ProcessorIMU>(processor_ptr_)->getMaxTimeSpan()) ){
        
            // Time and data variables
            dt += 0.001;
            ts.set(dt);
            imu_ptr->setTimeStamp(ts);
        	imu_ptr->setData(data);

            // process data in capture
            imu_ptr->getTimeStamp();
            sen_imu->process(imu_ptr);
        }

        // PROCESS ODOM 3D DATA
        Eigen::Vector6s data_odom3D;
        data_odom3D << 0,0,0, 0,0,0;
    
        wolf::CaptureMotionPtr mot_ptr = std::make_shared<CaptureMotion>(ts, sen_odom3D, data_odom3D);
        sen_odom3D->process(mot_ptr);

        //===================================================== END{PROCESS DATA}

        //===================================================== TESTS PREPARATION

        origin_KF = std::static_pointer_cast<FrameIMU>(wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front());
        last_KF = std::static_pointer_cast<FrameIMU>(wolf_problem_ptr_->getTrajectoryPtr()->closestKeyFrameToTimeStamp(ts));

        initial_origin_state.resize(16);
        initial_final_state.resize(16);
        perturbated_origin_state.resize(16);
        perturbated_final_state.resize(16);

        //store states before optimization so that we can reset the frames to their original state for other optimization tests
        origin_KF->getState(initial_origin_state);
        last_KF->getState(initial_final_state);

        //get stateblocks from origin and last KF and concatenate them in one std::vector to unfix stateblocks
        originStateBlock_vec = origin_KF->getStateBlockVec();
        finalStateBlock_vec = last_KF->getStateBlockVec();

        allStateBlocks.reserve(originStateBlock_vec.size() + finalStateBlock_vec.size());
        allStateBlocks.insert( allStateBlocks.end(), originStateBlock_vec.begin(), originStateBlock_vec.end() );
        allStateBlocks.insert( allStateBlocks.end(), finalStateBlock_vec.begin(), finalStateBlock_vec.end() );

        //===================================================== END{TESTS PREPARATION}
    }

    virtual void TearDown(){}
};


class ProcessorIMU_Odom_tests_details3KF : public testing::Test
{
    /* In this scenario, we simulate the integration of a perfect IMU that is not moving and we add an odometry and IMU constraint.
     * 
     * Finally, we can represent the graph as :
     *
     *  KF0 ---- constraintIMU ---- KF1
     *     \____constraintOdom3D___/
     */

    public:

        ProblemPtr wolf_problem_ptr_;
        CeresManager* ceres_manager_wolf_diff;
        Eigen::VectorXs initial_origin_state;
        Eigen::VectorXs initial_final_state;
        Eigen::VectorXs initial_middle_state;
        FrameIMUPtr origin_KF;
        FrameIMUPtr middle_KF;
        FrameIMUPtr last_KF;
        std::vector<StateBlockPtr> originStateBlock_vec;
        std::vector<StateBlockPtr> middleStateBlock_vec;
        std::vector<StateBlockPtr> finalStateBlock_vec;
        std::vector<StateBlockPtr> allStateBlocks;
        Eigen::VectorXs perturbated_origin_state;
        Eigen::VectorXs perturbated_middle_state;
        Eigen::VectorXs perturbated_final_state;
        ceres::Solver::Summary summary;

    virtual void SetUp()
    {
        using std::shared_ptr;
        using std::make_shared;
        using std::static_pointer_cast;

        //===================================================== SETTING PROBLEM
        std::string wolf_root = _WOLF_ROOT_DIR;
        ASSERT_TRUE(number_of_KF>0) << "number_of_KF (number of Keyframe created) must be int >0";

        // WOLF PROBLEM
        wolf_problem_ptr_ = Problem::create(FRM_PQVBB_3D);
        Eigen::VectorXs x0(16);
        x0 << 0,0,0,  0,0,0,1,  0,0,0,  0,0,.00,  0,0,.00;
        TimeStamp t(0);

        // CERES WRAPPER
        ceres::Solver::Options ceres_options;
        ceres_options.minimizer_type = ceres::TRUST_REGION; //ceres::TRUST_REGION;ceres::LINE_SEARCH
        ceres_options.max_line_search_step_contraction = 1e-3;
        ceres_options.max_num_iterations = 1e4;
        ceres_manager_wolf_diff = new CeresManager(wolf_problem_ptr_, ceres_options, true);


        // SENSOR + PROCESSOR IMU
        //We want a processorIMU with a specific max_time_span (1s) forour test
        SensorBasePtr sen0_ptr = wolf_problem_ptr_->installSensor("IMU", "Main IMU", (Vector7s()<<0,0,0,0,0,0,1).finished(), wolf_root + "/src/examples/sensor_imu.yaml");
        ProcessorIMUParamsPtr prc_imu_params = std::make_shared<ProcessorIMUParams>();
        prc_imu_params->max_time_span = 1;
        prc_imu_params->max_buff_length = 1000000000; //make it very high so that this condition will not pass
        prc_imu_params->dist_traveled = 1000000000;
        prc_imu_params->angle_turned = 1000000000;
        prc_imu_params->voting_active = true;

        ProcessorBasePtr processor_ptr_ = wolf_problem_ptr_->installProcessor("IMU", "IMU pre-integrator", sen0_ptr, prc_imu_params);
        SensorIMUPtr sen_imu = std::static_pointer_cast<SensorIMU>(sen0_ptr);
        ProcessorIMUPtr processor_ptr_imu = std::static_pointer_cast<ProcessorIMU>(processor_ptr_);


        // SENSOR + PROCESSOR ODOM 3D
        SensorBasePtr sen1_ptr = wolf_problem_ptr_->installSensor("ODOM 3D", "odom", (Vector7s()<<0,0,0,0,0,0,1).finished(), wolf_root + "/src/examples/sensor_odom_3D.yaml");
        ProcessorOdom3DParamsPtr prc_odom3D_params = std::make_shared<ProcessorOdom3DParams>();
        prc_odom3D_params->max_time_span = 1;
        prc_odom3D_params->max_buff_length = 1000000000; //make it very high so that this condition will not pass
        prc_odom3D_params->dist_traveled = 1000000000;
        prc_odom3D_params->angle_turned = 1000000000;

        ProcessorBasePtr processor_ptr_odom = wolf_problem_ptr_->installProcessor("ODOM 3D", "odom", sen1_ptr, prc_odom3D_params);
        SensorOdom3DPtr sen_odom3D = std::static_pointer_cast<SensorOdom3D>(sen1_ptr);
        ProcessorOdom3DPtr processor_ptr_odom3D = std::static_pointer_cast<ProcessorOdom3D>(processor_ptr_odom);

        //set processorMotions
        FrameBasePtr setOrigin_KF = processor_ptr_imu->setOrigin(x0, t);
        processor_ptr_odom3D->setOrigin(setOrigin_KF);

        wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->fix();
        //There should be 3 captures at origin_frame : CaptureOdom, captureIMU
        EXPECT_EQ((wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front())->getCaptureList().size(),2);
        ASSERT_TRUE(wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->isKey()) << "origin_frame is not a KeyFrame..." << std::endl;

    //===================================================== END{SETTING PROBLEM}

    //===================================================== PROCESS DATA
    // PROCESS IMU DATA

        Eigen::Vector6s data;
        data << 0.00, 0.000, -wolf::gravity()(2), 0.0, 0.0, 0.0;
        Scalar dt = t.get();
        TimeStamp ts(0.001);
        wolf::CaptureIMUPtr imu_ptr = std::make_shared<CaptureIMU>(ts, sen_imu, data);

        while( (dt-t.get()) < (std::static_pointer_cast<ProcessorIMU>(processor_ptr_)->getMaxTimeSpan()*2) ){
        
            // Time and data variables
            dt += 0.001;
            ts.set(dt);
            imu_ptr->setTimeStamp(ts);
        	imu_ptr->setData(data);

            // process data in capture
            imu_ptr->getTimeStamp();
            sen_imu->process(imu_ptr);

            if(ts.get() == 1 || ts.get() == 2)
            {
                // PROCESS ODOM 3D DATA
                Eigen::Vector6s data_odom3D;
                data_odom3D << 0,0,0, 0,0,0;
    
                wolf::CaptureMotionPtr mot_ptr = std::make_shared<CaptureMotion>(ts, sen_odom3D, data_odom3D);
                sen_odom3D->process(mot_ptr);
            }
        }

        //===================================================== END{PROCESS DATA}

        //===================================================== TESTS PREPARATION
        wolf::TimeStamp middle_ts(1);

        origin_KF = std::static_pointer_cast<FrameIMU>(wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front());
        middle_KF = std::static_pointer_cast<FrameIMU>(wolf_problem_ptr_->getTrajectoryPtr()->closestKeyFrameToTimeStamp(middle_ts));
        last_KF = std::static_pointer_cast<FrameIMU>(wolf_problem_ptr_->getTrajectoryPtr()->closestKeyFrameToTimeStamp(ts));

        initial_origin_state.resize(16);
        initial_middle_state.resize(16);
        initial_final_state.resize(16);
        perturbated_origin_state.resize(16);
        perturbated_middle_state.resize(16);
        perturbated_final_state.resize(16);

        //store states before optimization so that we can reset the frames to their original state for other optimization tests
        origin_KF->getState(initial_origin_state);
        middle_KF->getState(initial_middle_state);
        last_KF->getState(initial_final_state);

        //get stateblocks from origin and last KF and concatenate them in one std::vector to unfix stateblocks
        originStateBlock_vec = origin_KF->getStateBlockVec();
        middleStateBlock_vec = middle_KF->getStateBlockVec();
        finalStateBlock_vec = last_KF->getStateBlockVec();

        allStateBlocks.reserve(originStateBlock_vec.size() + finalStateBlock_vec.size() + middleStateBlock_vec.size());
        allStateBlocks.insert( allStateBlocks.end(), originStateBlock_vec.begin(), originStateBlock_vec.end() );
        allStateBlocks.insert( allStateBlocks.end(), middleStateBlock_vec.begin(), middleStateBlock_vec.end() );
        allStateBlocks.insert( allStateBlocks.end(), finalStateBlock_vec.begin(), finalStateBlock_vec.end() );

        //===================================================== END{TESTS PREPARATION}
    }

    virtual void TearDown(){}
};


class ProcessorIMU_Odom_tests_plateform_simulation : public testing::Test
{
    public:
        wolf::TimeStamp t;
        Eigen::Vector6s data_;
        wolf::Scalar dt;
        SensorIMUPtr sen_imu;
        SensorOdom3DPtr sen_odom3D;
        ProblemPtr wolf_problem_ptr_;
        CeresManager* ceres_manager_wolf_diff;
        ProcessorBasePtr processor_ptr_;
        ProcessorIMUPtr processor_ptr_imu;
        ProcessorOdom3DPtr processor_ptr_odom3D;
        FrameIMUPtr origin_KF;
        FrameIMUPtr last_KF;
        Eigen::Matrix<wolf::Scalar, 10, 1> expected_final_state;



    virtual void SetUp()
    {
        using std::shared_ptr;
        using std::make_shared;
        using std::static_pointer_cast;

        //===================================================== SETTING PROBLEM
        std::string wolf_root = _WOLF_ROOT_DIR;
        ASSERT_TRUE(number_of_KF>0) << "number_of_KF (number of Keyframe created) must be int >0";

        // WOLF PROBLEM
        wolf_problem_ptr_ = Problem::create(FRM_PQVBB_3D);
        Eigen::VectorXs x0(16);
        x0 << 0,0,0,  0,0,0,1,  0,0,0,  0,0,.00,  0,0,.00;
        t.set(0);

        // CERES WRAPPER
        ceres::Solver::Options ceres_options;
        ceres_options.minimizer_type = ceres::TRUST_REGION; //ceres::TRUST_REGION;ceres::LINE_SEARCH
        ceres_options.max_line_search_step_contraction = 1e-3;
        ceres_options.max_num_iterations = 1e4;
        ceres_manager_wolf_diff = new CeresManager(wolf_problem_ptr_, ceres_options, true);


        // SENSOR + PROCESSOR IMU
        //We want a processorIMU with a specific max_time_span (1s) forour test
        SensorBasePtr sen0_ptr = wolf_problem_ptr_->installSensor("IMU", "Main IMU", (Vector7s()<<0,0,0,0,0,0,1).finished(), wolf_root + "/src/examples/sensor_imu.yaml");
        ProcessorIMUParamsPtr prc_imu_params = std::make_shared<ProcessorIMUParams>();
        prc_imu_params->max_time_span = 1;
        prc_imu_params->max_buff_length = 1000000000; //make it very high so that this condition will not pass
        prc_imu_params->dist_traveled = 1000000000;
        prc_imu_params->angle_turned = 1000000000;

        processor_ptr_ = wolf_problem_ptr_->installProcessor("IMU", "IMU pre-integrator", sen0_ptr, prc_imu_params);
        sen_imu = std::static_pointer_cast<SensorIMU>(sen0_ptr);
        processor_ptr_imu = std::static_pointer_cast<ProcessorIMU>(processor_ptr_);


        // SENSOR + PROCESSOR ODOM 3D
        SensorBasePtr sen1_ptr = wolf_problem_ptr_->installSensor("ODOM 3D", "odom", (Vector7s()<<0,0,0,0,0,0,1).finished(), wolf_root + "/src/examples/sensor_odom_3D.yaml");
        ProcessorOdom3DParamsPtr prc_odom3D_params = std::make_shared<ProcessorOdom3DParams>();
        prc_odom3D_params->max_time_span = 0.99;
        prc_odom3D_params->max_buff_length = 1000000000; //make it very high so that this condition will not pass
        prc_odom3D_params->dist_traveled = 1000000000;
        prc_odom3D_params->angle_turned = 1000000000;

        ProcessorBasePtr processor_ptr_odom = wolf_problem_ptr_->installProcessor("ODOM 3D", "odom", sen1_ptr, prc_odom3D_params);
        sen_odom3D = std::static_pointer_cast<SensorOdom3D>(sen1_ptr);
        processor_ptr_odom3D = std::static_pointer_cast<ProcessorOdom3D>(processor_ptr_odom);

        //ORIGIN MUST BE SET IN THE TEST

    //===================================================== END{SETTING PROBLEM}

        char* imu_filepath;
        char* odom_filepath;
        //std::string imu_filepath_string(wolf_root + "/src/test/data/IMU/Test_plateforme/data_pure_translation.txt");
        //std::string odom_filepath_string(wolf_root + "/src/test/data/IMU/Test_plateforme/odom_pure_translation.txt");
        //std::string imu_filepath_string(wolf_root + "/src/test/data/IMU/Test_plateforme/data_trajectory_full.txt");
        //std::string odom_filepath_string(wolf_root + "/src/test/data/IMU/Test_plateforme/odom_trajectory_full.txt");
        std::string imu_filepath_string(wolf_root + "/src/test/data/IMU/Test_plateforme/data_pure_rotation.txt");
        std::string odom_filepath_string(wolf_root + "/src/test/data/IMU/Test_plateforme/odom_pure_rotation.txt");
        imu_filepath   = new char[imu_filepath_string.length() + 1];
        odom_filepath   = new char[odom_filepath_string.length() + 1];
        std::strcpy(imu_filepath, imu_filepath_string.c_str());
        std::strcpy(odom_filepath, odom_filepath_string.c_str());
        std::ifstream imu_data_input;
        std::ifstream odom_data_input;

        imu_data_input.open(imu_filepath);
        odom_data_input.open(odom_filepath);
        WOLF_INFO("imu file: ", imu_filepath)
        if(!imu_data_input.is_open() | !odom_data_input.is_open()){
            std::cerr << "Failed to open data files... Exiting" << std::endl;
            ADD_FAILURE();
        }

        //===================================================== SETTING PROBLEM

        // reset origin of problem
        Eigen::VectorXs x_origin((Eigen::Matrix<wolf::Scalar,16,1>()<<0,0,0, 0,0,0,1, 0,0,0, 0,0,0, 0,0,0).finished());

        // initial conditions defined from data file
        // remember that matlab's quaternion is W,X,Y,Z and the one in Eigen has X,Y,Z,W form
        imu_data_input >> x_origin[0] >> x_origin[1] >> x_origin[2] >> x_origin[6] >> x_origin[3] >> x_origin[4] >> x_origin[5] >> x_origin[7] >> x_origin[8] >> x_origin[9];

        t.set(0);
        origin_KF = std::static_pointer_cast<FrameIMU>(processor_ptr_imu->setOrigin(x_origin, t));
        processor_ptr_odom3D->setOrigin(origin_KF);

        odom_data_input >> expected_final_state[0] >> expected_final_state[1] >> expected_final_state[2] >> expected_final_state[6] >> expected_final_state[3] >>
                             expected_final_state[4] >> expected_final_state[5] >> expected_final_state[7] >> expected_final_state[8] >> expected_final_state[9];
    
    //===================================================== END{SETTING PROBLEM}

    //===================================================== PROCESS DATA
    // PROCESS DATA

        Eigen::Vector6s data_imu, data_odom3D;
        data_imu << 0,0,-wolf::gravity()(2), 0,0,0;
        data_odom3D << 0,0,0, 0,0,0;

        Scalar input_clock;
        TimeStamp ts(0);
        TimeStamp t_odom(0);
        wolf::CaptureIMUPtr imu_ptr = std::make_shared<CaptureIMU>(ts, sen_imu, data_imu);
        wolf::CaptureMotionPtr mot_ptr = std::make_shared<CaptureMotion>(t, sen_odom3D, data_odom3D);
    
        //read first odom data from file
        odom_data_input >> input_clock >> data_odom3D[0] >> data_odom3D[1] >> data_odom3D[2] >> data_odom3D[3] >> data_odom3D[4] >> data_odom3D[5];
        t_odom.set(input_clock);
        //when we find a IMU timestamp corresponding with this odometry timestamp then we process odometry measurement

        while( !imu_data_input.eof() && !odom_data_input.eof() )
        {
            // PROCESS IMU DATA
            // Time and data variables
            imu_data_input >> input_clock >> data_imu[0] >> data_imu[1] >> data_imu[2] >> data_imu[3] >> data_imu[4] >> data_imu[5]; //Ax, Ay, Az, Gx, Gy, Gz

            ts.set(input_clock);
            imu_ptr->setTimeStamp(ts);
            imu_ptr->setData(data_imu);

            // process data in capture
            imu_ptr->getTimeStamp();
            sen_imu->process(imu_ptr);

            if(ts.get() == t_odom.get()) //every 100 ms
            {
                // PROCESS ODOM 3D DATA
                mot_ptr->setTimeStamp(t_odom);
                mot_ptr->setData(data_odom3D);
                sen_odom3D->process(mot_ptr);

                //prepare next odometry measurement if there is any
                odom_data_input >> input_clock >> data_odom3D[0] >> data_odom3D[1] >> data_odom3D[2] >> data_odom3D[3] >> data_odom3D[4] >> data_odom3D[5];
                t_odom.set(input_clock);
            }
        }

        last_KF = std::static_pointer_cast<FrameIMU>(wolf_problem_ptr_->getTrajectoryPtr()->closestKeyFrameToTimeStamp(ts));

        //closing file
        imu_data_input.close();
        odom_data_input.close();

    //===================================================== END{PROCESS DATA}
    }

    virtual void TearDown(){}
};


TEST(ProcessorOdom3D, static_ceresOptimisation_Odom_PO)
{

    /* Simple odom test including only processorOdom3D :
     * First keyFrame (origin) is fixed (0,0,0, 0,0,0,1). Then the odometry data for 2 second is [0,0,0, 0,0,0,1], meaning that we did not move at all. 
     * We give this simple graph to solver and let it solve.
     * Everything should converge and final keyFrame State should be exactly [0,0,0, 0,0,0,1]
     */

    using std::shared_ptr;
    using std::make_shared;
    using std::static_pointer_cast;
                                            /************** SETTING PROBLEM  **************/

    std::string wolf_root = _WOLF_ROOT_DIR;

    // Wolf problem
    ProblemPtr wolf_problem_ptr_ = Problem::create(FRM_PO_3D);

    SensorBasePtr sen = wolf_problem_ptr_->installSensor("ODOM 3D", "odom", (Vector7s()<<0,0,0,0,0,0,1).finished(), wolf_root + "/src/examples/sensor_odom_3D.yaml");

    // We want to create a processorMotion with a max_time_span of 2 seconds. but here we integrate only odometry and there should be no interpolation between
    // Default processorMotionParams is made so that a KeyFrame will be created at each step. This works in this case
    ProcessorOdom3DParamsPtr prc_odom_params = std::make_shared<ProcessorOdom3DParams>();
    wolf_problem_ptr_->installProcessor("ODOM 3D", "odometry integrator", sen, prc_odom_params);
    wolf_problem_ptr_->getProcessorMotionPtr()->setOrigin((Vector7s()<<0,0,0,0,0,0,1).finished(), TimeStamp(0));

    // Ceres wrappers
    ceres::Solver::Options ceres_options;
    ceres_options.minimizer_type = ceres::TRUST_REGION; //ceres::TRUST_REGION;ceres::LINE_SEARCH
    ceres_options.max_line_search_step_contraction = 1e-3;
    ceres_options.max_num_iterations = 1e4;
    CeresManager* ceres_manager_wolf_diff = new CeresManager(wolf_problem_ptr_, ceres_options, true);

                                             /************** USE ODOM_3D CLASSES  **************/

    VectorXs d(7);
    d << 0,0,0, 0,0,0,1;
    TimeStamp t(2);

    wolf::CaptureMotionPtr odom_ptr = std::make_shared<CaptureMotion>(t, sen, d);
    wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->fix();
    // process data in capture
    sen->process(odom_ptr);

    /* We do not need to create features and frames and constraints here. Everything is done in wolf.
    Features and constraint at created automatically when a new Keyframe is generated. Whether a new keyframe should be created or not, this is
    handled by voteForKeyFrame() function for this processorMotion
    */

    if(wolf_problem_ptr_->check(1)){
        wolf_problem_ptr_->print(4,1,1,1);
    }

                                             /************** SOLVER PART  **************/
     ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
     std::cout << summary.FullReport() << std::endl;

     //There should be 3 frames : origin KeyFrame, Generated KeyFrame at t = 2s, and another Frame for incoming data to be processed
     ASSERT_EQ(wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().size(),3);
     
     //This is a static test so we are not supposed to have moved from origin to last KeyFrame
     ASSERT_TRUE( (wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->getPPtr()->getVector() - wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().back()->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL) ) <<
                "origin and final frame position are different" << std::endl;
     ASSERT_TRUE( (wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->getOPtr()->getVector() - wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().back()->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL) ) <<
                "origin and final frame orientation are different" << std::endl;
     EXPECT_TRUE(wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->isFixed()) << "origin_frame is not fixed" << std::endl;

    // COMPUTE COVARIANCES
    std::cout << "\t\t\t ______computing covariances______" << std::endl;
    ceres_manager_wolf_diff->computeCovariances(ALL_MARGINALS);//ALL_MARGINALS, ALL
    std::cout << "\t\t\t ______computed!______" << std::endl;
}

TEST(ProcessorOdom3D, static_ceresOptimisation_convergenceOdom_PO)
{

    /* Simple odom test including only processorOdom3D :
     * First keyFrame (origin) is fixed (0,0,0, 0,0,0,1). Then the odometry data for 2 second is [0,0,0, 0,0,0,1], meaning that we did not move at all. 
     * We give this simple graph to solver and let it solve. 
     *
     * But before solving, we change the state of final KeyFrame.
     * First we change only Px, then Py, then Pz, then all of them
     * Second : we change Ox, then Oy, then Oz, then all of them
     * Third : we change everything
     * Everything should converge and final keyFrame State should be exactly [0,0,0, 0,0,0,1]
     */

    using std::shared_ptr;
    using std::make_shared;
    using std::static_pointer_cast;
                                            /************** SETTING PROBLEM  **************/

    std::string wolf_root = _WOLF_ROOT_DIR;

    // Wolf problem
    ProblemPtr wolf_problem_ptr_ = Problem::create(FRM_PO_3D);

    SensorBasePtr sen = wolf_problem_ptr_->installSensor("ODOM 3D", "odom", (Vector7s()<<0,0,0,0,0,0,1).finished(), wolf_root + "/src/examples/sensor_odom_3D.yaml");

    // We want to create a processorMotion with a max_time_span of 2 seconds. but here we integrate only odometry and there should be no interpolation between
    // Default processorMotionParams is made so that a KeyFrame will be created at each step. This works in this case
    ProcessorOdom3DParamsPtr prc_odom_params = std::make_shared<ProcessorOdom3DParams>();
    wolf_problem_ptr_->installProcessor("ODOM 3D", "odometry integrator", sen, prc_odom_params);
    wolf_problem_ptr_->getProcessorMotionPtr()->setOrigin((Vector7s()<<0,0,0,0,0,0,1).finished(), TimeStamp(0));

    // Ceres wrappers
    ceres::Solver::Options ceres_options;
    ceres_options.minimizer_type = ceres::TRUST_REGION; //ceres::TRUST_REGION;ceres::LINE_SEARCH
    ceres_options.max_line_search_step_contraction = 1e-3;
    ceres_options.max_num_iterations = 1e4;
    CeresManager* ceres_manager_wolf_diff = new CeresManager(wolf_problem_ptr_, ceres_options, true);

                                             /************** USE ODOM_3D CLASSES  **************/

    VectorXs d(7);
    d << 0,0,0, 0,0,0,1;
    TimeStamp t(2);

    wolf::CaptureMotionPtr odom_ptr = std::make_shared<CaptureMotion>(t, sen, d);
    wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->fix();
    // process data in capture
    sen->process(odom_ptr);

    if(wolf_problem_ptr_->check(1)){
        wolf_problem_ptr_->print(4,1,1,1);
    }

                                             /************** SOLVER PART  **************/

     /* ___________________________________________ CHANGING FINAL FRAME BEFORE OPTIMIZATION ___________________________________________*/
    
    //There should be 3 frames : origin KeyFrame, Generated KeyFrame at t = 2s, and another Frame for incoming data to be processed
    ASSERT_EQ(wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().size(),3);
    EXPECT_TRUE(wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->isFixed()) << "origin_frame is not fixed" << std::endl;

    //get pointer to the last KeyFrame (which is at t = 2s)
    EXPECT_EQ(t.get(),2);
    FrameBasePtr last_KF = wolf_problem_ptr_->getTrajectoryPtr()->closestKeyFrameToTimeStamp(t);

    // FIRST SOLVER TEST WITHOUT CHANGING ANYTHING - WE DID NOT MOVE

    std::cout << "______ solving...______" << std::endl;
    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
    std::cout << summary.BriefReport() << std::endl;
    std::cout << "______ solved !______" << std::endl;

    // COMPUTE COVARIANCES
    //std::cout << "\t\t\t ______computing covariances______" << std::endl;
    ceres_manager_wolf_diff->computeCovariances(ALL_MARGINALS);//ALL_MARGINALS, ALL
    //std::cout << "\t\t\t ______computed!______" << std::endl;


    //This is a static test so we are not supposed to have moved from origin to last KeyFrame
    ASSERT_TRUE( (wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->getPPtr()->getVector() - wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().back()->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL) ) <<
                "origin and final frame position are different" << std::endl;
    ASSERT_TRUE( (wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->getOPtr()->getVector() - wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().back()->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL) ) <<
                "origin and final frame orientation are different" << std::endl;
    

                                                    /*********************/
                                                    //CHANGE PX AND SOLVE//
                                                    /*********************/

    last_KF->setState((Vector7s()<<30,0,0,0,0,0,1).finished());

    std::cout << "______ solving... Px changed______" << std::endl;
    summary = ceres_manager_wolf_diff->solve();
    std::cout << summary.BriefReport() << std::endl;
    std::cout << "______ solved ! Px changed______" << std::endl;

    //This is a static test so we are not supposed to have moved from origin to last KeyFrame
    ASSERT_TRUE( (wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->getPPtr()->getVector() - wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().back()->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL) ) <<
                "origin and final frame position are different - problem when Px is changed" << std::endl;
    ASSERT_TRUE( (wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->getOPtr()->getVector() - wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().back()->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL) ) <<
                "origin and final frame orientation are different - problem when Px is changed" << std::endl;

    // COMPUTE COVARIANCES
    //std::cout << "\t\t\t ______computing covariances______ Px changed" << std::endl;
    ceres_manager_wolf_diff->computeCovariances(ALL_MARGINALS);//ALL_MARGINALS, ALL
    //std::cout << "\t\t\t ______computed!______ Px changed" << std::endl;


                                                    /*********************/
                                                    //CHANGE PY AND SOLVE//
                                                    /*********************/

    last_KF->setState((Vector7s()<<0,30,0,0,0,0,1).finished());

    std::cout << "______ solving... Py changed______" << std::endl;
    summary = ceres_manager_wolf_diff->solve();
    std::cout << summary.BriefReport() << std::endl;
    std::cout << "______ solved ! Py changed______" << std::endl;

    //This is a static test so we are not supposed to have moved from origin to last KeyFrame
    ASSERT_TRUE( (wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->getPPtr()->getVector() - wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().back()->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL) ) <<
                "origin and final frame position are different - problem when Py is changed" << std::endl;
    ASSERT_TRUE( (wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->getOPtr()->getVector() - wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().back()->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL) ) <<
                "origin and final frame orientation are different - problem when Py is changed" << std::endl;

    // COMPUTE COVARIANCES
    //std::cout << "\t\t\t ______computing covariances______ Py changed" << std::endl;
    ceres_manager_wolf_diff->computeCovariances(ALL_MARGINALS);//ALL_MARGINALS, ALL
    //std::cout << "\t\t\t ______computed!______ Py changed" << std::endl;


                                                    /*********************/
                                                    //CHANGE PZ AND SOLVE//
                                                    /*********************/

    last_KF->setState((Vector7s()<<0,0,30,0,0,0,1).finished());

    std::cout << "______ solving... Pz changed______" << std::endl;
    summary = ceres_manager_wolf_diff->solve();
    std::cout << summary.BriefReport() << std::endl;
    std::cout << "______ solved ! Pz changed______" << std::endl;

    //This is a static test so we are not supposed to have moved from origin to last KeyFrame
    ASSERT_TRUE( (wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->getPPtr()->getVector() - wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().back()->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL) ) <<
                "origin and final frame position are different - problem when Pz is changed" << std::endl;
    ASSERT_TRUE( (wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->getOPtr()->getVector() - wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().back()->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL) ) <<
                "origin and final frame orientation are different - problem when Pz is changed" << std::endl;

    // COMPUTE COVARIANCES
    //std::cout << "\t\t\t ______computing covariances______ Pz changed" << std::endl;
    ceres_manager_wolf_diff->computeCovariances(ALL_MARGINALS);//ALL_MARGINALS, ALL
    //std::cout << "\t\t\t ______computed!______ Pz changed" << std::endl;


                                                    /********************************/
                                                    //CHANGE PX, Py AND PZ AND SOLVE//
                                                    /********************************/

    last_KF->setState((Vector7s()<<25,20,30,0,0,0,1).finished());

    std::cout << "______ solving... Px, Py and Pz changed______" << std::endl;
    summary = ceres_manager_wolf_diff->solve();
    std::cout << summary.BriefReport() << std::endl;
    std::cout << "______ solved ! Px, Py and Pz changed______" << std::endl;

    //This is a static test so we are not supposed to have moved from origin to last KeyFrame
    ASSERT_TRUE( (wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->getPPtr()->getVector() - wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().back()->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL) ) <<
                "origin and final frame position are different - problem when Px, Py and Pz are changed" << std::endl;
    ASSERT_TRUE( (wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->getOPtr()->getVector() - wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().back()->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL) ) <<
                "origin and final frame orientation are different - problem when Px, Py and Pz are changed" << std::endl;

    // COMPUTE COVARIANCES
    //std::cout << "\t\t\t ______computing covariances______ Px, Py and Pz changed" << std::endl;
    ceres_manager_wolf_diff->computeCovariances(ALL_MARGINALS);//ALL_MARGINALS, ALL
    //std::cout << "\t\t\t ______computed!______ Px, Py and Pz changed" << std::endl;

                                                    /*********************/
                                                    //CHANGE OX AND SOLVE//
                                                    /*********************/
    Eigen::Vector3s o_initial_guess;
    Eigen::Quaternions q_init_guess;

    o_initial_guess << 40,0,0;
    q_init_guess = v2q(o_initial_guess);
    last_KF->setState((Eigen::Matrix<wolf::Scalar,7,1>()<<0,0,0,q_init_guess.x(),q_init_guess.y(),q_init_guess.z(),q_init_guess.w()).finished());

    std::cout << "______ solving... Ox changed______" << std::endl;
    summary = ceres_manager_wolf_diff->solve();
    std::cout << summary.BriefReport() << std::endl;
    std::cout << "______ solved ! Ox changed______" << std::endl;

    //This is a static test so we are not supposed to have moved from origin to last KeyFrame
    ASSERT_TRUE( (wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->getPPtr()->getVector() - wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().back()->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL) ) <<
                "origin and final frame position are different - problem when Ox is changed" << std::endl;
    ASSERT_TRUE( (wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->getOPtr()->getVector() - wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().back()->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL) ) <<
                "origin and final frame orientation are different - problem when Ox is changed" << std::endl;

    // COMPUTE COVARIANCES
    //std::cout << "\t\t\t ______computing covariances______ Ox changed" << std::endl;
    ceres_manager_wolf_diff->computeCovariances(ALL_MARGINALS);//ALL_MARGINALS, ALL
    //std::cout << "\t\t\t ______computed!______ Ox changed" << std::endl;


                                                    /*********************/
                                                    //CHANGE OY AND SOLVE//
                                                    /*********************/
    o_initial_guess << 0,40,0;
    q_init_guess = v2q(o_initial_guess);
    last_KF->setState((Eigen::Matrix<wolf::Scalar,7,1>()<<0,0,0,q_init_guess.x(),q_init_guess.y(),q_init_guess.z(),q_init_guess.w()).finished());

    std::cout << "______ solving... Oy changed______" << std::endl;
    summary = ceres_manager_wolf_diff->solve();
    std::cout << summary.BriefReport() << std::endl;
    std::cout << "______ solved ! Oy changed______" << std::endl;

    //This is a static test so we are not supposed to have moved from origin to last KeyFrame
    ASSERT_TRUE( (wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->getPPtr()->getVector() - wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().back()->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL) ) <<
                "origin and final frame position are different - problem when Oy is changed" << std::endl;
    ASSERT_TRUE( (wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->getOPtr()->getVector() - wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().back()->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL) ) <<
                "origin and final frame orientation are different - problem when Oy is changed" << std::endl;

    // COMPUTE COVARIANCES
    //std::cout << "\t\t\t ______computing covariances______ Oy changed" << std::endl;
    ceres_manager_wolf_diff->computeCovariances(ALL_MARGINALS);//ALL_MARGINALS, ALL
    //std::cout << "\t\t\t ______computed!______ Oy changed" << std::endl;

                                                    /*********************/
                                                    //CHANGE OZ AND SOLVE//
                                                    /*********************/
    o_initial_guess << 0,0,40;
    q_init_guess = v2q(o_initial_guess);
    last_KF->setState((Eigen::Matrix<wolf::Scalar,7,1>()<<0,0,0,q_init_guess.x(),q_init_guess.y(),q_init_guess.z(),q_init_guess.w()).finished());

    std::cout << "______ solving... Oz changed______" << std::endl;
    summary = ceres_manager_wolf_diff->solve();
    std::cout << summary.BriefReport() << std::endl;
    std::cout << "______ solved ! Oz changed______" << std::endl;

    //This is a static test so we are not supposed to have moved from origin to last KeyFrame
    ASSERT_TRUE( (wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->getPPtr()->getVector() - wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().back()->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL) ) <<
                "origin and final frame position are different - problem when Oz is changed" << std::endl;
    ASSERT_TRUE( (wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->getOPtr()->getVector() - wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().back()->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL) ) <<
                "origin and final frame orientation are different - problem when Oz is changed" << std::endl;

    // COMPUTE COVARIANCES
    //std::cout << "\t\t\t ______computing covariances______ Oz changed" << std::endl;
    ceres_manager_wolf_diff->computeCovariances(ALL_MARGINALS);//ALL_MARGINALS, ALL
    //std::cout << "\t\t\t ______computed!______ Oz changed" << std::endl;


                                                    /********************************/
                                                    //CHANGE OX, OY AND OZ AND SOLVE//
                                                    /********************************/
    o_initial_guess << 80,50,40;
    q_init_guess = v2q(o_initial_guess);
    last_KF->setState((Eigen::Matrix<wolf::Scalar,7,1>()<<0,0,0,q_init_guess.x(),q_init_guess.y(),q_init_guess.z(),q_init_guess.w()).finished());

    std::cout << "______ solving... Ox, Oy and Oz changed______" << std::endl;
    summary = ceres_manager_wolf_diff->solve();
    std::cout << summary.BriefReport() << std::endl;
    std::cout << "______ solved ! Ox, Oy and Oz changed______" << std::endl;

    //This is a static test so we are not supposed to have moved from origin to last KeyFrame
    ASSERT_TRUE( (wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->getPPtr()->getVector() - wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().back()->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL) ) <<
                "origin and final frame position are different - problem when Ox, Oy and Oz changed" << std::endl;
    ASSERT_TRUE( (wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->getOPtr()->getVector() - wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().back()->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL) ) <<
                "origin and final frame orientation are different - problem when Ox, Oy and Oz changed" << std::endl;

    // COMPUTE COVARIANCES
    //std::cout << "\t\t\t ______computing covariances______ Ox, Oy and Oz changed" << std::endl;
    ceres_manager_wolf_diff->computeCovariances(ALL_MARGINALS);//ALL_MARGINALS, ALL
    //std::cout << "\t\t\t ______computed!______ Ox, Oy and Oz changed" << std::endl;

    wolf_problem_ptr_->print(4,1,1,1);
}


TEST(ProcessorOdom3D, static_ceresOptimisation_convergenceOdom_POV)
{

    /* Simple odom test including only processorOdom3D :
     * First keyFrame (origin) is fixed (0,0,0, 0,0,0,1). Then the odometry data for 2 second is [0,0,0, 0,0,0,1], meaning that we did not move at all. 
     * We give this simple graph to solver and let it solve. 
     *
     * But before solving, we change the state of final KeyFrame.
     * First we change only Px, then Py, then Pz, then all of them
     * Second : we change Ox, then Oy, then Oz, then all of them
     * Third : we change everything
     * Everything should converge and final keyFrame State should be exactly [0,0,0, 0,0,0,1 ,0,0,0] -->Checked with Assertions
     *
     */

    using std::shared_ptr;
    using std::make_shared;
    using std::static_pointer_cast;
                                            /************** SETTING PROBLEM  **************/

    std::string wolf_root = _WOLF_ROOT_DIR;

    // Wolf problem
    ProblemPtr wolf_problem_ptr_ = Problem::create(FRM_POV_3D);
    Eigen::VectorXs x0(10);
    x0 << 0,0,0,  0,0,0,1,  0,0,0;
    TimeStamp t(0);
    wolf_problem_ptr_->setOrigin(x0, Eigen::Matrix6s::Identity() * 0.001, t);

    SensorBasePtr sen = wolf_problem_ptr_->installSensor("ODOM 3D", "odom", (Vector7s()<<0,0,0,0,0,0,1).finished(), wolf_root + "/src/examples/sensor_odom_3D.yaml");

    // We want to create a processorMotion with a max_time_span of 2 seconds. but here we integrate only odometry and there should be no interpolation between
    // Default processorMotionParams is made so that a KeyFrame will be created at each step. This works in this case
    ProcessorOdom3DParamsPtr prc_odom_params = std::make_shared<ProcessorOdom3DParams>();
    wolf_problem_ptr_->installProcessor("ODOM 3D", "odometry integrator", sen, prc_odom_params);

    // Ceres wrappers
    ceres::Solver::Options ceres_options;
    ceres_options.minimizer_type = ceres::TRUST_REGION; //ceres::TRUST_REGION;ceres::LINE_SEARCH
    ceres_options.max_line_search_step_contraction = 1e-3;
    ceres_options.max_num_iterations = 1e4;
    CeresManager* ceres_manager_wolf_diff = new CeresManager(wolf_problem_ptr_, ceres_options, true);

                                             /************** USE ODOM_3D CLASSES  **************/

    VectorXs d(7);
    d << 0,0,0, 0,0,0,1;
    t.set(2);

    wolf::CaptureMotionPtr odom_ptr = std::make_shared<CaptureMotion>(t, sen, d);
    wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->fix();
    // process data in capture
    sen->process(odom_ptr);

    /*if(wolf_problem_ptr_->check(1)){
        wolf_problem_ptr_->print(4,1,1,1);
    }*/
    wolf_problem_ptr_->print(4,1,1,1);

                                             /************** SOLVER PART  **************/

    //If we want the covariances to be computed, then we need to fix all Velocity StateBlocks because they cannot be observed we Odometry measurements only
    /*for(FrameBasePtr Frame_ptr : wolf_problem_ptr_->getTrajectoryPtr()->getFrameList())
    {
        Frame_ptr->getVPtr()->fix();
    }*/

     /* ___________________________________________ CHANGING FINAL FRAME BEFORE OPTIMIZATION ___________________________________________*/
    
    //There should be 3 frames : origin KeyFrame, Generated KeyFrame at t = 2s, and another Frame for incoming data to be processed
    ASSERT_EQ(wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().size(),3);
    EXPECT_TRUE(wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->isFixed()) << "origin_frame is not fixed" << std::endl;

    //get pointer to the last KeyFrame (which is at t = 2s)
    EXPECT_EQ(t.get(),2);
    FrameBasePtr last_KF = wolf_problem_ptr_->getTrajectoryPtr()->closestKeyFrameToTimeStamp(t);
    Eigen::Matrix<wolf::Scalar, 10, 1> kf2_state = last_KF->getState(); //This state vector will be used to get the velocity state

    // FIRST SOLVER TEST WITHOUT CHANGING ANYTHING - WE DID NOT MOVE

    std::cout << "\n\t\t\t______ solving...______" << std::endl;
    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
    std::cout << summary.BriefReport() << std::endl;
    std::cout << "\t\t\t______ solved !______" << std::endl;

    // COMPUTE COVARIANCES
    //std::cout << "\t\t\t ______computing covariances______" << std::endl;
    //ceres_manager_wolf_diff->computeCovariances(ALL_MARGINALS);//ALL_MARGINALS, ALL
    //std::cout << "\t\t\t ______computed!______" << std::endl;


    //This is a static test so we are not supposed to have moved from origin to last KeyFrame
    ASSERT_TRUE( (wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->getPPtr()->getVector() - last_KF->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL) ) <<
                "origin and final frame position are different" << std::endl;
    ASSERT_TRUE( (wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->getOPtr()->getVector() - last_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL) ) <<
                "origin and final frame orientation are different" << std::endl;
    

                                                    /*********************/
                                                    //CHANGE PX AND SOLVE//
                                                    /*********************/

    last_KF->setState((Eigen::Matrix<wolf::Scalar,10,1>()<<30,0,0,0,0,0,1,kf2_state(7),kf2_state(8),kf2_state(9)).finished());

    std::cout << "\n\t\t\t______ solving... Px changed______" << std::endl;
    summary = ceres_manager_wolf_diff->solve();
    std::cout << summary.BriefReport() << std::endl;
    std::cout << "\t\t\t______ solved ! Px changed______" << std::endl;

    //This is a static test so we are not supposed to have moved from origin to last KeyFrame
    ASSERT_TRUE( (wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->getPPtr()->getVector() - last_KF->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS) ) <<
                "origin and final frame position are different - problem when Px is changed" << std::endl;
    ASSERT_TRUE( (wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->getOPtr()->getVector() - last_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL) ) <<
                "origin and final frame orientation are different - problem when Px is changed" << std::endl;

    // COMPUTE COVARIANCES
    //std::cout << "\t\t\t ______computing covariances______ Px changed" << std::endl;
    //ceres_manager_wolf_diff->computeCovariances(ALL_MARGINALS);//ALL_MARGINALS, ALL
    //std::cout << "\t\t\t ______computed!______ Px changed" << std::endl;


                                                    /*********************/
                                                    //CHANGE PY AND SOLVE//
                                                    /*********************/

    last_KF->setState((Eigen::Matrix<wolf::Scalar,10,1>()<<0,30,0,0,0,0,1,kf2_state(7),kf2_state(8),kf2_state(9)).finished());

    std::cout << "\n\t\t\t______ solving... Py changed______" << std::endl;
    summary = ceres_manager_wolf_diff->solve();
    std::cout << summary.BriefReport() << std::endl;
    std::cout << "\t\t\t______ solved ! Py changed______" << std::endl;

    //This is a static test so we are not supposed to have moved from origin to last KeyFrame
    ASSERT_TRUE( (wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->getPPtr()->getVector() - last_KF->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS) ) <<
                "origin and final frame position are different - problem when Py is changed" << std::endl;
    ASSERT_TRUE( (wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->getOPtr()->getVector() - last_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL) ) <<
                "origin and final frame orientation are different - problem when Py is changed" << std::endl;

    // COMPUTE COVARIANCES
    //std::cout << "\t\t\t ______computing covariances______ Py changed" << std::endl;
    //ceres_manager_wolf_diff->computeCovariances(ALL_MARGINALS);//ALL_MARGINALS, ALL
    //std::cout << "\t\t\t ______computed!______ Py changed" << std::endl;


                                                    /*********************/
                                                    //CHANGE PZ AND SOLVE//
                                                    /*********************/

    last_KF->setState((Eigen::Matrix<wolf::Scalar,10,1>()<<0,0,30,0,0,0,1,kf2_state(7),kf2_state(8),kf2_state(9)).finished());

    std::cout << "\n\t\t\t______ solving... Pz changed______" << std::endl;
    summary = ceres_manager_wolf_diff->solve();
    std::cout << summary.BriefReport() << std::endl;
    std::cout << "\t\t\t______ solved ! Pz changed______" << std::endl;

    //This is a static test so we are not supposed to have moved from origin to last KeyFrame
    ASSERT_TRUE( (wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->getPPtr()->getVector() - last_KF->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS) ) <<
                "origin and final frame position are different - problem when Pz is changed" << std::endl;
    ASSERT_TRUE( (wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->getOPtr()->getVector() - last_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL) ) <<
                "origin and final frame orientation are different - problem when Pz is changed" << std::endl;

    // COMPUTE COVARIANCES
    //std::cout << "\t\t\t ______computing covariances______ Pz changed" << std::endl;
    //ceres_manager_wolf_diff->computeCovariances(ALL_MARGINALS);//ALL_MARGINALS, ALL
    //std::cout << "\t\t\t ______computed!______ Pz changed" << std::endl;


                                                    /********************************/
                                                    //CHANGE PX, Py AND PZ AND SOLVE//
                                                    /********************************/

    last_KF->setState((Eigen::Matrix<wolf::Scalar,10,1>()<<25,20,30,0,0,0,1,kf2_state(7),kf2_state(8),kf2_state(9)).finished());

    std::cout << "\n\t\t\t______ solving... Px, Py and Pz changed______" << std::endl;
    summary = ceres_manager_wolf_diff->solve();
    std::cout << summary.BriefReport() << std::endl;
    std::cout << "\t\t\t______ solved ! Px, Py and Pz changed______" << std::endl;

    //This is a static test so we are not supposed to have moved from origin to last KeyFrame
    ASSERT_TRUE( (wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->getPPtr()->getVector() - last_KF->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS) ) <<
                "origin and final frame position are different - problem when Px, Py and Pz are changed" << std::endl;
    ASSERT_TRUE( (wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->getOPtr()->getVector() - last_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL) ) <<
                "origin and final frame orientation are different - problem when Px, Py and Pz are changed" << std::endl;

    // COMPUTE COVARIANCES
    //std::cout << "\n\t\t\t ______computing covariances______ Px, Py and Pz changed" << std::endl;
    //ceres_manager_wolf_diff->computeCovariances(ALL_MARGINALS);//ALL_MARGINALS, ALL
    //std::cout << "\t\t\t ______computed!______ Px, Py and Pz changed" << std::endl;


                                                    /*********************/
                                                    //CHANGE OX AND SOLVE//
                                                    /*********************/
    Eigen::Vector3s o_initial_guess;
    Eigen::Quaternions q_init_guess;

    o_initial_guess << 40,0,0;
    q_init_guess = v2q(o_initial_guess);
    last_KF->setState((Eigen::Matrix<wolf::Scalar,10,1>()<<0,0,0,q_init_guess.x(),q_init_guess.y(),q_init_guess.z(),q_init_guess.w(),kf2_state(7),kf2_state(8),kf2_state(9)).finished());

    std::cout << "\n\t\t\t______ solving... Ox changed______" << std::endl;
    summary = ceres_manager_wolf_diff->solve();
    std::cout << summary.BriefReport() << std::endl;
    std::cout << "\t\t\t______ solved ! Ox changed______" << std::endl;

    //This is a static test so we are not supposed to have moved from origin to last KeyFrame
    ASSERT_TRUE( (wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->getPPtr()->getVector() - last_KF->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL) ) <<
                "origin and final frame position are different - problem when Ox is changed" << std::endl;
    ASSERT_TRUE( (wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->getOPtr()->getVector() - last_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS) ) <<
                "origin and final frame orientation are different - problem when Ox is changed" << std::endl;

    // COMPUTE COVARIANCES
    //std::cout << "\t\t\t ______computing covariances______ Ox changed" << std::endl;
    //ceres_manager_wolf_diff->computeCovariances(ALL_MARGINALS);//ALL_MARGINALS, ALL
    //std::cout << "\t\t\t ______computed!______ Ox changed" << std::endl;


                                                    /*********************/
                                                    //CHANGE OY AND SOLVE//
                                                    /*********************/
    o_initial_guess << 0,40,0;
    q_init_guess = v2q(o_initial_guess);
    last_KF->setState((Eigen::Matrix<wolf::Scalar,10,1>()<<0,0,0,q_init_guess.x(),q_init_guess.y(),q_init_guess.z(),q_init_guess.w(),kf2_state(7),kf2_state(8),kf2_state(9)).finished());

    std::cout << "\n\t\t\t______ solving... Oy changed______" << std::endl;
    summary = ceres_manager_wolf_diff->solve();
    std::cout << summary.BriefReport() << std::endl;
    std::cout << "\t\t\t______ solved ! Oy changed______" << std::endl;

    //This is a static test so we are not supposed to have moved from origin to last KeyFrame
    ASSERT_TRUE( (wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->getPPtr()->getVector() - last_KF->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL) ) <<
                "origin and final frame position are different - problem when Oy is changed" << std::endl;
    ASSERT_TRUE( (wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->getOPtr()->getVector() - last_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS) ) <<
                "origin and final frame orientation are different - problem when Oy is changed" << std::endl;

    // COMPUTE COVARIANCES
    //std::cout << "\t\t\t ______computing covariances______ Oy changed" << std::endl;
    //ceres_manager_wolf_diff->computeCovariances(ALL_MARGINALS);//ALL_MARGINALS, ALL
    //std::cout << "\t\t\t ______computed!______ Oy changed" << std::endl;

                                                    /*********************/
                                                    //CHANGE OZ AND SOLVE//
                                                    /*********************/
    o_initial_guess << 0,0,40;
    q_init_guess = v2q(o_initial_guess);
    last_KF->setState((Eigen::Matrix<wolf::Scalar,10,1>()<<0,0,0,q_init_guess.x(),q_init_guess.y(),q_init_guess.z(),q_init_guess.w(),kf2_state(7),kf2_state(8),kf2_state(9)).finished());

    std::cout << "\n\t\t\t______ solving... Oz changed______" << std::endl;
    summary = ceres_manager_wolf_diff->solve();
    std::cout << summary.BriefReport() << std::endl;
    std::cout << "\t\t\t______ solved ! Oz changed______" << std::endl;

    //This is a static test so we are not supposed to have moved from origin to last KeyFrame
    ASSERT_TRUE( (wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->getPPtr()->getVector() - last_KF->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL) ) <<
                "origin and final frame position are different - problem when Oz is changed" << std::endl;
    ASSERT_TRUE( (wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->getOPtr()->getVector() - last_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS) ) <<
                "origin and final frame orientation are different - problem when Oz is changed" << std::endl;

    // COMPUTE COVARIANCES
    //std::cout << "\t\t\t ______computing covariances______ Oz changed" << std::endl;
    //ceres_manager_wolf_diff->computeCovariances(ALL_MARGINALS);//ALL_MARGINALS, ALL
    //std::cout << "\t\t\t ______computed!______ Oz changed" << std::endl;


                                                    /********************************/
                                                    //CHANGE OX, OY AND OZ AND SOLVE//
                                                    /********************************/
    o_initial_guess << 80,50,40;
    q_init_guess = v2q(o_initial_guess);
    last_KF->setState((Eigen::Matrix<wolf::Scalar,10,1>()<<0,0,0,q_init_guess.x(),q_init_guess.y(),q_init_guess.z(),q_init_guess.w(),kf2_state(7),kf2_state(8),kf2_state(9)).finished());

    std::cout << "\n\t\t\t______ solving... Ox, Oy and Oz changed______" << std::endl;
    summary = ceres_manager_wolf_diff->solve();
    std::cout << summary.BriefReport() << std::endl;
    std::cout << "\t\t\t______ solved ! Ox, Oy and Oz changed______" << std::endl;

    //This is a static test so we are not supposed to have moved from origin to last KeyFrame
    ASSERT_TRUE( (wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->getPPtr()->getVector() - last_KF->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL) ) <<
                "origin and final frame position are different - problem when Ox, Oy and Oz changed" << std::endl;
    ASSERT_TRUE( (wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->getOPtr()->getVector() - last_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS) ) <<
                "origin and final frame orientation are different - problem when Ox, Oy and Oz changed" << std::endl;

    // COMPUTE COVARIANCES
    //std::cout << "\t\t\t ______computing covariances______ Ox, Oy and Oz changed" << std::endl;
    //ceres_manager_wolf_diff->computeCovariances(ALL_MARGINALS);//ALL_MARGINALS, ALL
    //std::cout << "\t\t\t ______computed!______ Ox, Oy and Oz changed" << std::endl;

    wolf_problem_ptr_->print(4,1,1,1);
}


TEST(ProcessorIMU, static_ceresOptimisation_fixBias)
{
    /* In this scenario, we simulate the integration of a perfect IMU that is not moving.
     * Initial State is [0,0,0, 0,0,0,1, 0,0,0] so we expect the Final State to be exactly the same
     * Origin KeyFrame is fixed
     * 
     * Bias of all frames are fixed before we call the solver
     * 
     * With IMU data only, biases are not observable ! So covariance cannot be computed due to jacobian rank deficiency.
     * We must add an odometry to make covariances observable Or... we could fix all bias stateBlocks
     * First we will try to fix bias stateBlocks
     */


    using std::shared_ptr;
    using std::make_shared;
    using std::static_pointer_cast;
                                            /************** SETTING PROBLEM  **************/

    std::string wolf_root = _WOLF_ROOT_DIR;

    // Wolf problem
    ProblemPtr wolf_problem_ptr_ = Problem::create(FRM_PQVBB_3D);

    SensorBasePtr sen_imu = wolf_problem_ptr_->installSensor("IMU", "Main IMU", (Vector7s()<<0,0,0,0,0,0,1).finished(), wolf_root + "/src/examples/sensor_imu.yaml");

    // We want to create a processorIMU with a max_time_span of 1 seconds.
    // Default processorIMUParams is made so that a KeyFrame will be created at each step.
    ProcessorIMUParamsPtr prc_imu_params = std::make_shared<ProcessorIMUParams>();
    prc_imu_params->max_time_span = 1;
    prc_imu_params->max_buff_length = 1000000000; //make it very high so that this condition will not pass
    prc_imu_params->dist_traveled = 1000000000;
    prc_imu_params->angle_turned = 1000000000;
    prc_imu_params->voting_active = true;

    ProcessorBasePtr processor_ptr = wolf_problem_ptr_->installProcessor("IMU", "IMU pre-integrator", sen_imu, prc_imu_params);

    //setting origin and fixing origin KeyFrame
    Eigen::VectorXs x0(16);
    TimeStamp t(0);
    x0 << 0,0,0,  0,0,0,1,  0,0,0,  0,0,0,  0,0,0;
    wolf_problem_ptr_->getProcessorMotionPtr()->setOrigin(x0, t); //this also creates a keyframe at origin
    wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->fix();

    // Ceres wrappers
    ceres::Solver::Options ceres_options;
    ceres_options.minimizer_type = ceres::TRUST_REGION; //ceres::TRUST_REGION;ceres::LINE_SEARCH
    ceres_options.max_line_search_step_contraction = 1e-3;
    ceres_options.max_num_iterations = 1e4;
    CeresManager* ceres_manager_wolf_diff = new CeresManager(wolf_problem_ptr_, ceres_options, true);

                                             /************** USE IMU CLASSES  **************/
    Eigen::Vector6s data;
    data << 0.0,0.0,-wolf::gravity()(2), 0.0,0.0,0.0; //we use exactly the value of gravity defined in wolf.h

    //integrate IMU data until KeyFrame creation (until we reach max_time_span)
    Scalar dt = t.get();
    TimeStamp ts(0);
    while((dt-t.get())<=std::static_pointer_cast<ProcessorIMU>(processor_ptr)->getMaxTimeSpan()){
    // Time and data variables
    dt += 0.001;
    ts.set(dt);

    wolf::CaptureIMUPtr imu_ptr = std::make_shared<CaptureIMU>(ts, sen_imu, data);
    // process data in capture
    sen_imu->process(imu_ptr);
    }

    //Fix all biases StateBlocks -> to make covariances computable
    for(FrameBasePtr it : wolf_problem_ptr_->getTrajectoryPtr()->getFrameList()){
        ( std::static_pointer_cast<FrameIMU>(it) )->getAccBiasPtr()->fix();
        ( std::static_pointer_cast<FrameIMU>(it) )->getGyroBiasPtr()->fix();
    }

    //Check and print wolf tree
    if(wolf_problem_ptr_->check(1)){
        wolf_problem_ptr_->print(4,1,1,1);
    }
    
                                             /************** SOLVER PART  **************/
     ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
     std::cout << summary.FullReport() << std::endl;

     wolf_problem_ptr_->print(4,1,1,1);

     //We check with assertions if Final KeyFrame has the same state as origin_KF
     FrameBasePtr origin_KF = wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front();
     FrameBasePtr last_KF = wolf_problem_ptr_->getTrajectoryPtr()->closestKeyFrameToTimeStamp(ts);
     ASSERT_TRUE( (last_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS ));
     ASSERT_TRUE( (last_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS ));
     ASSERT_TRUE( (last_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS ));
     
    // COMPUTE COVARIANCES
    std::cout << "\t\t\t ______computing covariances______" << std::endl;
    ceres_manager_wolf_diff->computeCovariances(ALL);//ALL_MARGINALS, ALL
    std::cout << "\t\t\t ______computed!______" << std::endl;
}

//_______________________________________________________________________________________________________________
// ##################################### static_Optim_IMUOdom_2KF TESTS #####################################
//_______________________________________________________________________________________________________________

/*  Tests below will be based on the following representation :
 *     KF0 ---- constraintIMU ---- KF1
 *        \____constraintOdom3D___/
 */

 //Following tests have been tested and work. They are commented because not sorelevant but very long at execution

/*TEST_F(ProcessorIMU_Odom_tests_details, static_Optim_IMUOdom_2KF_Unfix_upTo5_StateBlocks)
{
    // We start by fixing both KeyFrames.
    // Among all the stateBlocks contained in these 2 keyFrames, we unfix up to 5 stateBlocks before calling Ceres

    WOLF_INFO("\n\t ######### BOTH KF FIXED, UNFIX UP TO 5 STATEBLOCKS #########")
    WOLF_WARN("This test is quite long\n");
    wolf::Scalar progress(0);

    for(int i = 0; i<allStateBlocks.size(); i++)
    {
        for(int j = 0; j<allStateBlocks.size(); j++)
        {   
            for(int k = 0; k<allStateBlocks.size(); k++)
            {
                for(int l = 0; l<allStateBlocks.size(); l++)
                {
                    for(int m = 0; m<allStateBlocks.size(); m++)
                    {
                        origin_KF->setState(initial_origin_state);
                        last_KF->setState(initial_final_state);

                        origin_KF->fix();
                        last_KF->fix();

                        // unfix desired stateBlocks
                        allStateBlocks[i]->unfix();
                        allStateBlocks[j]->unfix();
                        allStateBlocks[k]->unfix();
                        allStateBlocks[l]->unfix();
                        allStateBlocks[m]->unfix();

                        summary = ceres_manager_wolf_diff->solve();

                        ASSERT_TRUE( (last_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*1000 ));
                        ASSERT_TRUE( (last_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*1000 ));
                        ASSERT_TRUE( (last_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*10000 ));
                        ASSERT_TRUE( (last_KF->getAccBiasPtr()->getVector() - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*1000 )); //because we simulate a perfect IMU
                        ASSERT_TRUE( (last_KF->getGyroBiasPtr()->getVector() - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*1000 )); //because we simulate a perfect IMU
                    }
                }
            }
        }
        progress +=10;
        WOLF_INFO("% completed : ", progress);
    }
}


TEST_F(ProcessorIMU_Odom_tests_details, static_Optim_IMUOdom_2KF_fixOrigin_UnfixUpTo3)
{
    // We keep last_KF unfixed and unfix up to 3 stateBlocks in origin_KF

    WOLF_INFO("\n\t ######### ORIGIN KF FIXED, UNFIX UP TO 3 STATEBLOCKS #########")
    WOLF_WARN("This test is quite long\n");
    wolf::Scalar progress(0);

    last_KF->unfix();
    for(int i = 0; i<originStateBlock_vec.size(); i++)
    {
        for(int j = 0; j<originStateBlock_vec.size(); j++)
        {   
            for(int k = 0; k<originStateBlock_vec.size(); k++)
            {
                origin_KF->setState(initial_origin_state);
                last_KF->setState(initial_final_state);

                origin_KF->fix();

                originStateBlock_vec[i]->unfix();
                originStateBlock_vec[j]->unfix();
                originStateBlock_vec[k]->unfix();

                summary = ceres_manager_wolf_diff->solve();

                ASSERT_TRUE( (last_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*1000 ));
                ASSERT_TRUE( (last_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*1000 ));
                ASSERT_TRUE( (last_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*10000 ));
                ASSERT_TRUE( (last_KF->getAccBiasPtr()->getVector() - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*1000 )); //because we simulate a perfect IMU
                ASSERT_TRUE( (last_KF->getGyroBiasPtr()->getVector() - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*1000 )); //because we simulate a perfect IMU

            }
        }
        progress += 20;
        WOLF_INFO("% completed : ", progress);
    }
}

TEST_F(ProcessorIMU_Odom_tests_details, static_Optim_IMUOdom_2KF_fixLast_UnfixUpTo3)
{
    // We keep origin_KF unfixed and unfix up to 3 stateBlocks in last_KF

    WOLF_INFO("\n\t ######### LAST KF FIXED, UNFIX UP TO 3 STATEBLOCKS #########")
    WOLF_WARN("This test is quite long\n");
    wolf::Scalar progress(0);
    
    origin_KF->unfix();
    for(int i = 0; i<finalStateBlock_vec.size(); i++)
    {
        for(int j = 0; j<finalStateBlock_vec.size(); j++)
        {   
            for(int k = 0; k<finalStateBlock_vec.size(); k++)
            {
                origin_KF->setState(initial_origin_state);
                last_KF->setState(initial_final_state);

                last_KF->fix(); 

                finalStateBlock_vec[i]->unfix();
                finalStateBlock_vec[j]->unfix();
                finalStateBlock_vec[k]->unfix();

                summary = ceres_manager_wolf_diff->solve();

                ASSERT_TRUE( (last_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*1000 ));
                ASSERT_TRUE( (last_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*1000 ));
                ASSERT_TRUE( (last_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*10000 ));
                ASSERT_TRUE( (last_KF->getAccBiasPtr()->getVector() - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*1000 )); //because we simulate a perfect IMU
                ASSERT_TRUE( (last_KF->getGyroBiasPtr()->getVector() - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*1000 )); //because we simulate a perfect IMU
            }
        }
        progress += 20;
        WOLF_INFO("% completed : ", progress);
    }
}


TEST_F(ProcessorIMU_Odom_tests_details, static_Optim_IMUOdom_2KF_AllUnfixed)
{
    origin_KF->setState(initial_origin_state);
    last_KF->setState(initial_final_state);

    origin_KF->unfix();
    last_KF->unfix();

    summary = ceres_manager_wolf_diff->solve();
    std::cout << summary.BriefReport() << std::endl;

    ASSERT_TRUE( (last_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*1000 ));
    ASSERT_TRUE( (last_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*1000 ));
    ASSERT_TRUE( (last_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*10000 ));
    ASSERT_TRUE( (last_KF->getAccBiasPtr()->getVector() - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*1000 )); //because we simulate a perfect IMU
    ASSERT_TRUE( (last_KF->getGyroBiasPtr()->getVector() - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*1000 )); //because we simulate a perfect IMU
}*/


TEST_F(ProcessorIMU_Odom_tests_details, static_Optim_IMUOdom_2KF_perturbate_positionOrigin1_Unfix1)
{
    /* In this test, only the position StateBlock from origin_KF and another StateBlock are unfixed. All the other KF are fixed.
     * We perturbate 1 of the origin positions before calling Ceres.
     * The added odometry and IMU constraints say that we did not move between both KF. We expect CERES to converge and get the same position for both KF
     * Since IMU constraint says that we did not move at all, we also expect the velocities to be equal in both KF.
     * The orientation should also be the same in both KeyFrames.
     *
     * Finally, we expect the Acceleration and Gyroscope Bias to be equal in both KeyFrames (Zero Vector)
     */

    for(int pert_index = 0; pert_index<3; pert_index++)
    {
        //perturate initial state
        perturbated_origin_state = initial_origin_state;
        perturbated_origin_state(pert_index) += 1.0;

        for(int i = 1; i<originStateBlock_vec.size(); i++)
        {

            origin_KF->setState(perturbated_origin_state);
            last_KF->setState(initial_final_state);

            origin_KF->fix();
            last_KF->fix();

            //we unfix origin position stateblock to let it converge
            originStateBlock_vec[0]->unfix();

            //we now unfix only one StateBlock
            originStateBlock_vec[i]->unfix();

            summary = ceres_manager_wolf_diff->solve();
            //std::cout << summary.BriefReport() << std::endl;

            ASSERT_TRUE( (last_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 )) << 
            "last position state : " << last_KF->getPPtr()->getVector().transpose() << "\n origin position state : " << origin_KF->getPPtr()->getVector().transpose() << std::endl;;
            ASSERT_TRUE( (last_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*1000 ));
            ASSERT_TRUE( (last_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1,  0.00000001 ));
            ASSERT_TRUE( (last_KF->getAccBiasPtr()->getVector() - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 ));
            ASSERT_TRUE( (last_KF->getGyroBiasPtr()->getVector() - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 ));
            ASSERT_TRUE( (last_KF->getPPtr()->getVector() - (Eigen::Vector3s()<<0,0,0).finished()).isMuchSmallerThan(1, 0.0000001 )) << 
            "last position state : " << last_KF->getPPtr()->getVector().transpose() << std::endl;
            ASSERT_TRUE( (last_KF->getOPtr()->getVector() - (Eigen::Vector4s()<<0,0,0,1).finished()).isMuchSmallerThan(1, 0.0000001 ));
            ASSERT_TRUE( (last_KF->getVPtr()->getVector() - (Eigen::Vector3s()<<0,0,0).finished()).isMuchSmallerThan(1, 0.0000001 ));
        }

        for(int i = 1; i<finalStateBlock_vec.size(); i++)
        {

            origin_KF->setState(perturbated_origin_state);
            last_KF->setState(initial_final_state);

            origin_KF->fix();
            last_KF->fix();

            //we unfix origin position stateblock to let it converge
            originStateBlock_vec[0]->unfix();

            //we now unfix only one StateBlock
            finalStateBlock_vec[i]->unfix();

            summary = ceres_manager_wolf_diff->solve();
            //std::cout << summary.BriefReport() << std::endl;

            ASSERT_TRUE( (last_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 )) << 
            "last position state : " << last_KF->getPPtr()->getVector().transpose() << "\n origin position state : " << origin_KF->getPPtr()->getVector().transpose() << std::endl;;
            ASSERT_TRUE( (last_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*1000 ));
            ASSERT_TRUE( (last_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1,  0.00000001 ));
            ASSERT_TRUE( (last_KF->getAccBiasPtr()->getVector() - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 ));
            ASSERT_TRUE( (last_KF->getGyroBiasPtr()->getVector() - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 ));
            ASSERT_TRUE( (last_KF->getPPtr()->getVector() - (Eigen::Vector3s()<<0,0,0).finished()).isMuchSmallerThan(1, 0.0000001 )) << 
            "last position state : " << last_KF->getPPtr()->getVector().transpose() << std::endl;
            ASSERT_TRUE( (last_KF->getOPtr()->getVector() - (Eigen::Vector4s()<<0,0,0,1).finished()).isMuchSmallerThan(1, 0.0000001 ));
            ASSERT_TRUE( (last_KF->getVPtr()->getVector() - (Eigen::Vector3s()<<0,0,0).finished()).isMuchSmallerThan(1, 0.0000001 ));
        }
    }
}

TEST_F(ProcessorIMU_Odom_tests_details, static_Optim_IMUOdom_2KF_perturbate_positionOrigin2_Unfix1)
{
    /* In this test, only the position StateBlock from origin_KF and another StateBlock are unfixed. All the other KF are fixed.
     * We perturbate 2 of the origin positions before calling Ceres.
     * The added odometry and IMU constraints say that we did not move between both KF. We expect CERES to converge and get the same position for both KF
     * Since IMU constraint says that we did not move at all, we also expect the velocities to be equal in both KF.
     * The orientation should also be the same in both KeyFrames.
     *
     * Finally, we expect the Acceleration and Gyroscope Bias to be equal in both KeyFrames (Zero Vector)
     */

    for(int pert_index0 = 0; pert_index0<3; pert_index0++)
    {
        for(int pert_index1 = 1; pert_index1<3; pert_index1++)
        {
            //perturate initial state
            perturbated_origin_state = initial_origin_state;
            perturbated_origin_state(pert_index0) += 1.0;
            perturbated_origin_state(pert_index1) += 1.0;

            for(int i = 1; i<originStateBlock_vec.size(); i++)
            {

                origin_KF->setState(perturbated_origin_state);
                last_KF->setState(initial_final_state);

                origin_KF->fix();
                last_KF->fix();

                //we unfix origin position stateblock to let it converge
                originStateBlock_vec[0]->unfix();

                //we now unfix only one StateBlock
                originStateBlock_vec[i]->unfix();

                summary = ceres_manager_wolf_diff->solve();
                //std::cout << summary.BriefReport() << std::endl;

                ASSERT_TRUE( (last_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 )) << 
                "last position state : " << last_KF->getPPtr()->getVector().transpose() << "\n origin position state : " << origin_KF->getPPtr()->getVector().transpose() << std::endl;;
                ASSERT_TRUE( (last_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*1000 ));
                ASSERT_TRUE( (last_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1,  0.00000001 ));
                ASSERT_TRUE( (last_KF->getAccBiasPtr()->getVector() - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 ));
                ASSERT_TRUE( (last_KF->getGyroBiasPtr()->getVector() - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 ));
                ASSERT_TRUE( (last_KF->getPPtr()->getVector() - (Eigen::Vector3s()<<0,0,0).finished()).isMuchSmallerThan(1, 0.0000001 )) << 
                "last position state : " << last_KF->getPPtr()->getVector().transpose() << std::endl;
                ASSERT_TRUE( (last_KF->getOPtr()->getVector() - (Eigen::Vector4s()<<0,0,0,1).finished()).isMuchSmallerThan(1, 0.0000001 ));
                ASSERT_TRUE( (last_KF->getVPtr()->getVector() - (Eigen::Vector3s()<<0,0,0).finished()).isMuchSmallerThan(1, 0.0000001 ));
            }

            for(int i = 1; i<finalStateBlock_vec.size(); i++)
            {

                origin_KF->setState(perturbated_origin_state);
                last_KF->setState(initial_final_state);

                origin_KF->fix();
                last_KF->fix();

                //we unfix origin position stateblock to let it converge
                originStateBlock_vec[0]->unfix();

                //we now unfix only one StateBlock
                finalStateBlock_vec[i]->unfix();

                summary = ceres_manager_wolf_diff->solve();
                //std::cout << summary.BriefReport() << std::endl;

                ASSERT_TRUE( (last_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 )) << 
                "last position state : " << last_KF->getPPtr()->getVector().transpose() << "\n origin position state : " << origin_KF->getPPtr()->getVector().transpose() << std::endl;;
                ASSERT_TRUE( (last_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*1000 ));
                ASSERT_TRUE( (last_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1,  0.00000001 ));
                ASSERT_TRUE( (last_KF->getAccBiasPtr()->getVector() - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 ));
                ASSERT_TRUE( (last_KF->getGyroBiasPtr()->getVector() - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 ));
                ASSERT_TRUE( (last_KF->getPPtr()->getVector() - (Eigen::Vector3s()<<0,0,0).finished()).isMuchSmallerThan(1, 0.0000001 )) << 
                "last position state : " << last_KF->getPPtr()->getVector().transpose() << std::endl;
                ASSERT_TRUE( (last_KF->getOPtr()->getVector() - (Eigen::Vector4s()<<0,0,0,1).finished()).isMuchSmallerThan(1, 0.0000001 ));
                ASSERT_TRUE( (last_KF->getVPtr()->getVector() - (Eigen::Vector3s()<<0,0,0).finished()).isMuchSmallerThan(1, 0.0000001 ));
            }
        }
    }
}

TEST_F(ProcessorIMU_Odom_tests_details, static_Optim_IMUOdom_2KF_perturbate_positionOrigin3_Unfix1)
{
    /* In this test, only the position StateBlock from origin_KF and another StateBlock are unfixed. All the other KF are fixed.
     * We perturbate all of the origin positions (Px, Py, Pz) before calling Ceres.
     * The added odometry and IMU constraints say that we did not move between both KF. We expect CERES to converge and get the same position for both KF
     * Since IMU constraint says that we did not move at all, we also expect the velocities to be equal in both KF.
     * The orientation should also be the same in both KeyFrames.
     *
     * Finally, we expect the Acceleration and Gyroscope Bias to be equal in both KeyFrames (Zero Vector)
     */

    //perturate initial state
    perturbated_origin_state = initial_origin_state;
    perturbated_origin_state(0) += 1.0;
    perturbated_origin_state(1) += 2.0;
    perturbated_origin_state(2) += 3.0;

    for(int i = 1; i<originStateBlock_vec.size(); i++)
    {
        origin_KF->setState(perturbated_origin_state);
        last_KF->setState(initial_final_state);

        origin_KF->fix(); 
        last_KF->fix();

        //we unfix origin position stateblock to let it converge
        originStateBlock_vec[0]->unfix();

        //we now unfix only one StateBlock
        originStateBlock_vec[i]->unfix();

        summary = ceres_manager_wolf_diff->solve();
        //std::cout << summary.BriefReport() << std::endl;

        ASSERT_TRUE( (last_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 )) << 
        "last position state : " << last_KF->getPPtr()->getVector().transpose() << "\n origin position state : " << origin_KF->getPPtr()->getVector().transpose() << std::endl;;
        ASSERT_TRUE( (last_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*1000)) <<
        "last velocity : " << last_KF->getOPtr()->getVector().transpose() << "\n origin velocity : " << origin_KF->getOPtr()->getVector().transpose() << std::endl;
        ASSERT_TRUE( (last_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1,  0.00000001 ));
        ASSERT_TRUE( (last_KF->getAccBiasPtr()->getVector() - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 )); 
        ASSERT_TRUE( (last_KF->getGyroBiasPtr()->getVector() - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 )); 
        ASSERT_TRUE( (last_KF->getPPtr()->getVector() - (Eigen::Vector3s()<<0,0,0).finished()).isMuchSmallerThan(1, 0.0000001 )) << 
        "last position state : " << last_KF->getPPtr()->getVector().transpose() << std::endl;
        ASSERT_TRUE( (last_KF->getOPtr()->getVector() - (Eigen::Vector4s()<<0,0,0,1).finished()).isMuchSmallerThan(1, 0.0000001 ));
        ASSERT_TRUE( (last_KF->getVPtr()->getVector() - (Eigen::Vector3s()<<0,0,0).finished()).isMuchSmallerThan(1, 0.0000001 ));
    }

    for(int i = 1; i<finalStateBlock_vec.size(); i++)
    {

        origin_KF->setState(perturbated_origin_state);
        last_KF->setState(initial_final_state);

        origin_KF->fix();
        last_KF->fix();

        //we unfix origin position stateblock to let it converge
        originStateBlock_vec[0]->unfix();

        //we now unfix only one StateBlock
        finalStateBlock_vec[i]->unfix();

        summary = ceres_manager_wolf_diff->solve();
        //std::cout << summary.BriefReport() << std::endl;

        ASSERT_TRUE( (last_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 )) << 
        "last position state : " << last_KF->getPPtr()->getVector().transpose() << "\n origin position state : " << origin_KF->getPPtr()->getVector().transpose() << std::endl;;
        ASSERT_TRUE( (last_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*1000 ));
        ASSERT_TRUE( (last_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1,  0.00000001 ));
        ASSERT_TRUE( (last_KF->getAccBiasPtr()->getVector() - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 )); //because we simulate a perfect IMU
        ASSERT_TRUE( (last_KF->getGyroBiasPtr()->getVector() - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 )); //because we simulate a perfect IMU
        ASSERT_TRUE( (last_KF->getPPtr()->getVector() - (Eigen::Vector3s()<<0,0,0).finished()).isMuchSmallerThan(1, 0.0000001 )) << 
        "last position state : " << last_KF->getPPtr()->getVector().transpose() << std::endl;
        ASSERT_TRUE( (last_KF->getOPtr()->getVector() - (Eigen::Vector4s()<<0,0,0,1).finished()).isMuchSmallerThan(1, 0.0000001 ));
        ASSERT_TRUE( (last_KF->getVPtr()->getVector() - (Eigen::Vector3s()<<0,0,0).finished()).isMuchSmallerThan(1, 0.0000001 ));
    }
}

TEST_F(ProcessorIMU_Odom_tests_details, static_Optim_IMUOdom_2KF_perturbate_positionLast3_Unfix1)
{
    /* In this test, only the position StateBlock from last_KF and another StateBlock are unfixed. All the other KF are fixed.
     * We perturbate all of the final positions (Px, Py, Pz) before calling Ceres.
     * The added odometry and IMU constraints say that we did not move between both KF. We expect CERES to converge and get the same position for both KF
     * Since IMU constraint says that we did not move at all, we also expect the velocities to be equal in both KF.
     * The orientation should also be the same in both KeyFrames.
     *
     * Finally, we expect the Acceleration and Gyroscope Bias to be equal in both KeyFrames (Zero Vector)
     */

    //perturate initial state
    perturbated_final_state = initial_final_state;
    perturbated_final_state(0) += 1.0;
    perturbated_final_state(1) += 2.0;
    perturbated_final_state(2) += 3.0;

    for(int i = 1; i<originStateBlock_vec.size(); i++)
    {

        origin_KF->setState(initial_origin_state);
        last_KF->setState(perturbated_final_state);

        origin_KF->fix();
        last_KF->fix();

        //we unfix origin position stateblock to let it converge
        finalStateBlock_vec[0]->unfix();

        //we now unfix only one StateBlock
        originStateBlock_vec[i]->unfix();

        summary = ceres_manager_wolf_diff->solve();
        //std::cout << summary.BriefReport() << std::endl;

        ASSERT_TRUE( (last_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 )) << 
        "last position state : " << last_KF->getPPtr()->getVector().transpose() << "\n origin position state : " << origin_KF->getPPtr()->getVector().transpose() << std::endl;;
        ASSERT_TRUE( (last_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*1000 ));
        ASSERT_TRUE( (last_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1,  0.00000001 ));
        ASSERT_TRUE( (last_KF->getAccBiasPtr()->getVector() - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 )); 
        ASSERT_TRUE( (last_KF->getGyroBiasPtr()->getVector() - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 )); 
        ASSERT_TRUE( (last_KF->getPPtr()->getVector() - (Eigen::Vector3s()<<0,0,0).finished()).isMuchSmallerThan(1, 0.0000001 )) << 
        "last position state : " << last_KF->getPPtr()->getVector().transpose() << std::endl;
        ASSERT_TRUE( (last_KF->getOPtr()->getVector() - (Eigen::Vector4s()<<0,0,0,1).finished()).isMuchSmallerThan(1, 0.0000001 ));
        ASSERT_TRUE( (last_KF->getVPtr()->getVector() - (Eigen::Vector3s()<<0,0,0).finished()).isMuchSmallerThan(1, 0.0000001 ));
    }

    for(int i = 1; i<finalStateBlock_vec.size(); i++)
    {
        
        origin_KF->setState(initial_origin_state);
        last_KF->setState(perturbated_final_state);

        origin_KF->fix(); 
        last_KF->fix();

        //we unfix origin position stateblock to let it converge
        finalStateBlock_vec[0]->unfix();

        //we now unfix only one StateBlock
        finalStateBlock_vec[i]->unfix();

        summary = ceres_manager_wolf_diff->solve();

        /*
         * Origin KF is fixed. Only 1 StateBlock is unfixed in last_KF.
         * Even with the odometry constraint imposing no movement. It is not sure that we can converge exactly to the pose of origin_KF.
         * otherwise, one of the conditions wil not be met.
         */

        EXPECT_TRUE( (last_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 )) << 
        "last position state : " << last_KF->getPPtr()->getVector().transpose() << "\n origin position state : " << origin_KF->getPPtr()->getVector().transpose() << std::endl;;
        ASSERT_TRUE( (last_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*1000 ));
        ASSERT_TRUE( (last_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1,  0.00000001 ));
        ASSERT_TRUE( (last_KF->getAccBiasPtr()->getVector() - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 )); //because we simulate a perfect IMU
        ASSERT_TRUE( (last_KF->getGyroBiasPtr()->getVector() - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 )); //because we simulate a perfect IMU
    }
}

TEST_F(ProcessorIMU_Odom_tests_details, static_Optim_IMUOdom_2KF_perturbate_positionOrigin3_UnfixAll)
{
    /* Both KeyFrames are here unfixed.
     * We perturbate all of the origin positions (Px, Py, Pz) before calling Ceres.
     * The added odometry and IMU constraints say that we did not move between both KF. We expect CERES to converge and get the same position for both KF
     * Since IMU constraint says that we did not move at all, we also expect the velocities to be equal in both KF.
     * The orientation should also be the same in both KeyFrames.
     *
     * Finally, we expect the Acceleration and Gyroscope Bias to be equal in both KeyFrames (Zero Vector)
     */

    perturbated_origin_state = initial_origin_state;
    perturbated_origin_state(0) += 1.0;
    perturbated_origin_state(1) += 2.0;
    perturbated_origin_state(2) += 3.0;

    origin_KF->setState(perturbated_origin_state);
    last_KF->setState(initial_final_state);

    origin_KF->unfix(); 
    last_KF->unfix();

    summary = ceres_manager_wolf_diff->solve();
    //std::cout << summary.BriefReport() << std::endl;

    ASSERT_TRUE( (last_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 )) << 
    "last position state : " << last_KF->getPPtr()->getVector().transpose() << "\n origin position state : " << origin_KF->getPPtr()->getVector().transpose() << std::endl;;
    ASSERT_TRUE( (last_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*1000 ));
    ASSERT_TRUE( (last_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1,  0.00000001 ));
    ASSERT_TRUE( (last_KF->getAccBiasPtr()->getVector() - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 )); 
    ASSERT_TRUE( (last_KF->getGyroBiasPtr()->getVector() - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 )); 

    /* Here we gave an initial position and final position. Everything is unfixed. We did not move between both KF according to
     * odometry constraint. There is no reason for the origin KF to impose its values to last in an optimization point of view.
     * the minimal cost should be somwhere between both keyframe positions.
     */
}

TEST_F(ProcessorIMU_Odom_tests_details, static_Optim_IMUOdom_2KF_perturbate_positionLast3_UnfixAll)
{
    /* Both KeyFrames are here unfixed.
     * We perturbate all of the final positions (Px, Py, Pz) before calling Ceres.
     * The added odometry and IMU constraints say that we did not move between both KF. We expect CERES to converge and get the same position for both KF
     * Since IMU constraint says that we did not move at all, we also expect the velocities to be equal in both KF.
     * The orientation should also be the same in both KeyFrames.
     *
     * Finally, we expect the Acceleration and Gyroscope Bias to be equal in both KeyFrames (Zero Vector)
     */
    perturbated_final_state = initial_final_state;
    perturbated_final_state(0) += 1.0;
    perturbated_final_state(1) += 2.0;
    perturbated_final_state(2) += 3.0;

    origin_KF->setState(initial_origin_state);
    last_KF->setState(perturbated_final_state);

    origin_KF->unfix(); 
    last_KF->unfix();

    summary = ceres_manager_wolf_diff->solve();
    //std::cout << summary.BriefReport() << std::endl;

    ASSERT_TRUE( (last_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 )) << 
    "last position state : " << last_KF->getPPtr()->getVector().transpose() << "\n origin position state : " << origin_KF->getPPtr()->getVector().transpose() << std::endl;;
    ASSERT_TRUE( (last_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*1000 ));
    ASSERT_TRUE( (last_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1,  0.00000001 ));
    ASSERT_TRUE( (last_KF->getAccBiasPtr()->getVector() - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 )); 
    ASSERT_TRUE( (last_KF->getGyroBiasPtr()->getVector() - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 )); 

    /* Here we gave an initial position and final position. Everything is unfixed. We did not move between both KF according to
     * odometry constraint. There is no reason for the origin KF to impose its values to last in an optimization point of view.
     * the minimal cost should be somwhere between both keyframe positions.
     */
}

TEST_F(ProcessorIMU_Odom_tests_details, static_Optim_IMUOdom_2KF_perturbate_positionOrigin3_FixLast)
{
    /* origin_KF is here unfixed. last_KF is fixed.
     * We perturbate all of the origin positions (Px, Py, Pz) before calling Ceres.
     * The added odometry and IMU constraints say that we did not move between both KF. We expect CERES to converge and get the same position for both KF
     * Since IMU constraint says that we did not move at all, we also expect the velocities to be equal in both KF.
     * The orientation should also be the same in both KeyFrames.
     *
     * Finally, we expect the Acceleration and Gyroscope Bias to be equal in both KeyFrames (Zero Vector)
     */

    perturbated_origin_state = initial_origin_state;
    perturbated_origin_state(0) += 1.0;
    perturbated_origin_state(1) += 2.0;
    perturbated_origin_state(2) += 3.0;

    origin_KF->setState(perturbated_origin_state);
    last_KF->setState(initial_final_state);

    origin_KF->unfix();
    last_KF->fix();

    summary = ceres_manager_wolf_diff->solve();
    //std::cout << summary.BriefReport() << std::endl;

    ASSERT_TRUE( (last_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS*0.001 )) << 
    "last position state : " << last_KF->getPPtr()->getVector().transpose() << "\n origin position state : " << origin_KF->getPPtr()->getVector().transpose() << std::endl;;
    ASSERT_TRUE( (last_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*1000 ));
    ASSERT_TRUE( (last_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1,  wolf::Constants::EPS)) << 
    "last velocity state : " << last_KF->getVPtr()->getVector().transpose() << "\n origin velocity state : " << origin_KF->getVPtr()->getVector().transpose() << std::endl;
}

TEST_F(ProcessorIMU_Odom_tests_details, static_Optim_IMUOdom_2KF_perturbate_positionlast3_FixOrigin)
{
    /* last_KF is here unfixed. origin_KF is fixed.
     * We perturbate all of the last positions (Px, Py, Pz) before calling Ceres.
     * The added odometry and IMU constraints say that we did not move between both KF. We expect CERES to converge and get the same position for both KF
     * Since IMU constraint says that we did not move at all, we also expect the velocities to be equal in both KF.
     * The orientation should also be the same in both KeyFrames.
     *
     * Finally, we expect the Acceleration and Gyroscope Bias to be equal in both KeyFrames (Zero Vector)
     */

    perturbated_final_state = initial_final_state;
    perturbated_final_state(0) += 1.0;
    perturbated_final_state(1) += 2.0;
    perturbated_final_state(2) += 3.0;

    origin_KF->setState(initial_origin_state);
    last_KF->setState(perturbated_final_state);

    origin_KF->fix();
    last_KF->unfix();

    summary = ceres_manager_wolf_diff->solve();
    //std::cout << summary.BriefReport() << std::endl;

    ASSERT_TRUE( (last_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS*0.001 )) << 
    "last position state : " << last_KF->getPPtr()->getVector().transpose() << "\n origin position state : " << origin_KF->getPPtr()->getVector().transpose() << std::endl;;
    ASSERT_TRUE( (last_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*1000 ));
    ASSERT_TRUE( (last_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1,  wolf::Constants::EPS )) << 
    "last velocity state : " << last_KF->getVPtr()->getVector().transpose() << "\n origin velocity state : " << origin_KF->getVPtr()->getVector().transpose() << std::endl;
    ASSERT_TRUE( (last_KF->getAccBiasPtr()->getVector() - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )); 
    ASSERT_TRUE( (last_KF->getGyroBiasPtr()->getVector() - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )); 
}


TEST_F(ProcessorIMU_Odom_tests_details, static_Optim_IMUOdom_2KF_perturbate_velocityOrigin1_Unfix1)
{
    /* Here we have only 1 StateBlock unfixed (Plus the velocity StateBlock from origin_KF that we perturbate)
     * only 1 component of origin_KF velocity is perturbated here : first we perturbate Vx, then Vy and finally Vz
     *
     * Odom and IMU contraints say that the 'robot' did not move between both KeyFrames.
     * So we expect CERES to converge so that origin_KF (=) last_KF meaning that all the stateBlocks should ideally be equal and at the origin..
     */

    for(int pert_index = 7; pert_index<10; pert_index++)
    {
        //perturate initial state
        perturbated_origin_state = initial_origin_state;
        perturbated_origin_state(pert_index) += 1.0;

        for(int i = 0; i<originStateBlock_vec.size(); i++)
        {
            origin_KF->setState(perturbated_origin_state);
            last_KF->setState(initial_final_state);

            origin_KF->fix();
            last_KF->fix();

            //we unfix origin velocity stateblock to let it converge
            originStateBlock_vec[2]->unfix();
            
            //we now unfix one StateBlock
            originStateBlock_vec[i]->unfix();

            summary = ceres_manager_wolf_diff->solve();
            //std::cout << summary.BriefReport() << std::endl;

            ASSERT_TRUE( (last_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 )) << 
            "last position state : " << last_KF->getPPtr()->getVector().transpose() << "\n origin position state : " << origin_KF->getPPtr()->getVector().transpose() << std::endl;;
            ASSERT_TRUE( (last_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*1000 ));
            ASSERT_TRUE( (last_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1,  0.00000001 ));
            ASSERT_TRUE( (last_KF->getAccBiasPtr()->getVector() - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 ));
            ASSERT_TRUE( (last_KF->getGyroBiasPtr()->getVector() - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 ));
        }

        for(int i = 0; i<finalStateBlock_vec.size(); i++)
        {
            origin_KF->setState(perturbated_origin_state);
            last_KF->setState(initial_final_state);

            origin_KF->fix();
            last_KF->fix();

            //we unfix origin VELOCITY stateblock to let it converge
            originStateBlock_vec[2]->unfix();

            //we now unfix only one StateBlock
            finalStateBlock_vec[i]->unfix();

            summary = ceres_manager_wolf_diff->solve();
            //std::cout << summary.BriefReport() << std::endl;

            ASSERT_TRUE( (last_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 )) << 
            "last position state : " << last_KF->getPPtr()->getVector().transpose() << "\n origin position state : " << origin_KF->getPPtr()->getVector().transpose() << std::endl;;
            ASSERT_TRUE( (last_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*1000 ));
            ASSERT_TRUE( (last_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1,  0.00000001 ));
            ASSERT_TRUE( (last_KF->getAccBiasPtr()->getVector() - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 ));
            ASSERT_TRUE( (last_KF->getGyroBiasPtr()->getVector() - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 ));
        }
    }
}


TEST_F(ProcessorIMU_Odom_tests_details, static_Optim_IMUOdom_2KF_perturbate_velocityOrigin2_Unfix1)
{
    /* Here we have only 1 StateBlock unfixed (Plus the velocity StateBlock from origin_KF that we perturbate)
     * 2 components of origin_KF velocity are perturbated here
     *
     * Odom and IMU contraints say that the 'robot' did not move between both KeyFrames.
     * So we expect CERES to converge so that origin_KF (=) last_KF meaning that all the stateBlocks should ideally be equal and at the origin..
     */

    for(int pert_index0 = 7; pert_index0<10; pert_index0++)
    {
        for(int pert_index1 = 8; pert_index1<10; pert_index1++)
        {
            //perturbate initial state
            perturbated_origin_state = initial_origin_state;
            perturbated_origin_state(pert_index0) += 1.0;
            perturbated_origin_state(pert_index1) += 1.0;

            for(int i = 1; i<originStateBlock_vec.size(); i++)
            {
                origin_KF->setState(perturbated_origin_state);
                last_KF->setState(initial_final_state);

                origin_KF->fix();
                last_KF->fix();

                //we unfix origin VELOCITY stateblock to let it converge
                originStateBlock_vec[2]->unfix();

                //we now unfix only one StateBlock
                originStateBlock_vec[i]->unfix();

                summary = ceres_manager_wolf_diff->solve();
                //std::cout << summary.BriefReport() << std::endl;

                ASSERT_TRUE( (last_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 )) << 
                "last position state : " << last_KF->getPPtr()->getVector().transpose() << "\n origin position state : " << origin_KF->getPPtr()->getVector().transpose() << std::endl;
                ASSERT_TRUE( (last_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1,  0.00000001 )) <<
                "origin orientation : " << last_KF->getOPtr()->getVector().transpose() << "\n last orientation : " << origin_KF->getOPtr()->getVector().transpose() << std::endl;
                ASSERT_TRUE( (last_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1,  0.00000001 )) <<
                "last velocity state : " << last_KF->getVPtr()->getVector().transpose() << "\n origin velocity state : " << origin_KF->getVPtr()->getVector().transpose() << std::endl;
                ASSERT_TRUE( (last_KF->getAccBiasPtr()->getVector() - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 ));
                ASSERT_TRUE( (last_KF->getGyroBiasPtr()->getVector() - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 ));
            }

            for(int i = 1; i<finalStateBlock_vec.size(); i++)
            {
                origin_KF->setState(perturbated_origin_state);
                last_KF->setState(initial_final_state);

                origin_KF->fix();
                last_KF->fix();

                //we unfix origin position stateblock to let it converge
                originStateBlock_vec[2]->unfix();

                //we now unfix only one StateBlock
                finalStateBlock_vec[i]->unfix();

                summary = ceres_manager_wolf_diff->solve();
                //std::cout << summary.BriefReport() << std::endl;

                ASSERT_TRUE( (last_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 )) << 
                "last position state : " << last_KF->getPPtr()->getVector().transpose() << "\n origin position state : " << origin_KF->getPPtr()->getVector().transpose() << std::endl;;
                ASSERT_TRUE( (last_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*1000 ));
                ASSERT_TRUE( (last_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1,  0.00000001 ));
                ASSERT_TRUE( (last_KF->getAccBiasPtr()->getVector() - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 ));
                ASSERT_TRUE( (last_KF->getGyroBiasPtr()->getVector() - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 ));
            }
        }
    }
}

TEST_F(ProcessorIMU_Odom_tests_details, static_Optim_IMUOdom_2KF_perturbate_velocityOrigin3_Unfix1)
{
        /* Here we have only 1 StateBlock unfixed (Plus the velocity StateBlock from origin_KF that we perturbate)
     * All 3 components of origin_KF velocity are perturbated here.
     *
     * Odom and IMU contraints say that the 'robot' did not move between both KeyFrames.
     * So we expect CERES to converge so that origin_KF (=) last_KF meaning that all the stateBlocks should ideally be equal and at the origin..
     */

    perturbated_origin_state = initial_origin_state;
    perturbated_origin_state(7) += 1.0;
    perturbated_origin_state(8) += 2.0;
    perturbated_origin_state(9) += 3.0;

    for(int i = 1; i<originStateBlock_vec.size(); i++)
    {
        origin_KF->setState(perturbated_origin_state);
        last_KF->setState(initial_final_state);

        origin_KF->fix();
        last_KF->fix();

        //we unfix origin VELOCITY stateblock to let it converge
        originStateBlock_vec[2]->unfix();

        //we now unfix only one StateBlock
        originStateBlock_vec[i]->unfix();

        summary = ceres_manager_wolf_diff->solve();
        //std::cout << summary.BriefReport() << std::endl;

        ASSERT_TRUE( (last_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 )) << 
        "last position state : " << last_KF->getPPtr()->getVector().transpose() << "\n origin position state : " << origin_KF->getPPtr()->getVector().transpose() << std::endl;;
        ASSERT_TRUE( (last_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*1000 ));
        ASSERT_TRUE( (last_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1,  0.00000001 ));
        ASSERT_TRUE( (last_KF->getAccBiasPtr()->getVector() - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 )); 
        ASSERT_TRUE( (last_KF->getGyroBiasPtr()->getVector() - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 )); 
        ASSERT_TRUE( (last_KF->getPPtr()->getVector() - (Eigen::Vector3s()<<0,0,0).finished()).isMuchSmallerThan(1, 0.0000001 )) << 
        "last position state : " << last_KF->getPPtr()->getVector().transpose() << std::endl;
        ASSERT_TRUE( (last_KF->getOPtr()->getVector() - (Eigen::Vector4s()<<0,0,0,1).finished()).isMuchSmallerThan(1, 0.0000001 ));
        ASSERT_TRUE( (last_KF->getVPtr()->getVector() - (Eigen::Vector3s()<<0,0,0).finished()).isMuchSmallerThan(1, 0.0000001 ));
    }

    for(int i = 1; i<finalStateBlock_vec.size(); i++)
    {
        origin_KF->setState(perturbated_origin_state);
        last_KF->setState(initial_final_state);

        origin_KF->fix();
        last_KF->fix();

        //we unfix origin velocity stateblock to let it converge
        originStateBlock_vec[2]->unfix();

        //we now unfix only one StateBlock
        finalStateBlock_vec[i]->unfix();

        summary = ceres_manager_wolf_diff->solve();
        //std::cout << summary.BriefReport() << std::endl;

        ASSERT_TRUE( (last_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 )) << 
        "last position state : " << last_KF->getPPtr()->getVector().transpose() << "\n origin position state : " << origin_KF->getPPtr()->getVector().transpose() << std::endl;
        ASSERT_TRUE( (last_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*1000 ));
        ASSERT_TRUE( (last_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1,  0.00000001 ));
        ASSERT_TRUE( (last_KF->getAccBiasPtr()->getVector() - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 )); //because we simulate a perfect IMU
        ASSERT_TRUE( (last_KF->getGyroBiasPtr()->getVector() - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 )); //because we simulate a perfect IMU
        ASSERT_TRUE( (last_KF->getPPtr()->getVector() - (Eigen::Vector3s()<<0,0,0).finished()).isMuchSmallerThan(1, 0.0000001 )) << 
        "last position state : " << last_KF->getPPtr()->getVector().transpose() << std::endl;
        ASSERT_TRUE( (last_KF->getOPtr()->getVector() - (Eigen::Vector4s()<<0,0,0,1).finished()).isMuchSmallerThan(1, 0.0000001 ));
        ASSERT_TRUE( (last_KF->getVPtr()->getVector() - (Eigen::Vector3s()<<0,0,0).finished()).isMuchSmallerThan(1, 0.0000001 ));
    }
}

TEST_F(ProcessorIMU_Odom_tests_details, static_Optim_IMUOdom_2KF_perturbate_velocityLast3_Unfix1)
{
    /* Here we have only 1 StateBlock unfixed (Plus the velocity StateBlock of last_KF that we perturbate)
     * All 3 components of last_KF velocity are perturbated here.
     *
     * Odom and IMU contraints say that the 'robot' did not move between both KeyFrames.
     * So we expect CERES to converge so that origin_KF (=) last_KF meaning that all the stateBlocks should ideally be equal and at the origin..
     */

    //perturate initial state
    perturbated_final_state = initial_final_state;
    perturbated_final_state(7) += 1.0;
    perturbated_final_state(8) += 2.0;
    perturbated_final_state(9) += 3.0;

    for(int i = 1; i<originStateBlock_vec.size(); i++)
    {
        origin_KF->setState(initial_origin_state);
        last_KF->setState(perturbated_final_state);

        origin_KF->fix();
        last_KF->fix();

        //we unfix origin velocity stateblock to let it converge
        finalStateBlock_vec[2]->unfix();

        //we now unfix only one StateBlock
        originStateBlock_vec[i]->unfix();

        summary = ceres_manager_wolf_diff->solve();
        //std::cout << summary.BriefReport() << std::endl;

        ASSERT_TRUE( (last_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 )) << 
        "last position state : " << last_KF->getPPtr()->getVector().transpose() << "\n origin position state : " << origin_KF->getPPtr()->getVector().transpose() << std::endl;;
        ASSERT_TRUE( (last_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*1000 ));
        ASSERT_TRUE( (last_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1,  0.00000001 )) << 
        "last velocity state : " << last_KF->getVPtr()->getVector().transpose() << "\n origin velocity state : " << origin_KF->getVPtr()->getVector().transpose() << std::endl;
        ASSERT_TRUE( (last_KF->getAccBiasPtr()->getVector() - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 )); 
        ASSERT_TRUE( (last_KF->getGyroBiasPtr()->getVector() - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 )); 
        ASSERT_TRUE( (last_KF->getPPtr()->getVector() - (Eigen::Vector3s()<<0,0,0).finished()).isMuchSmallerThan(1, 0.0000001 )) << 
        "last position state : " << last_KF->getPPtr()->getVector().transpose() << std::endl;
        ASSERT_TRUE( (last_KF->getOPtr()->getVector() - (Eigen::Vector4s()<<0,0,0,1).finished()).isMuchSmallerThan(1, 0.0000001 ));
        ASSERT_TRUE( (last_KF->getVPtr()->getVector() - (Eigen::Vector3s()<<0,0,0).finished()).isMuchSmallerThan(1, 0.0000001 ));
    }

    for(int i = 1; i<finalStateBlock_vec.size(); i++)
    {
        origin_KF->setState(initial_origin_state);
        last_KF->setState(perturbated_final_state);

        origin_KF->fix(); 
        last_KF->fix();

        //we unfix origin velocity stateblock to let it converge
        finalStateBlock_vec[2]->unfix();

        //we now unfix only one StateBlock
        finalStateBlock_vec[i]->unfix();

        summary = ceres_manager_wolf_diff->solve();
        //std::cout << summary.BriefReport() << std::endl;

        EXPECT_TRUE( (last_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 )) << 
        "last position state : " << last_KF->getPPtr()->getVector().transpose() << "\n origin position state : " << origin_KF->getPPtr()->getVector().transpose() << std::endl;;
        ASSERT_TRUE( (last_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*1000 ));
        EXPECT_TRUE( (last_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1,  0.00000001 )) <<
        "last velocity state : " << last_KF->getVPtr()->getVector().transpose() << "\n origin velocity state : " << origin_KF->getVPtr()->getVector().transpose() << std::endl;
        ASSERT_TRUE( (last_KF->getAccBiasPtr()->getVector() - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 )); 
        ASSERT_TRUE( (last_KF->getGyroBiasPtr()->getVector() - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 ));
    }
}

TEST_F(ProcessorIMU_Odom_tests_details, static_Optim_IMUOdom_2KF_perturbate_velocityOrigin3_UnfixAll)
{
    /* Both KeyFrames are unfixed
     * All 3 components of origin_KF velocity are perturbated here.
     *
     * Odom and IMU contraints say that the 'robot' did not move between both KeyFrames.
     * So we expect CERES to converge so that origin_KF (=) last_KF meaning that all the stateBlocks should ideally be equal and at the origin..
     */

    perturbated_origin_state = initial_origin_state;
    perturbated_origin_state(7) += 1.0;
    perturbated_origin_state(8) += 2.0;
    perturbated_origin_state(9) += 3.0;

    origin_KF->setState(perturbated_origin_state);
    last_KF->setState(initial_final_state);

    origin_KF->unfix();
    last_KF->unfix();

    summary = ceres_manager_wolf_diff->solve();
    //std::cout << summary.BriefReport() << std::endl;

    ASSERT_TRUE( (last_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 )) << 
    "last position state : " << last_KF->getPPtr()->getVector().transpose() << "\n origin position state : " << origin_KF->getPPtr()->getVector().transpose() << std::endl;
    ASSERT_TRUE( (last_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*1000 ));
    EXPECT_FALSE( (last_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1,  0.00000001 )) << 
    "last velocity state : " << last_KF->getVPtr()->getVector().transpose() << "\n origin velocity state : " << origin_KF->getVPtr()->getVector().transpose() << std::endl;
    EXPECT_FALSE( (last_KF->getAccBiasPtr()->getVector() - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 )) << 
    "last acc bias : " << last_KF->getAccBiasPtr()->getVector().transpose() << "\n origin acc bias : " << origin_KF->getAccBiasPtr()->getVector().transpose() << std::endl; 
    ASSERT_TRUE( (last_KF->getGyroBiasPtr()->getVector() - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 )); 

    /* Here we gave an initial velocity and a final velocity. Everything is unfixed. We did not move between both KF according to
     * odometry constraint. There is no reason for the origin KF to impose its values to last in an optimization point of view.
     * the minimal cost should be somewhere between both keyframe velocities. The Robot couldhave done anything between both KF
     *
     * Robot could have done anything. Velocity changed. So acc bias could also change to anything... But gyroscope bias should still be 0
     */
}

TEST_F(ProcessorIMU_Odom_tests_details, static_Optim_IMUOdom_2KF_perturbate_velocityLast3_UnfixAll)
{
    /* Both KeyFrames are unfixed
     * All 3 components of last_KF velocity are perturbated here.
     *
     * Odom and IMU contraints say that the 'robot' did not move between both KeyFrames.
     * So we expect CERES to converge so that origin_KF (=) last_KF meaning that all the stateBlocks should ideally be equal and at the origin..
     */

    perturbated_final_state = initial_final_state;
    perturbated_final_state(7) += 1.0;
    perturbated_final_state(8) += 2.0;
    perturbated_final_state(9) += 3.0;

    origin_KF->setState(initial_origin_state);
    last_KF->setState(perturbated_final_state);

    origin_KF->unfix();
    last_KF->unfix();

    summary = ceres_manager_wolf_diff->solve();
    //std::cout << summary.BriefReport() << std::endl;

    ASSERT_TRUE( (last_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 )) << 
    "last position state : " << last_KF->getPPtr()->getVector().transpose() << "\n origin position state : " << origin_KF->getPPtr()->getVector().transpose() << std::endl;
    ASSERT_TRUE( (last_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*1000 ));
    EXPECT_FALSE( (last_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1,  0.00000001 )) << 
    "last velocity state : " << last_KF->getVPtr()->getVector().transpose() << "\n origin velocity state : " << origin_KF->getVPtr()->getVector().transpose() << std::endl;
    EXPECT_FALSE( (last_KF->getAccBiasPtr()->getVector() - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 )) << 
    "last acc bias : " << last_KF->getAccBiasPtr()->getVector().transpose() << "\n origin acc bias : " << origin_KF->getAccBiasPtr()->getVector().transpose() << std::endl; 
    ASSERT_TRUE( (last_KF->getGyroBiasPtr()->getVector() - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 )); 

    /* Here we gave an initial velocity and final velocity. Everything is unfixed. We did not move between both KF according to
     * odometry constraint. But we perturbated the final velocities before optimization.
     * There is no reason for the origin KF to impose its values to last in an optimization point of view.
     * the minimal cost should be somewhere between both keyframe velocities.
     * 
     * Acceleration bias can also change here. And in fact it will. We may wonder what happens if we only fix acceleration biases (in origin and last KF) :
     * We can suppose that velocities will be changed even more. The difference in velocity cannot be compensated in acceleration biases but both origin_KF and last_KF
     * velocities can be changed so that the 'robot' does not move. So we can either expect the velocities to be 0, or we can expect the velocities to be contrary.
     * But due to IMU constraint saying 'no acceleration' we expect both velocity StateBlocks to be null.
     * We will try this case in test TEST 9
     */
}

TEST_F(ProcessorIMU_Odom_tests_details, static_Optim_IMUOdom_2KF_perturbate_velocityOrigin3_FixLast)
{
    /* last_KF is fixed 
     * All 3 components of origin_KF velocity are perturbated here.
     *
     * Odom and IMU contraints say that the 'robot' did not move between both KeyFrames.
     * So we expect CERES to converge so that origin_KF (=) last_KF meaning that all the stateBlocks should ideally be equal and at the origin..
     */

    perturbated_origin_state = initial_origin_state;
    perturbated_origin_state(7) += 1.0;
    perturbated_origin_state(8) += 2.0;
    perturbated_origin_state(9) += 3.0;

    origin_KF->setState(perturbated_origin_state);
    last_KF->setState(initial_final_state);

    origin_KF->unfix();
    last_KF->fix();

    summary = ceres_manager_wolf_diff->solve();
    //std::cout << summary.BriefReport() << std::endl;

    ASSERT_TRUE( (last_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS*0.001 )) << 
    "last position state : " << last_KF->getPPtr()->getVector().transpose() << "\n origin position state : " << origin_KF->getPPtr()->getVector().transpose() << std::endl;;
    ASSERT_TRUE( (last_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*1000 ));
    ASSERT_TRUE( (last_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1,  wolf::Constants::EPS)) << 
    "last velocity state : " << last_KF->getVPtr()->getVector().transpose() << "\n origin velocity state : " << origin_KF->getVPtr()->getVector().transpose() << std::endl;
    ASSERT_TRUE( (last_KF->getAccBiasPtr()->getVector() - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )); 
    ASSERT_TRUE( (last_KF->getGyroBiasPtr()->getVector() - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )); 
}

TEST_F(ProcessorIMU_Odom_tests_details, static_Optim_IMUOdom_2KF_perturbate_velocityLast3_FixOrigin)
{
    /* we only fixed acceleration biases (in origin and last KF) :
     * We can suppose that velocities will be changed even more. The difference in velocity cannot be compensated in acceleration biases but both origin_KF and last_KF
     * velocities can be changed so that the 'robot' does not move. So we can either expect the velocities to be 0, or we can expect the velocities to be contrary.
     * But due to IMU constraint saying 'no acceleration' we expect both velocity StateBlocks to be null.
    */

    perturbated_final_state = initial_final_state;
    perturbated_final_state(7) += 1.0;
    perturbated_final_state(8) += 2.0;
    perturbated_final_state(9) += 3.0;

    origin_KF->setState(initial_origin_state);
    last_KF->setState(perturbated_final_state);

    origin_KF->fix();
    last_KF->unfix();

    summary = ceres_manager_wolf_diff->solve();
    //std::cout << summary.BriefReport() << std::endl;

    ASSERT_TRUE( (last_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS*0.001 )) << 
    "last position state : " << last_KF->getPPtr()->getVector().transpose() << "\n origin position state : " << origin_KF->getPPtr()->getVector().transpose() << std::endl;;
    ASSERT_TRUE( (last_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*1000 ));
    ASSERT_TRUE( (last_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1,  wolf::Constants::EPS )) << 
    "last velocity state : " << last_KF->getVPtr()->getVector().transpose() << "\n origin velocity state : " << origin_KF->getVPtr()->getVector().transpose() << std::endl;
    ASSERT_TRUE( (last_KF->getAccBiasPtr()->getVector() - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )); 
    ASSERT_TRUE( (last_KF->getGyroBiasPtr()->getVector() - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )); 
}

TEST_F(ProcessorIMU_Odom_tests_details, static_Optim_IMUOdom_2KF_perturbate_velocityLast3_FixAccBiaseS)
{
    /* Both KeyFrames are unfixed here. however, all Acceleration bias StateBlocks are fixed.
     * All 3 components of last_KF velocity are perturbated here.
     *
     * Odom and IMU contraints say that the 'robot' did not move between both KeyFrames.
     * So we expect CERES to converge so that origin_KF (=) last_KF meaning that all the stateBlocks should ideally be equal and at the origin..
     */

    perturbated_final_state = initial_final_state;
    perturbated_final_state(7) += 1.0;
    perturbated_final_state(8) += 2.0;
    perturbated_final_state(9) += 3.0;

    origin_KF->setState(initial_origin_state);
    last_KF->setState(perturbated_final_state);

    origin_KF->unfix(); 
    last_KF->unfix();

    //fix acceleration biases
    originStateBlock_vec[3]->fix();
    finalStateBlock_vec[3]->fix();

    summary = ceres_manager_wolf_diff->solve();
    //std::cout << summary.BriefReport() << std::endl;

    ASSERT_TRUE( (last_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 )) << 
    "last position state : " << last_KF->getPPtr()->getVector().transpose() << "\n origin position state : " << origin_KF->getPPtr()->getVector().transpose() << std::endl;
    ASSERT_TRUE( (last_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*1000 ));
    EXPECT_TRUE( (last_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1,  wolf::Constants::EPS*100 )) << 
    "last velocity state : " << last_KF->getVPtr()->getVector().transpose() << "\n origin velocity state : " << origin_KF->getVPtr()->getVector().transpose() << std::endl;
    ASSERT_TRUE( (last_KF->getAccBiasPtr()->getVector() - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 )) << 
    "last acc bias : " << last_KF->getAccBiasPtr()->getVector().transpose() << "\n origin acc bias : " << origin_KF->getAccBiasPtr()->getVector().transpose() << std::endl; 
    ASSERT_TRUE( (last_KF->getGyroBiasPtr()->getVector() - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 ));

    // As expected, both velocity StateBlocks converge to 0. The error is in 1e-6
}   

TEST_F(ProcessorIMU_Odom_tests_details, static_Optim_IMUOdom_2KF_perturbate_orientation1Origin_Unfix1)
{
    /* Both KeyFrames are fixed. We unfix 1 stateblock among those we have in these 2 KeyFrames.
     * We perturbate 1 orientation (ox, oy, oz) in origin_KF. ==> We also unfix origin_KF's quaternion StateBlock.
     * The perturbation is introduced in this quaternion stateblocks using q * v2q(rotation_vector_perturbation)
     *
     * Odom and IMU contraints say that the 'robot' did not move between both KeyFrames.
     * So we expect CERES to converge so that origin_KF (=) last_KF meaning that all the stateBlocks should ideally be equal and at the origin..
     */

    for(int pert_index = 0; pert_index<3; pert_index++)
    {
        Eigen::Vector3s orientation_perturbation((Eigen::Vector3s()<<0,0,0).finished());
        perturbated_origin_state = initial_origin_state;
        Eigen::Map<Eigen::Quaternions> quat_map(perturbated_origin_state.data() + 3);
        orientation_perturbation(pert_index) += 1.0;

        //introduce the perturbation directly in the quaternion StateBlock
        quat_map = quat_map * v2q(orientation_perturbation);

        for(int i = 0; i<originStateBlock_vec.size(); i++)
        {
            origin_KF->setState(perturbated_origin_state);
            last_KF->setState(initial_final_state);

            origin_KF->fix();
            last_KF->fix();

            //we unfix origin orientation stateblock to let it converge
            originStateBlock_vec[1]->unfix();
            
            //we now unfix one StateBlock
            originStateBlock_vec[i]->unfix();

            summary = ceres_manager_wolf_diff->solve();
            //std::cout << summary.BriefReport() << std::endl;

            ASSERT_TRUE( (last_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 )) << 
            "last position state : " << last_KF->getPPtr()->getVector().transpose() << "\n origin position state : " << origin_KF->getPPtr()->getVector().transpose() << std::endl;;
            ASSERT_TRUE( (last_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS ));
            ASSERT_TRUE( (last_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1,  0.00000001 ));
            ASSERT_TRUE( (last_KF->getAccBiasPtr()->getVector() - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 ));
            ASSERT_TRUE( (last_KF->getGyroBiasPtr()->getVector() - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 ));
        }

        for(int i = 0; i<finalStateBlock_vec.size(); i++)
        {
            origin_KF->setState(perturbated_origin_state);
            last_KF->setState(initial_final_state);

            origin_KF->fix();
            last_KF->fix();

            //we unfix origin orientation stateblock to let it converge
            originStateBlock_vec[1]->unfix();

            //we now unfix only one StateBlock
            finalStateBlock_vec[i]->unfix();

            summary = ceres_manager_wolf_diff->solve();
            //std::cout << summary.BriefReport() << std::endl;

            ASSERT_TRUE( (last_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 )) << 
            "last position state : " << last_KF->getPPtr()->getVector().transpose() << "\n origin position state : " << origin_KF->getPPtr()->getVector().transpose() << std::endl;;
            ASSERT_TRUE( (last_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS ));
            ASSERT_TRUE( (last_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1,  0.00000001 ));
            ASSERT_TRUE( (last_KF->getAccBiasPtr()->getVector() - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 ));
            ASSERT_TRUE( (last_KF->getGyroBiasPtr()->getVector() - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 ));
            ASSERT_TRUE( (last_KF->getPPtr()->getVector() - (Eigen::Vector3s()<<0,0,0).finished()).isMuchSmallerThan(1, 0.0000001 )) << 
            "last position state : " << last_KF->getPPtr()->getVector().transpose() << std::endl;
            // ifgyro bias stateBlock is the only one unfixed. it will be changed so that conditions can be met on orientation.
            ASSERT_TRUE( (last_KF->getVPtr()->getVector() - (Eigen::Vector3s()<<0,0,0).finished()).isMuchSmallerThan(1, 0.0000001 ));
        }
    }
}


TEST_F(ProcessorIMU_Odom_tests_details, static_Optim_IMUOdom_2KF_perturbate_orientation2Origin_Unfix1)
{
    /* Both KeyFrames are fixed. We unfix 1 stateblock among those we have in these 2 KeyFrames.
     * We perturbate 2 angles (ox, oy, oz) in origin_KF. ==> We also unfix origin_KF's quaternion StateBlock.
     * The perturbation is introduced in this quaternion stateblocks using q * v2q(rotation_vector_perturbation)
     *
     * Odom and IMU contraints say that the 'robot' did not move between both KeyFrames.
     * So we expect CERES to converge so that origin_KF (=) last_KF meaning that all the stateBlocks should ideally be equal and at the origin..
     */

    for(int pert_index0 = 0; pert_index0<3; pert_index0++)
    {
        for(int pert_index1 = 1; pert_index1<3; pert_index1++)
        {
            //perturate initial state
            Eigen::Vector3s orientation_perturbation((Eigen::Vector3s()<<0,0,0).finished());
            perturbated_origin_state = initial_origin_state;

            Eigen::Map<Eigen::Quaternions> quat_map(perturbated_origin_state.data() + 3);
            orientation_perturbation(pert_index0) += 1.0;
            orientation_perturbation(pert_index1) += 1.0;

            //introduce the perturbation directly in the quaternion StateBlock
            quat_map = quat_map * v2q(orientation_perturbation);

            for(int i = 0; i<originStateBlock_vec.size(); i++)
            {
                origin_KF->setState(perturbated_origin_state);
                last_KF->setState(initial_final_state);

                origin_KF->fix();
                last_KF->fix();

                //we unfix origin ORIENTATION stateblock to let it converge
                originStateBlock_vec[1]->unfix();
                
                //we now unfix only one StateBlock
                originStateBlock_vec[i]->unfix();

                summary = ceres_manager_wolf_diff->solve();
                //std::cout << summary.BriefReport() << std::endl;

                ASSERT_TRUE( (last_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 )) << 
                "last position state : " << last_KF->getPPtr()->getVector().transpose() << "\n origin position state : " << origin_KF->getPPtr()->getVector().transpose() << std::endl;
                ASSERT_TRUE( (last_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1,  0.00000001 )) <<
                "origin orientation : " << last_KF->getOPtr()->getVector().transpose() << "\n last orientation : " << origin_KF->getOPtr()->getVector().transpose() << std::endl;
                ASSERT_TRUE( (last_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1,  0.00000001 )) <<
                "last velocity state : " << last_KF->getVPtr()->getVector().transpose() << "\n origin velocity state : " << origin_KF->getVPtr()->getVector().transpose() << std::endl;
                ASSERT_TRUE( (last_KF->getAccBiasPtr()->getVector() - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 ));
                ASSERT_TRUE( (last_KF->getGyroBiasPtr()->getVector() - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 ));
            }

            for(int i = 0; i<finalStateBlock_vec.size(); i++)
            {
                origin_KF->setState(perturbated_origin_state);
                last_KF->setState(initial_final_state);

                origin_KF->fix();
                last_KF->fix();

                //we unfix origin ORIENTATION stateblock to let it converge
                originStateBlock_vec[1]->unfix();

                //we now unfix only one StateBlock
                finalStateBlock_vec[i]->unfix();

                summary = ceres_manager_wolf_diff->solve();
                //std::cout << summary.BriefReport() << std::endl;

                ASSERT_TRUE( (last_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 )) << 
                "last position state : " << last_KF->getPPtr()->getVector().transpose() << "\n origin position state : " << origin_KF->getPPtr()->getVector().transpose() << std::endl;;
                ASSERT_TRUE( (last_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS ));
                ASSERT_TRUE( (last_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1,  0.00000001 ));
                ASSERT_TRUE( (last_KF->getAccBiasPtr()->getVector() - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 ));
                ASSERT_TRUE( (last_KF->getGyroBiasPtr()->getVector() - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 ));
                
                ASSERT_TRUE( (last_KF->getPPtr()->getVector() - (Eigen::Vector3s()<<0,0,0).finished()).isMuchSmallerThan(1, 0.0000001 )) << 
                "last position state : " << last_KF->getPPtr()->getVector().transpose() << std::endl;
                if(i != 1) //if we do not fix origin_KF Quaternion StateBlock we cannot expect to have unit Quaternion here
                    ASSERT_TRUE( (last_KF->getOPtr()->getVector() - (Eigen::Vector4s()<<0,0,0,1).finished()).isMuchSmallerThan(1, 0.0000001 )) <<
                    "last orientation state : " << last_KF->getOPtr()->getVector().transpose() << std::endl;
                ASSERT_TRUE( (last_KF->getVPtr()->getVector() - (Eigen::Vector3s()<<0,0,0).finished()).isMuchSmallerThan(1, 0.0000001 ));
            }
        }
    }
}

TEST_F(ProcessorIMU_Odom_tests_details, static_Optim_IMUOdom_2KF_perturbate_orientation3Origin_Unfix1)
{
    /* Both KeyFrames are fixed. We unfix 1 stateblock among those we have in these 2 KeyFrames.
     * We perturbate all 3 angles (ox, oy, oz) in origin_KF. ==> We also unfix origin_KF's quaternion StateBlock.
     * The perturbation is introduced in this quaternion stateblocks using q * v2q(rotation_vector_perturbation)
     *
     * Odom and IMU contraints say that the 'robot' did not move between both KeyFrames.
     * So we expect CERES to converge so that origin_KF (=) last_KF meaning that all the stateBlocks should ideally be equal and at the origin..
     */
     WOLF_WARN("CHECK THIS !")

    Eigen::Vector3s orientation_perturbation((Eigen::Vector3s()<<0,0,0).finished());
    perturbated_origin_state = initial_origin_state;
    Eigen::Map<Eigen::Quaternions> quat_map(perturbated_origin_state.data() + 3);

    orientation_perturbation(0) = 1.0;
    orientation_perturbation(1) = 2.0;
    orientation_perturbation(2) = 0.5; //DOES NOT WORK IF = 3.0

    //introduce the perturbation directly in the quaternion StateBlock
    quat_map = quat_map * v2q(orientation_perturbation);

    for(int i = 0; i<originStateBlock_vec.size(); i++)
    {
        origin_KF->setState(perturbated_origin_state);
        last_KF->setState(initial_final_state);

        origin_KF->fix();
        last_KF->fix();
        
        //we unfix origin ORIENTATION stateblock to let it converge
        originStateBlock_vec[1]->unfix();

        //we now unfix only one StateBlock
        originStateBlock_vec[i]->unfix();

        summary = ceres_manager_wolf_diff->solve();
        //std::cout << summary.BriefReport() << std::endl;

        ASSERT_TRUE( (last_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 )) << 
        "last position state : " << last_KF->getPPtr()->getVector().transpose() << "\n origin position state : " << origin_KF->getPPtr()->getVector().transpose() << std::endl;;
        ASSERT_TRUE( (last_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS*10 )) <<
        "last orientation : " << last_KF->getOPtr()->getVector().transpose() << "\n origin orientation : " << origin_KF->getOPtr()->getVector().transpose() << std::endl;
        ASSERT_TRUE( (last_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1,  0.00000001 ));
        ASSERT_TRUE( (last_KF->getAccBiasPtr()->getVector() - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 )); 
        //if gyro bias stateBlock is the only other stateBlock that is unfixed, it will be changed to that orientation condition can be met
        /*EXPECT_TRUE( (last_KF->getGyroBiasPtr()->getVector() - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 )) << 
        "last_KF gyro bias : " << last_KF->getGyroBiasPtr()->getVector().transpose() << "\n origin_KF gyro bias : " << origin_KF->getGyroBiasPtr()->getVector().transpose() << std::endl;*/ 
        //We expect both gyroscope bias to be equal. However, if gyroscope bias stateblock is the only unfixed stateblock it will be changed.
        ASSERT_TRUE( (last_KF->getPPtr()->getVector() - (Eigen::Vector3s()<<0,0,0).finished()).isMuchSmallerThan(1, 0.0000001 )) << 
        "last position state : " << last_KF->getPPtr()->getVector().transpose() << std::endl;
        ASSERT_TRUE( (last_KF->getOPtr()->getVector() - (Eigen::Vector4s()<<0,0,0,1).finished()).isMuchSmallerThan(1, 0.0000001 )) <<
        "last orientation state : " << last_KF->getOPtr()->getVector().transpose() << std::endl;
        ASSERT_TRUE( (last_KF->getVPtr()->getVector() - (Eigen::Vector3s()<<0,0,0).finished()).isMuchSmallerThan(1, 0.0000001 ));
    }

    for(int i = 0; i<finalStateBlock_vec.size(); i++)
    {
        origin_KF->setState(perturbated_origin_state);
        last_KF->setState(initial_final_state);

        origin_KF->fix(); 
        last_KF->fix();

        //we unfix origin ORIENTATION stateblock to let it converge
        originStateBlock_vec[1]->unfix();

        //we now unfix only one StateBlock
        finalStateBlock_vec[i]->unfix();

        summary = ceres_manager_wolf_diff->solve();
        //std::cout << summary.BriefReport() << std::endl;

        ASSERT_TRUE( (last_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 )) << 
        "last position state : " << last_KF->getPPtr()->getVector().transpose() << "\n origin position state : " << origin_KF->getPPtr()->getVector().transpose() << std::endl;
        ASSERT_TRUE( (last_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS ));
        ASSERT_TRUE( (last_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1,  0.00000001 ));
        ASSERT_TRUE( (last_KF->getAccBiasPtr()->getVector() - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 )); //because we simulate a perfect IMU
        ASSERT_TRUE( (last_KF->getGyroBiasPtr()->getVector() - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 )); //because we simulate a perfect IMU
        ASSERT_TRUE( (last_KF->getPPtr()->getVector() - (Eigen::Vector3s()<<0,0,0).finished()).isMuchSmallerThan(1, 0.0000001 )) << 
        "last position state : " << last_KF->getPPtr()->getVector().transpose() << std::endl;
        if(i != 1) //if orientation block is the only one unfixed it will be changed
            ASSERT_TRUE( (last_KF->getOPtr()->getVector() - (Eigen::Vector4s()<<0,0,0,1).finished()).isMuchSmallerThan(1, 0.0000001 ));
        ASSERT_TRUE( (last_KF->getVPtr()->getVector() - (Eigen::Vector3s()<<0,0,0).finished()).isMuchSmallerThan(1, 0.0000001 ));
    }
}

TEST_F(ProcessorIMU_Odom_tests_details, static_Optim_IMUOdom_2KF_perturbate_orientation3Last_Unfix1)
{
    /* Both KeyFrames are fixed. We unfix 1 stateblock among those we have in these 2 KeyFrames.
     * We perturbate all 3 angles (ox, oy, oz) in last_KF. ==> We also unfix last_KF's quaternion StateBlock.
     * The perturbation is introduced in this quaternion stateblocks using q * v2q(rotation_vector_perturbation)
     *
     * Odom and IMU contraints say that the 'robot' did not move between both KeyFrames.
     * So we expect CERES to converge so that origin_KF (=) last_KF meaning that all the stateBlocks should ideally be equal and at the origin..
     */
    
    Eigen::Vector3s orientation_perturbation((Eigen::Vector3s()<<0,0,0).finished());
    perturbated_final_state = initial_final_state;
    Eigen::Map<Eigen::Quaternions> quat_map(perturbated_final_state.data() + 3);

    orientation_perturbation(0) = 1.0;
    orientation_perturbation(1) = 2.0;
    orientation_perturbation(2) = 1.0;

    //introduce the perturbation directly in the quaternion StateBlock
    quat_map = quat_map * v2q(orientation_perturbation);

    for(int i = 1; i<originStateBlock_vec.size(); i++)
    {
        origin_KF->setState(initial_origin_state);
        last_KF->setState(perturbated_final_state);

        origin_KF->fix(); //this fix the all keyframe
        last_KF->fix();

        //we unfix final orientation stateblock to let it converge
        finalStateBlock_vec[1]->unfix();

        //we now unfix only one StateBlock
        originStateBlock_vec[i]->unfix();

        summary = ceres_manager_wolf_diff->solve();
        //std::cout << summary.BriefReport() << std::endl;

        ASSERT_TRUE( (last_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 )) << 
        "last position state : " << last_KF->getPPtr()->getVector().transpose() << "\n origin position state : " << origin_KF->getPPtr()->getVector().transpose() << std::endl;;
        ASSERT_TRUE( (last_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS ));
        ASSERT_TRUE( (last_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1,  0.00000001 )) << 
        "last velocity state : " << last_KF->getVPtr()->getVector().transpose() << "\n origin velocity state : " << origin_KF->getVPtr()->getVector().transpose() << std::endl;
        ASSERT_TRUE( (last_KF->getAccBiasPtr()->getVector() - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 )); 
        ASSERT_TRUE( (last_KF->getGyroBiasPtr()->getVector() - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 )); 
        ASSERT_TRUE( (last_KF->getPPtr()->getVector() - (Eigen::Vector3s()<<0,0,0).finished()).isMuchSmallerThan(1, 0.0000001 )) << 
        "last position state : " << last_KF->getPPtr()->getVector().transpose() << std::endl;
        // if only orientation stateblocks are unfixed. We can hardly expect the final orienation to converge toward the one in origin_KF.
        // Both are likely to be changed to an intermediate orientation state.
        // but in the other cases, if origin orientation is fixed to identity quaternion and the robot did not move we expect the final orientation to converge to identity quaternion too
        if(i != 1) 
            ASSERT_TRUE( (last_KF->getOPtr()->getVector() - (Eigen::Vector4s()<<0,0,0,1).finished()).isMuchSmallerThan(1, 0.0000001 )) << "last orientation state : " << last_KF->getOPtr()->getVector().transpose() ;
        ASSERT_TRUE( (last_KF->getVPtr()->getVector() - (Eigen::Vector3s()<<0,0,0).finished()).isMuchSmallerThan(1, 0.0000001 ));
    }

    for(int i = 1; i<finalStateBlock_vec.size(); i++)
    {
        origin_KF->setState(initial_origin_state);
        last_KF->setState(perturbated_final_state);

        origin_KF->fix();
        last_KF->fix();

        //we unfix final orientation stateblock to let it converge
        finalStateBlock_vec[1]->unfix();

        //we now unfix only one StateBlock
        finalStateBlock_vec[i]->unfix();

        summary = ceres_manager_wolf_diff->solve();
        //std::cout << summary.BriefReport() << std::endl;

        EXPECT_TRUE( (last_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 )) << 
        "last position state : " << last_KF->getPPtr()->getVector().transpose() << "\n origin position state : " << origin_KF->getPPtr()->getVector().transpose() << std::endl;;
        ASSERT_TRUE( (last_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS ));
        EXPECT_TRUE( (last_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1,  0.00000001 )) <<
        "last velocity state : " << last_KF->getVPtr()->getVector().transpose() << "\n origin velocity state : " << origin_KF->getVPtr()->getVector().transpose() << std::endl;
        ASSERT_TRUE( (last_KF->getAccBiasPtr()->getVector() - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 )); 
        ASSERT_TRUE( (last_KF->getGyroBiasPtr()->getVector() - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 ));
    }
}

TEST_F(ProcessorIMU_Odom_tests_details, static_Optim_IMUOdom_2KF_perturbate_orientation3Origin_UnfixAll)
{
    /* Both KeyFrames are unfixed.
     * We perturbate all 3 angles (ox, oy, oz) in origin_KF.
     * The perturbation is introduced in this quaternion stateblocks using q * v2q(rotation_vector_perturbation)
     *
     * Odom and IMU contraints say that the 'robot' did not move between both KeyFrames.
     * So we expect CERES to converge so that origin_KF (=) last_KF meaning that all the stateBlocks should ideally be equal and at the origin..
     */

    Eigen::Vector3s orientation_perturbation((Eigen::Vector3s()<<0,0,0).finished());
    perturbated_origin_state = initial_origin_state;
    Eigen::Map<Eigen::Quaternions> quat_map(perturbated_origin_state.data() + 3);

    orientation_perturbation(0) = 1.0;
    orientation_perturbation(1) = 2.0;
    orientation_perturbation(2) = 3.0; //DOES NOT WORK IF = 3.0

    //introduce the perturbation directly in the quaternion StateBlock
    //quat_map = quat_map * v2q(orientation_perturbation);

    origin_KF->setState(perturbated_origin_state);
    last_KF->setState(initial_final_state);

    origin_KF->unfix();
    last_KF->unfix();

    summary = ceres_manager_wolf_diff->solve();
    //std::cout << summary.BriefReport() << std::endl;

    ASSERT_TRUE( (last_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 )) << 
    "last position state : " << last_KF->getPPtr()->getVector().transpose() << "\n origin position state : " << origin_KF->getPPtr()->getVector().transpose() << std::endl;
    EXPECT_TRUE( (last_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )) <<
    "last KF orientation : " << last_KF->getOPtr()->getVector().transpose() << "\n origin KF orientation : " << origin_KF->getOPtr()->getVector().transpose() << std::endl;
    ASSERT_TRUE( (last_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1,  0.00000001 )) << 
    "last velocity state : " << last_KF->getVPtr()->getVector().transpose() << "\n origin velocity state : " << origin_KF->getVPtr()->getVector().transpose() << std::endl;
    ASSERT_TRUE( (last_KF->getAccBiasPtr()->getVector() - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 )) << 
    "last acc bias : " << last_KF->getAccBiasPtr()->getVector().transpose() << "\n origin acc bias : " << origin_KF->getAccBiasPtr()->getVector().transpose() << std::endl; 
    EXPECT_TRUE( (last_KF->getGyroBiasPtr()->getVector() - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 )); 

    /* Here we gave an initial orientation and a final orientation. Everything is unfixed. We did not move between both KF according to
     * odometry constraint. There is no reason for the origin KF to impose its values to last in an optimization point of view.
     * the minimal cost should be somewhere between both keyframe velocities. The Robot couldhave done anything between both KF.
     * However, IMU constraint imposes no rate of turn. So we expect both velocities to be equal after optimization but biases could have been changed. Or bias is
     * unchanged and orientations have changed or both have been changed.
     */
}

TEST_F(ProcessorIMU_Odom_tests_details, static_Optim_IMUOdom_2KF_perturbate_orientation3Last_UnfixAll)
{
    /* Both KeyFrames are unfixed.
     * We perturbate all 3 angles (ox, oy, oz) in last_KF.
     * The perturbation is introduced in this quaternion stateblocks using q * v2q(rotation_vector_perturbation)
     *
     * Odom and IMU contraints say that the 'robot' did not move between both KeyFrames.
     * So we expect CERES to converge so that origin_KF (=) last_KF meaning that all the stateBlocks should ideally be equal and at the origin..
     */

    Eigen::Vector3s orientation_perturbation((Eigen::Vector3s()<<0,0,0).finished());
    perturbated_final_state = initial_final_state;
    Eigen::Map<Eigen::Quaternions> quat_map(perturbated_final_state.data() + 3);

    orientation_perturbation(0) = 1.0;
    orientation_perturbation(1) = 2.0;
    orientation_perturbation(2) = 1.0;

    //introduce the perturbation directly in the quaternion StateBlock
    //quat_map = quat_map * v2q(orientation_perturbation);

    origin_KF->setState(initial_origin_state);
    last_KF->setState(perturbated_final_state);

    origin_KF->unfix();
    last_KF->unfix();

    summary = ceres_manager_wolf_diff->solve();
    //std::cout << summary.BriefReport() << std::endl;

    ASSERT_TRUE( (last_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 )) << 
    "last position state : " << last_KF->getPPtr()->getVector().transpose() << "\n origin position state : " << origin_KF->getPPtr()->getVector().transpose() << std::endl;
    ASSERT_TRUE( (last_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS));
    ASSERT_TRUE( (last_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1,  0.00000001 )) << 
    "last velocity state : " << last_KF->getVPtr()->getVector().transpose() << "\n origin velocity state : " << origin_KF->getVPtr()->getVector().transpose() << std::endl;
    ASSERT_TRUE( (last_KF->getAccBiasPtr()->getVector() - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 )) << 
    "last acc bias : " << last_KF->getAccBiasPtr()->getVector().transpose() << "\n origin acc bias : " << origin_KF->getAccBiasPtr()->getVector().transpose() << std::endl; 
    EXPECT_TRUE( (last_KF->getGyroBiasPtr()->getVector() - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 )); 

    /* Here we gave an initial orientation and final orientation. Everything is unfixed. We did not move between both KF according to
     * odometry constraint. But we perturbated the final orientation before optimization.
     * There is no reason for the origin KF to impose its values to last in an optimization point of view.
     * the minimal cost should be somewhere between both keyframe orientations.
     */
}

TEST_F(ProcessorIMU_Odom_tests_details, static_Optim_IMUOdom_2KF_perturbate_orientation3origin_FixLast)
{
    /* Both KeyFrames are unfixed.
     * We perturbate all 3 angles (ox, oy, oz) in origin_KF.
     * The perturbation is introduced in this quaternion stateblocks using q * v2q(rotation_vector_perturbation)
     *
     * Odom and IMU contraints say that the 'robot' did not move between both KeyFrames.
     * So we expect CERES to converge so that origin_KF (=) last_KF meaning that all the stateBlocks should ideally be equal and at the origin..
     */

    Eigen::Vector3s orientation_perturbation((Eigen::Vector3s()<<0,0,0).finished());
    perturbated_origin_state = initial_origin_state;
    Eigen::Map<Eigen::Quaternions> quat_map(perturbated_origin_state.data() + 3);

    orientation_perturbation(0) = 1.0;
    orientation_perturbation(1) = 2.0;
    orientation_perturbation(2) = 1.0;

    //introduce the perturbation directly in the quaternion StateBlock
    //quat_map = quat_map * v2q(orientation_perturbation);

    origin_KF->setState(perturbated_origin_state);
    last_KF->setState(initial_final_state);

    origin_KF->unfix();
    last_KF->fix();

    summary = ceres_manager_wolf_diff->solve();
    //std::cout << summary.BriefReport() << std::endl;

    ASSERT_TRUE( (last_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS*0.001 )) << 
    "last position state : " << last_KF->getPPtr()->getVector().transpose() << "\n origin position state : " << origin_KF->getPPtr()->getVector().transpose() << std::endl;;
    EXPECT_TRUE( (last_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )) <<
    "last KF orientation : " << last_KF->getOPtr()->getVector().transpose() << "\norigin KF orientation : " << origin_KF->getOPtr()->getVector().transpose() << std::endl;
    ASSERT_TRUE( (last_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1,  wolf::Constants::EPS)) << 
    "last velocity state : " << last_KF->getVPtr()->getVector().transpose() << "\n origin velocity state : " << origin_KF->getVPtr()->getVector().transpose() << std::endl;
    ASSERT_TRUE( (last_KF->getAccBiasPtr()->getVector() - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )); 
    EXPECT_TRUE( (last_KF->getGyroBiasPtr()->getVector() - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )); 
}

TEST_F(ProcessorIMU_Odom_tests_details, static_Optim_IMUOdom_2KF_perturbate_orientation3Last_FixOrigin)
{
    /* we only fixed gyroscope biases (in origin and last KF) :
     * We can suppose that orientation will be changed even more. The difference in orientation cannot be compensated in gyroscope biases but both origin_KF and last_KF
     * orientations can be changed so that the 'robot' does not move. So we can either expect the orientation to be 0.
     * Due to IMU constraint saying 'no rate of turn' we expect both velocity StateBlocks to be null.
    */

    Eigen::Vector3s orientation_perturbation((Eigen::Vector3s()<<0,0,0).finished());
    perturbated_final_state = initial_final_state;
    Eigen::Map<Eigen::Quaternions> quat_map(perturbated_final_state.data() + 3);

    orientation_perturbation(0) = 1.0;
    orientation_perturbation(1) = 2.0;
    orientation_perturbation(2) = 1.0;

    //introduce the perturbation directly in the quaternion StateBlock
    //quat_map = quat_map * v2q(orientation_perturbation);

    origin_KF->setState(initial_origin_state);
    last_KF->setState(perturbated_final_state);

    origin_KF->fix();
    last_KF->unfix();

    summary = ceres_manager_wolf_diff->solve();
    //std::cout << summary.BriefReport() << std::endl;

    ASSERT_TRUE( (last_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS*0.001 )) << 
    "last position state : " << last_KF->getPPtr()->getVector().transpose() << "\n origin position state : " << origin_KF->getPPtr()->getVector().transpose() << std::endl;;
    ASSERT_TRUE( (last_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS));
    ASSERT_TRUE( (last_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1,  wolf::Constants::EPS )) << 
    "last velocity state : " << last_KF->getVPtr()->getVector().transpose() << "\n origin velocity state : " << origin_KF->getVPtr()->getVector().transpose() << std::endl;
    ASSERT_TRUE( (last_KF->getAccBiasPtr()->getVector() - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )); 
    ASSERT_TRUE( (last_KF->getGyroBiasPtr()->getVector() - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )); 
}

TEST_F(ProcessorIMU_Odom_tests_details, static_Optim_IMUOdom_2KF_perturbate_orientation3Last_FixGyroBiaseS)
{
    Eigen::Vector3s orientation_perturbation((Eigen::Vector3s()<<0,0,0).finished());
    perturbated_final_state = initial_final_state;
    Eigen::Map<Eigen::Quaternions> quat_map(perturbated_final_state.data() + 3);

    orientation_perturbation(0) = 1.0;
    orientation_perturbation(1) = 2.0;
    orientation_perturbation(2) = 1.0;

    //introduce the perturbation directly in the quaternion StateBlock
    //quat_map = quat_map * v2q(orientation_perturbation);

    origin_KF->setState(initial_origin_state);
    last_KF->setState(perturbated_final_state);

    origin_KF->unfix();
    last_KF->unfix();
    
    //fix gyroscope biases
    originStateBlock_vec[4]->fix();
    finalStateBlock_vec[4]->fix();

    summary = ceres_manager_wolf_diff->solve();
    //std::cout << summary.BriefReport() << std::endl;

    ASSERT_TRUE( (last_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 )) << 
    "last position state : " << last_KF->getPPtr()->getVector().transpose() << "\n origin position state : " << origin_KF->getPPtr()->getVector().transpose() << std::endl;
    ASSERT_TRUE( (last_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS));
    EXPECT_TRUE( (last_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1,  wolf::Constants::EPS*100 )) << 
    "last velocity state : " << last_KF->getVPtr()->getVector().transpose() << "\n origin velocity state : " << origin_KF->getVPtr()->getVector().transpose() << std::endl;
    ASSERT_TRUE( (last_KF->getAccBiasPtr()->getVector() - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 )) << 
    "last acc bias : " << last_KF->getAccBiasPtr()->getVector().transpose() << "\n origin acc bias : " << origin_KF->getAccBiasPtr()->getVector().transpose() << std::endl; 
    ASSERT_TRUE( (last_KF->getGyroBiasPtr()->getVector() - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 ));

    // As expected, both velocity StateBlocks converge to 0. The error is in 1e-6
}


TEST_F(ProcessorIMU_Odom_tests_details, static_Optim_IMUOdom_2KF_perturbate_AccBiasOrigin_FixedLast)
{
    /* last_KF is fixed. Origin_KF is unfixed
     * Accelerometer bias of origin_KF is perturbated. 
     * We expect Ceres to be able to converge anyway and solve the problem so that the bias goes back to Zero
     *
     * Odom and IMU contraints say that the 'robot' did not move between both KeyFrames.
     * So we expect CERES to converge so that origin_KF (=) last_KF meaning that all the stateBlocks should ideally be equal and at the origin..
     */

    perturbated_origin_state = initial_origin_state;
    perturbated_origin_state(10) += 1.0;
    perturbated_origin_state(11) += 2.0;
    perturbated_origin_state(12) += 4.0;

    origin_KF->setState(perturbated_origin_state);
    last_KF->setState(initial_final_state);

    origin_KF->unfix();
    last_KF->fix();

    summary = ceres_manager_wolf_diff->solve();
    //std::cout << summary.BriefReport() << std::endl;

    ASSERT_TRUE( (last_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )) << 
    "last position state : " << last_KF->getPPtr()->getVector().transpose() << "\n origin position state : " << origin_KF->getPPtr()->getVector().transpose() << std::endl;;
    ASSERT_TRUE( (last_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )) <<
    "last orientation : " << last_KF->getOPtr()->getVector().transpose() << "\n origin orientation : " << origin_KF->getOPtr()->getVector().transpose() << std::endl;
    ASSERT_TRUE( (last_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1,  wolf::Constants::EPS)) << 
    "last velocity state : " << last_KF->getVPtr()->getVector().transpose() << "\n origin velocity state : " << origin_KF->getVPtr()->getVector().transpose() << std::endl;
    ASSERT_TRUE( (last_KF->getAccBiasPtr()->getVector() - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )) << 
    "last acc bias : " << last_KF->getAccBiasPtr()->getVector().transpose() << "\n origin acc bias : " << origin_KF->getAccBiasPtr()->getVector().transpose() << std::endl;  
    ASSERT_TRUE( (last_KF->getGyroBiasPtr()->getVector() - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )); 
}

TEST_F(ProcessorIMU_Odom_tests_details, static_Optim_IMUOdom_2KF_perturbate_AccBiasLast_FixedOrigin)
{
    /* Origin_KF is fixed. Last_KF is unfixed
     * Accelerometer bias of Last_KF is perturbated. 
     * Let's think about the situation we have here ... The problem is that the sensor bias in t2 has no effect on all measurements made before t2 
     * and thus no efect on keyFrames created before timeStamp t2 (except if the bias was always the same...)
     * Thus we do not expect the acc bias to be changed by CERES.
     *
     * Odom and IMU contraints say that the 'robot' did not move between both KeyFrames.
     * So we expect CERES to converge so that origin_KF (=) last_KF meaning that all the stateBlocks should ideally be equal and at the origin.
     */

    perturbated_final_state = initial_final_state;
    perturbated_final_state(10) += 1.0;
    perturbated_final_state(11) += 2.0;
    perturbated_final_state(12) += 3.0;

    origin_KF->setState(initial_origin_state);
    last_KF->setState(perturbated_final_state);

    origin_KF->fix();
    last_KF->unfix();

    summary = ceres_manager_wolf_diff->solve();
    //std::cout << summary.BriefReport() << std::endl;

    ASSERT_TRUE( (last_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )) << 
    "last position state : " << last_KF->getPPtr()->getVector().transpose() << "\n origin position state : " << origin_KF->getPPtr()->getVector().transpose() << std::endl;;
    ASSERT_TRUE( (last_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )) <<
    "last orientation : " << last_KF->getOPtr()->getVector().transpose() << "\n origin orientation : " << origin_KF->getOPtr()->getVector().transpose() << std::endl;
    ASSERT_TRUE( (last_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1,  wolf::Constants::EPS)) << 
    "last velocity state : " << last_KF->getVPtr()->getVector().transpose() << "\n origin velocity state : " << origin_KF->getVPtr()->getVector().transpose() << std::endl;
    //See description above to understand why we assert this to be false
    ASSERT_FALSE( (last_KF->getAccBiasPtr()->getVector() - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )) << 
    "last acc bias : " << last_KF->getAccBiasPtr()->getVector().transpose() << "\n origin acc bias : " << origin_KF->getAccBiasPtr()->getVector().transpose() << std::endl; 
    ASSERT_TRUE( (last_KF->getAccBiasPtr()->getVector() - (Eigen::Vector3s()<<perturbated_final_state(10),perturbated_final_state(11),perturbated_final_state(12)).finished()).isMuchSmallerThan(1, wolf::Constants::EPS )) << 
    "last acc bias : " << last_KF->getAccBiasPtr()->getVector().transpose() << "\n perturbated_final_state acc bias : " << (Eigen::Vector3s()<<perturbated_final_state(10),perturbated_final_state(11),perturbated_final_state(12)).finished().transpose() << std::endl; 
    ASSERT_TRUE( (last_KF->getGyroBiasPtr()->getVector() - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )); 
}

TEST_F(ProcessorIMU_Odom_tests_details, static_Optim_IMUOdom_2KF_perturbate_GyroBiasOrigin_FixedLast)
{
    /* last_KF is fixed. Origin_KF is unfixed
     * Gyrometer bias of origin_KF is perturbated. 
     * We expect Ceres to be able to converge anyway and solve the problem so that the bias goes back to Zero
     *
     * Odom and IMU contraints say that the 'robot' did not move between both KeyFrames.
     * So we expect CERES to converge so that origin_KF (=) last_KF meaning that all the stateBlocks should ideally be equal and at the origin..
     */

    WOLF_WARN("not working if perturbation is too strong")
    perturbated_origin_state = initial_origin_state;
    perturbated_origin_state(13) += 1.0;
    perturbated_origin_state(14) += 0.5;
    perturbated_origin_state(15) += 1.5;

    origin_KF->setState(perturbated_origin_state);
    last_KF->setState(initial_final_state);

    origin_KF->unfix();
    last_KF->fix();

    summary = ceres_manager_wolf_diff->solve();
    //std::cout << summary.BriefReport() << std::endl;

    ASSERT_TRUE( (last_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )) << 
    "last position state : " << last_KF->getPPtr()->getVector().transpose() << "\n origin position state : " << origin_KF->getPPtr()->getVector().transpose() << std::endl;;
    ASSERT_TRUE( (last_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )) <<
    "last orientation : " << last_KF->getOPtr()->getVector().transpose() << "\n origin orientation : " << origin_KF->getOPtr()->getVector().transpose() << std::endl;
    ASSERT_TRUE( (last_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1,  wolf::Constants::EPS)) << 
    "last velocity state : " << last_KF->getVPtr()->getVector().transpose() << "\n origin velocity state : " << origin_KF->getVPtr()->getVector().transpose() << std::endl;
    ASSERT_TRUE( (last_KF->getAccBiasPtr()->getVector() - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )) << 
    "last acc bias : " << last_KF->getAccBiasPtr()->getVector().transpose() << "\n origin acc bias : " << origin_KF->getAccBiasPtr()->getVector().transpose() << std::endl;  
    ASSERT_TRUE( (last_KF->getGyroBiasPtr()->getVector() - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )) << 
    "last gyro bias : " << last_KF->getGyroBiasPtr()->getVector().transpose() << "\n origin gyro bias : " << origin_KF->getGyroBiasPtr()->getVector().transpose() << std::endl;  
}

TEST_F(ProcessorIMU_Odom_tests_details, static_Optim_IMUOdom_2KF_perturbate_GyroBiaslast_FixedOrigin)
{
    /* Origin_KF is fixed. Last_KF is unfixed
     * Gyrometer bias of Last_KF is perturbated. 
     *
     * Let's think about the situation we have here ... The problem is that the sensor bias in t2 has no effect on all measurements made before t2 
     * and thus no efect on keyFrames created before timeStamp t2 (except if the bias was always the same...)
     * Thus we do not expect the gyroscope bias to be changed by CERES.
     *
     * Odom and IMU contraints say that the 'robot' did not move between both KeyFrames.
     * So we expect CERES to converge so that origin_KF (=) last_KF meaning that all the stateBlocks should ideally be equal and at the origin..
     */

    perturbated_final_state = initial_final_state;
    perturbated_final_state(13) += 1.0;
    perturbated_final_state(14) += 1.5;
    perturbated_final_state(15) += 0.8;

    origin_KF->setState(initial_origin_state);
    last_KF->setState(perturbated_final_state);

    origin_KF->fix();
    last_KF->unfix();

    summary = ceres_manager_wolf_diff->solve();
    //std::cout << summary.BriefReport() << std::endl;

    ASSERT_TRUE( (last_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )) << 
    "last position state : " << last_KF->getPPtr()->getVector().transpose() << "\n origin position state : " << origin_KF->getPPtr()->getVector().transpose() << std::endl;;
    ASSERT_TRUE( (last_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )) <<
    "last orientation : " << last_KF->getOPtr()->getVector().transpose() << "\n origin orientation : " << origin_KF->getOPtr()->getVector().transpose() << std::endl;
    ASSERT_TRUE( (last_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1,  wolf::Constants::EPS)) << 
    "last velocity state : " << last_KF->getVPtr()->getVector().transpose() << "\n origin velocity state : " << origin_KF->getVPtr()->getVector().transpose() << std::endl;
    ASSERT_TRUE( (last_KF->getAccBiasPtr()->getVector() - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )) << 
    "last acc bias : " << last_KF->getAccBiasPtr()->getVector().transpose() << "\n origin acc bias : " << origin_KF->getAccBiasPtr()->getVector().transpose() << std::endl;  
    //See description to understand why we assert this to false
    ASSERT_FALSE( (last_KF->getGyroBiasPtr()->getVector() - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )) << 
    "last gyro bias : " << last_KF->getGyroBiasPtr()->getVector().transpose() << "\n origin gyro bias : " << origin_KF->getGyroBiasPtr()->getVector().transpose() << std::endl; 
    ASSERT_TRUE( (last_KF->getGyroBiasPtr()->getVector() - (Eigen::Vector3s()<<perturbated_final_state(13),perturbated_final_state(14),perturbated_final_state(15)).finished()).isMuchSmallerThan(1, wolf::Constants::EPS )) << 
    "last gyro bias : " << last_KF->getAccBiasPtr()->getVector().transpose() << "\n perturbated_final_state gyro bias : " << (Eigen::Vector3s()<<perturbated_final_state(13),perturbated_final_state(14),perturbated_final_state(15)).finished().transpose() << std::endl; 
}


TEST_F(ProcessorIMU_Odom_tests, static_Optim_IMUOdom_2KF)
{

    /* In this scenario, we simulate the integration of a perfect IMU that is not moving and we add an odometry measurement.
     * Initial State is [0,0,0, 0,0,0,1, 0,0,0] so we expect the Final State to be exactly the same
     * Origin KeyFrame is fixed
     * 
     * Finally, we can represent the graph as :
     *
     *  KF0 ---- constraintIMU ---- KF1
     *     \____constraintOdom3D___/
     *
     *
     * With IMU data only, biases are not observable ! So covariance cannot be computed due to jacobian rank deficiency.
     * We must add an odometry to make covariances computable
     */

     //===================================================== END OF SETTINGS

     // set origin of processorMotions
    Eigen::VectorXs x_origin((Eigen::Matrix<wolf::Scalar,16,1>()<<0,0,0, 0,0,0,1, 0,0,0, 0,0,0, 0,0,0).finished());
    t.set(0);

    FrameBasePtr setOrigin_KF = processor_ptr_imu->setOrigin(x_origin, t);
    processor_ptr_odom3D->setOrigin(setOrigin_KF);

    wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->fix();
    //There should be 3 captures at origin_frame : CaptureOdom, captureIMU
    EXPECT_EQ((wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front())->getCaptureList().size(),2);
    ASSERT_TRUE(wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->isKey()) << "origin_frame is not a KeyFrame..." << std::endl;

    //===================================================== END{END OF SETTINGS}

    //===================================================== PROCESS DATA

    // PROCESS IMU DATA

    Eigen::Vector6s data;
    data << 0.00, 0.000, -wolf::gravity()(2), 0.0, 0.0, 0.0;
    Scalar dt = t.get();
    TimeStamp ts(0.001);
    wolf::CaptureIMUPtr imu_ptr = std::make_shared<CaptureIMU>(ts, sen_imu, data);

    while( (dt-t.get()) < (std::static_pointer_cast<ProcessorIMU>(processor_ptr_)->getMaxTimeSpan()) ){
        
        // Time and data variables
        dt += 0.001;
        ts.set(dt);
        imu_ptr->setTimeStamp(ts);
        imu_ptr->setData(data);

        // process data in capture
        imu_ptr->getTimeStamp();
        sen_imu->process(imu_ptr);
    }

    // PROCESS ODOM 3D DATA
    Eigen::Vector6s data_odom3D;
    data_odom3D << 0,0,0, 0,0,0;
    
    wolf::CaptureMotionPtr mot_ptr = std::make_shared<CaptureMotion>(ts, sen_odom3D, data_odom3D);
    sen_odom3D->process(mot_ptr);

    //===================================================== END{PROCESS DATA}

    //===================================================== SOLVER PART

    FrameIMUPtr origin_KF = std::static_pointer_cast<FrameIMU>(wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front());
    FrameIMUPtr last_KF = std::static_pointer_cast<FrameIMU>(wolf_problem_ptr_->getTrajectoryPtr()->closestKeyFrameToTimeStamp(ts));

    //Check and print wolf tree
    /*if(wolf_problem_ptr_->check(1)){
        wolf_problem_ptr_->print(4,1,1,1);
    }*/

    wolf_problem_ptr_->print(4,1,1,1);
     
    std::cout << "\t\t\t ______solving______" << std::endl;
    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
    std::cout << summary.FullReport() << std::endl;
    std::cout << "\t\t\t ______solved______" << std::endl;

    wolf_problem_ptr_->print(4,1,1,1);

    ASSERT_TRUE( (last_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS ));
    ASSERT_TRUE( (last_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS ));
    ASSERT_TRUE( (last_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS ));
    ASSERT_TRUE( (last_KF->getAccBiasPtr()->getVector() - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )); //because we simulate a perfect IMU
    ASSERT_TRUE( (last_KF->getGyroBiasPtr()->getVector() - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )); //because we simulate a perfect IMU

    // COMPUTE COVARIANCES
    std::cout << "\t\t\t ______computing covariances______" << std::endl;
    ceres_manager_wolf_diff->computeCovariances(ALL);//ALL_MARGINALS, ALL
    std::cout << "\t\t\t ______computed!______" << std::endl;

    //===================================================== END{SOLVER PART}

}

/* 2 of the tests above seem to indicate that more precise tests are required :
 *  - ProcessorIMU_Odom_tests_details.static_Optim_IMUOdom_2KF_perturbate_GyroBiasOrigin_FixedLast :
 *          If the gyroscope perturbation is too strong, then the test does not pass, from tests below , we see that when we introduce perturbation along 1 axis only
 *          then this threshold is the same on all axis : PI
 *          If another gyr+oscope bias is perturbated, then the threshold is different
 */

 TEST_F(ProcessorIMU_Odom_tests_details, static_Optim_IMUOdom_2KF_perturbate_GyroBiasOrigin_FixedLast_extensive_bwx)
{
    /* last_KF is fixed. Origin_KF is unfixed
     * Gyrometer bias of origin_KF is perturbated. 
     * We expect Ceres to be able to converge anyway and solve the problem so that the bias goes back to Zero
     *
     * Odom and IMU contraints say that the 'robot' did not move between both KeyFrames.
     * So we expect CERES to converge so that origin_KF (=) last_KF meaning that all the stateBlocks should ideally be equal and at the origin..
     *
     *Due to detected strange behaviour. We try here to see how perturbated can a gyroscope bias value be before the optimation gives incorrect values
     */

    WOLF_WARN("not working if perturbation > PI")
    perturbated_origin_state = initial_origin_state;
    perturbated_origin_state(13) += M_PI-0.002;

    //loop over the bwz to find the problematic threshold
    for(int i = 0; i<2 ; i++)
    {
        perturbated_origin_state(13) += 0.001;
        WOLF_INFO("added perturbation : ", perturbated_origin_state(13))

        origin_KF->setState(perturbated_origin_state);
        last_KF->setState(initial_final_state);

        origin_KF->unfix();
        last_KF->fix();

        summary = ceres_manager_wolf_diff->solve();
        //std::cout << summary.BriefReport() << std::endl;

        ASSERT_TRUE( (last_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )) << 
        "last position state : " << last_KF->getPPtr()->getVector().transpose() << "\n origin position state : " << origin_KF->getPPtr()->getVector().transpose() << std::endl;;
        
        ASSERT_TRUE( (last_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )) <<
        "last orientation : " << last_KF->getOPtr()->getVector().transpose() << "\n origin orientation : " << origin_KF->getOPtr()->getVector().transpose() << std::endl;
        
        if(perturbated_origin_state(13) < M_PI){
            ASSERT_TRUE( (last_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1,  wolf::Constants::EPS)) << 
            "last velocity state : " << last_KF->getVPtr()->getVector().transpose() << "\n origin velocity state : " << origin_KF->getVPtr()->getVector().transpose() << std::endl;
            
            ASSERT_TRUE( (last_KF->getAccBiasPtr()->getVector() - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )) << 
            "last acc bias : " << last_KF->getAccBiasPtr()->getVector().transpose() << "\n origin acc bias : " << origin_KF->getAccBiasPtr()->getVector().transpose() << std::endl;  
            
            ASSERT_TRUE( (last_KF->getGyroBiasPtr()->getVector() - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )) << 
            "last gyro bias : " << last_KF->getGyroBiasPtr()->getVector().transpose() << "\n origin gyro bias : " << origin_KF->getGyroBiasPtr()->getVector().transpose() << std::endl;  
        }
        else{
            EXPECT_FALSE( (last_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1,  wolf::Constants::EPS)) << 
            "last velocity state : " << last_KF->getVPtr()->getVector().transpose() << "\n origin velocity state : " << origin_KF->getVPtr()->getVector().transpose() << std::endl;
            
            EXPECT_FALSE( (last_KF->getAccBiasPtr()->getVector() - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )) << 
            "last acc bias : " << last_KF->getAccBiasPtr()->getVector().transpose() << "\n origin acc bias : " << origin_KF->getAccBiasPtr()->getVector().transpose() << std::endl;  
            
            EXPECT_FALSE( (last_KF->getGyroBiasPtr()->getVector() - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )) << 
            "last gyro bias : " << last_KF->getGyroBiasPtr()->getVector().transpose() << "\n origin gyro bias : " << origin_KF->getGyroBiasPtr()->getVector().transpose() << std::endl;  
        }
    }
}

 TEST_F(ProcessorIMU_Odom_tests_details, static_Optim_IMUOdom_2KF_perturbate_GyroBiasOrigin_FixedLast_extensive_bwy)
{
    /* last_KF is fixed. Origin_KF is unfixed
     * Gyrometer bias of origin_KF is perturbated. 
     * We expect Ceres to be able to converge anyway and solve the problem so that the bias goes back to Zero
     *
     * Odom and IMU contraints say that the 'robot' did not move between both KeyFrames.
     * So we expect CERES to converge so that origin_KF (=) last_KF meaning that all the stateBlocks should ideally be equal and at the origin..
     *
     *Due to detected strange behaviour. We try here to see how perturbated can a gyroscope bias value be before the optimation gives incorrect values
     */

    WOLF_WARN("not working if perturbation > PI")
    perturbated_origin_state = initial_origin_state;
    perturbated_origin_state(14) += M_PI-0.002;

    //loop over the bwz to find the problematic threshold
    for(int i = 0; i<2 ; i++)
    {
        perturbated_origin_state(14) += 0.001;
        WOLF_INFO("added perturbation : ", perturbated_origin_state(14))

        origin_KF->setState(perturbated_origin_state);
        last_KF->setState(initial_final_state);

        origin_KF->unfix();
        last_KF->fix();

        summary = ceres_manager_wolf_diff->solve();
        //std::cout << summary.BriefReport() << std::endl;

        ASSERT_TRUE( (last_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )) << 
        "last position state : " << last_KF->getPPtr()->getVector().transpose() << "\n origin position state : " << origin_KF->getPPtr()->getVector().transpose() << std::endl;;
        
        ASSERT_TRUE( (last_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )) <<
        "last orientation : " << last_KF->getOPtr()->getVector().transpose() << "\n origin orientation : " << origin_KF->getOPtr()->getVector().transpose() << std::endl;
        
        if(perturbated_origin_state(14) < M_PI){
            ASSERT_TRUE( (last_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1,  wolf::Constants::EPS)) << 
            "last velocity state : " << last_KF->getVPtr()->getVector().transpose() << "\n origin velocity state : " << origin_KF->getVPtr()->getVector().transpose() << std::endl;
            
            ASSERT_TRUE( (last_KF->getAccBiasPtr()->getVector() - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )) << 
            "last acc bias : " << last_KF->getAccBiasPtr()->getVector().transpose() << "\n origin acc bias : " << origin_KF->getAccBiasPtr()->getVector().transpose() << std::endl;  
            
            ASSERT_TRUE( (last_KF->getGyroBiasPtr()->getVector() - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )) << 
            "last gyro bias : " << last_KF->getGyroBiasPtr()->getVector().transpose() << "\n origin gyro bias : " << origin_KF->getGyroBiasPtr()->getVector().transpose() << std::endl;  
        }
        else{
            EXPECT_FALSE( (last_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1,  wolf::Constants::EPS)) << 
            "last velocity state : " << last_KF->getVPtr()->getVector().transpose() << "\n origin velocity state : " << origin_KF->getVPtr()->getVector().transpose() << std::endl;
            
            EXPECT_FALSE( (last_KF->getAccBiasPtr()->getVector() - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )) << 
            "last acc bias : " << last_KF->getAccBiasPtr()->getVector().transpose() << "\n origin acc bias : " << origin_KF->getAccBiasPtr()->getVector().transpose() << std::endl;  
            
            EXPECT_FALSE( (last_KF->getGyroBiasPtr()->getVector() - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )) << 
            "last gyro bias : " << last_KF->getGyroBiasPtr()->getVector().transpose() << "\n origin gyro bias : " << origin_KF->getGyroBiasPtr()->getVector().transpose() << std::endl;  
        }
    }
}

 TEST_F(ProcessorIMU_Odom_tests_details, static_Optim_IMUOdom_2KF_perturbate_GyroBiasOrigin_FixedLast_extensive_bwz)
{
    /* last_KF is fixed. Origin_KF is unfixed
     * Gyrometer bias of origin_KF is perturbated. 
     * We expect Ceres to be able to converge anyway and solve the problem so that the bias goes back to Zero
     *
     * Odom and IMU contraints say that the 'robot' did not move between both KeyFrames.
     * So we expect CERES to converge so that origin_KF (=) last_KF meaning that all the stateBlocks should ideally be equal and at the origin..
     *
     * Due to detected strange behaviour. We try here to see how perturbated can a gyroscope bias value be before the optimation gives incorrect values
     *
     * Z axis is special since it will measure gravity in this static context
     */

    WOLF_WARN("not working if perturbation > PI")
    perturbated_origin_state = initial_origin_state;
    perturbated_origin_state(15) += M_PI-0.002;

    //loop over the bwz to find the problematic threshold
    for(int i = 0; i<2 ; i++)
    {
        perturbated_origin_state(15) += 0.001;
        WOLF_INFO("added perturbation : ", perturbated_origin_state(15))

        origin_KF->setState(perturbated_origin_state);
        last_KF->setState(initial_final_state);

        origin_KF->unfix();
        last_KF->fix();

        summary = ceres_manager_wolf_diff->solve();
        //std::cout << summary.BriefReport() << std::endl;

        ASSERT_TRUE( (last_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )) << 
        "last position state : " << last_KF->getPPtr()->getVector().transpose() << "\n origin position state : " << origin_KF->getPPtr()->getVector().transpose() << std::endl;;
        
        ASSERT_TRUE( (last_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )) <<
        "last orientation : " << last_KF->getOPtr()->getVector().transpose() << "\n origin orientation : " << origin_KF->getOPtr()->getVector().transpose() << std::endl;
        
        ASSERT_TRUE( (last_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1,  wolf::Constants::EPS)) << 
        "last velocity state : " << last_KF->getVPtr()->getVector().transpose() << "\n origin velocity state : " << origin_KF->getVPtr()->getVector().transpose() << std::endl;
            
        ASSERT_TRUE( (last_KF->getAccBiasPtr()->getVector() - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )) << 
        "last acc bias : " << last_KF->getAccBiasPtr()->getVector().transpose() << "\n origin acc bias : " << origin_KF->getAccBiasPtr()->getVector().transpose() << std::endl; 

        if(perturbated_origin_state(15) < M_PI){ 
            ASSERT_TRUE( (last_KF->getGyroBiasPtr()->getVector() - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )) << 
            "last gyro bias : " << last_KF->getGyroBiasPtr()->getVector().transpose() << "\n origin gyro bias : " << origin_KF->getGyroBiasPtr()->getVector().transpose() << std::endl;  
        }
        else{  
            EXPECT_FALSE( (last_KF->getGyroBiasPtr()->getVector() - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )) << 
            "last gyro bias : " << last_KF->getGyroBiasPtr()->getVector().transpose() << "\n origin gyro bias : " << origin_KF->getGyroBiasPtr()->getVector().transpose() << std::endl;  
        }
    }
}

 TEST_F(ProcessorIMU_Odom_tests_details, static_Optim_IMUOdom_2KF_perturbate_GyroBiasOrigin_FixedLast_extensive_bwxy)
{
    /* last_KF is fixed. Origin_KF is unfixed
     * Gyrometer bias of origin_KF is perturbated. 
     * We expect Ceres to be able to converge anyway and solve the problem so that the bias goes back to Zero
     *
     * Odom and IMU contraints say that the 'robot' did not move between both KeyFrames.
     * So we expect CERES to converge so that origin_KF (=) last_KF meaning that all the stateBlocks should ideally be equal and at the origin..
     *
     *Due to detected strange behaviour. We try here to see how perturbated can a gyroscope bias value be before the optimation gives incorrect values
     */

    WOLF_WARN("not working if perturbation is too strong")
    perturbated_origin_state = initial_origin_state;
    perturbated_origin_state(13) += M_PI - 1;
    perturbated_origin_state(14) += 2.1;

    //loop over the bwz to find the problematic threshold
    for(int i = 0; i<2 ; i++)
    {
        perturbated_origin_state(14) += 0.1;
        WOLF_INFO("added perturbation : ", perturbated_origin_state(14))

        origin_KF->setState(perturbated_origin_state);
        last_KF->setState(initial_final_state);

        origin_KF->unfix();
        last_KF->fix();

        summary = ceres_manager_wolf_diff->solve();
        //std::cout << summary.BriefReport() << std::endl;

        ASSERT_TRUE( (last_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )) << 
        "last position state : " << last_KF->getPPtr()->getVector().transpose() << "\n origin position state : " << origin_KF->getPPtr()->getVector().transpose() << std::endl;;
        ASSERT_TRUE( (last_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )) <<
        "last orientation : " << last_KF->getOPtr()->getVector().transpose() << "\n origin orientation : " << origin_KF->getOPtr()->getVector().transpose() << std::endl;
        ASSERT_TRUE( (last_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1,  wolf::Constants::EPS)) << 
        "last velocity state : " << last_KF->getVPtr()->getVector().transpose() << "\n origin velocity state : " << origin_KF->getVPtr()->getVector().transpose() << std::endl;
        ASSERT_TRUE( (last_KF->getAccBiasPtr()->getVector() - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )) << 
        "last acc bias : " << last_KF->getAccBiasPtr()->getVector().transpose() << "\n origin acc bias : " << origin_KF->getAccBiasPtr()->getVector().transpose() << std::endl;  
        ASSERT_TRUE( (last_KF->getGyroBiasPtr()->getVector() - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )) << 
        "last gyro bias : " << last_KF->getGyroBiasPtr()->getVector().transpose() << "\n origin gyro bias : " << origin_KF->getGyroBiasPtr()->getVector().transpose() << std::endl;  
    }
}

/*  - ProcessorIMU_Odom_tests_details.static_Optim_IMUOdom_2KF_perturbate_orientation3Origin_Unfix1 :
 *          If the introduced orientation perturbation is bigger than pi then the test does not pass.
 */

TEST_F(ProcessorIMU_Odom_tests_details, static_Optim_IMUOdom_2KF_perturbate_orientation3Origin_Unfix1_extensive)
{
    /* Both KeyFrames are fixed. We unfix 1 stateblock among those we have in these 2 KeyFrames.
     * We perturbate all 3 angles (ox, oy, oz) in origin_KF. ==> We also unfix origin_KF's quaternion StateBlock.
     * The perturbation is introduced in this quaternion stateblocks using q * v2q(rotation_vector_perturbation)
     *
     * Odom and IMU contraints say that the 'robot' did not move between both KeyFrames.
     * So we expect CERES to converge so that origin_KF (=) last_KF meaning that all the stateBlocks should ideally be equal and at the origin..
     */

    Eigen::Vector3s orientation_perturbation((Eigen::Vector3s()<<0,0,0).finished());
    perturbated_origin_state = initial_origin_state;
    Eigen::Map<Eigen::Quaternions> quat_map(perturbated_origin_state.data() + 3);

    //M_PI not passed, it should work
    perturbated_origin_state = initial_origin_state;
    orientation_perturbation(2) += M_PI;
    WOLF_DEBUG("Introduced perturbation : ", orientation_perturbation(2))

    //introduce the perturbation directly in the quaternion StateBlock
    quat_map = quat_map * v2q(orientation_perturbation);
    WOLF_DEBUG("q2v(quat_map) : ", q2v(quat_map).transpose())
    origin_KF->setState(perturbated_origin_state);
    last_KF->setState(initial_final_state);

    origin_KF->fix();
    last_KF->fix();
        
    //we unfix origin ORIENTATION stateblock to let it converge
    originStateBlock_vec[1]->unfix();

    summary = ceres_manager_wolf_diff->solve();

    ASSERT_TRUE( (last_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 )) << 
    "last position state : " << last_KF->getPPtr()->getVector().transpose() << "\n origin position state : " << origin_KF->getPPtr()->getVector().transpose() << std::endl;;
    ASSERT_TRUE( (last_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS*10 )) <<
    "last orientation : " << last_KF->getOPtr()->getVector().transpose() << "\n origin orientation : " << origin_KF->getOPtr()->getVector().transpose() << std::endl;
    ASSERT_TRUE( (last_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1,  0.00000001 ));
    ASSERT_TRUE( (last_KF->getAccBiasPtr()->getVector() - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 )); 

    // Now we reach PI, and it will not work 
    orientation_perturbation(2) = M_PI+0.0001;

    perturbated_origin_state = initial_origin_state;
    WOLF_DEBUG("Introduced perturbation : ", orientation_perturbation(2))

    //introduce the perturbation directly in the quaternion StateBlock
    quat_map = quat_map * v2q(orientation_perturbation);
    WOLF_DEBUG("q2v(quat_map) : ", q2v(quat_map).transpose())
    origin_KF->setState(perturbated_origin_state);
    last_KF->setState(initial_final_state);

    origin_KF->fix();
    last_KF->fix();
        
    //we unfix origin ORIENTATION stateblock to let it converge
    originStateBlock_vec[1]->unfix();

    summary = ceres_manager_wolf_diff->solve();

    ASSERT_TRUE( (last_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 )) << 
    "last position state : " << last_KF->getPPtr()->getVector().transpose() << "\n origin position state : " << origin_KF->getPPtr()->getVector().transpose() << std::endl;;
    EXPECT_FALSE( (last_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS*10 )) <<
    "last orientation : " << last_KF->getOPtr()->getVector().transpose() << "\n origin orientation : " << origin_KF->getOPtr()->getVector().transpose() << std::endl;
    ASSERT_TRUE( (last_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1,  0.00000001 ));
    ASSERT_TRUE( (last_KF->getAccBiasPtr()->getVector() - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, 0.00000001 )); 
}

//_______________________________________________________________________________________________________________
// END ##################################### static_Optim_IMUOdom_2KF TESTS #####################################
//_______________________________________________________________________________________________________________


//_______________________________________________________________________________________________________________
// ##################################### static_Optim_IMUOdom_3KF TESTS #####################################
//_______________________________________________________________________________________________________________

/*  Tests below will be based on the following representation :
 *     KF0 ---- constraintIMU ---- KF1 --- constraintIMU -- KF2
 *        \____constraintOdom3D___/  \____constraintOdom3D___/
 */

TEST_F(ProcessorIMU_Odom_tests_details3KF, static_optim_IMUOdom_perturbatePositionOrigin_fixLast)
{
    /* We perturbate the origin KF and fix the last KF. (origin_KF and middle_KF are unfixed)
     * We want tomake sure that the middle KF will not have undesired behaviour.
     *
     * We notice some strange thing about acceleration bias here.
     * We could think that the optimizer can use the fact that origin and middle KF are unfixed and use acc biases to satisfy position conditions given odom and imu constraints
     * Thus we cannot expect accelerometer bias in origin_KF and middle_KF to be equal 
     * in practice, We notice that if we perturbate Pz in origin_KF, then the equality of acc biases in origin and middle_KF is met.
     * but if Pz is not perturbated then this equality does not seem to be true. 
     * We can wonder what will happen if acc bias stateBlocks are fixed. We investigate this in the next test
     */

     WOLF_WARN("investigation required here ...")

    for(int pert_index0 = 0; pert_index0<3; pert_index0++)
    {
        for(int pert_index1 = 0; pert_index1<3; pert_index1++)
        {
            for(int pert_index2 = 0; pert_index2<3; pert_index2++)
            {
                //perturate initial state
                perturbated_origin_state = initial_origin_state;
                perturbated_origin_state(pert_index0) += 1.0;
                perturbated_origin_state(pert_index1) += 1.0;
                perturbated_origin_state(pert_index2) += 1.0;

                origin_KF->setState(perturbated_origin_state);
                middle_KF->setState(initial_middle_state);
                last_KF->setState(initial_final_state);

                origin_KF->unfix();
                middle_KF->unfix();
                last_KF->fix();

                summary = ceres_manager_wolf_diff->solve();
                //std::cout << summary.BriefReport() << std::endl;

                //test last against origin
                ASSERT_TRUE( (last_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )) << 
                "last position state : " << last_KF->getPPtr()->getVector().transpose() << "\n origin position state : " << origin_KF->getPPtr()->getVector().transpose() << std::endl;;
                ASSERT_TRUE( (last_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*1000 ));
                ASSERT_TRUE( (last_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1,  wolf::Constants::EPS ));
                ASSERT_TRUE( (last_KF->getAccBiasPtr()->getVector() - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS ));
                ASSERT_TRUE( (last_KF->getGyroBiasPtr()->getVector() - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS ));
                ASSERT_TRUE( (last_KF->getPPtr()->getVector() - (Eigen::Vector3s()<<0,0,0).finished()).isMuchSmallerThan(1, wolf::Constants::EPS )) << 
                "last position state : " << last_KF->getPPtr()->getVector().transpose() << std::endl;
                ASSERT_TRUE( (last_KF->getOPtr()->getVector() - (Eigen::Vector4s()<<0,0,0,1).finished()).isMuchSmallerThan(1, wolf::Constants::EPS ));
                ASSERT_TRUE( (last_KF->getVPtr()->getVector() - (Eigen::Vector3s()<<0,0,0).finished()).isMuchSmallerThan(1, wolf::Constants::EPS ));

                //test middle against origin
                ASSERT_TRUE( (middle_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )) << 
                "middle position state : " << middle_KF->getPPtr()->getVector().transpose() << "\n origin position state : " << origin_KF->getPPtr()->getVector().transpose() << std::endl;;
                ASSERT_TRUE( (middle_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*1000 ));
                ASSERT_TRUE( (middle_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1,  wolf::Constants::EPS ));

                /*EXPECT_TRUE( (middle_KF->getAccBiasPtr()->getVector() - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )) <<
                "middle_KF acc bias : " << middle_KF->getAccBiasPtr()->getVector().transpose() << "\n origin_KF acc bias : " << origin_KF->getAccBiasPtr()->getVector().transpose() <<
                "\n perturbation index : " << pert_index0 << "." << pert_index1 << "." << pert_index2 << std::endl;*/
                ASSERT_TRUE( (middle_KF->getGyroBiasPtr()->getVector() - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS ));
                ASSERT_TRUE( (middle_KF->getPPtr()->getVector() - (Eigen::Vector3s()<<0,0,0).finished()).isMuchSmallerThan(1, wolf::Constants::EPS )) << 
                "middle position state : " << middle_KF->getPPtr()->getVector().transpose() << std::endl;
                ASSERT_TRUE( (middle_KF->getOPtr()->getVector() - (Eigen::Vector4s()<<0,0,0,1).finished()).isMuchSmallerThan(1, wolf::Constants::EPS ));
                ASSERT_TRUE( (middle_KF->getVPtr()->getVector() - (Eigen::Vector3s()<<0,0,0).finished()).isMuchSmallerThan(1, wolf::Constants::EPS ));

            }
        }
    }
}

TEST_F(ProcessorIMU_Odom_tests_details3KF, static_optim_IMUOdom_perturbatePositionOrigin_fixLast_fixAccBiaseS)
{
    /* We perturbate the origin KF and fix the last KF. (origin_KF and middle_KF are unfixed) All Accelerometer biase StateBlocks are fixed
     * We want tomake sure that the middle KF will not have undesired behaviour.
     */

     //WOLF_WARN("investigation required here ...")

    for(int pert_index0 = 0; pert_index0<3; pert_index0++)
    {
        for(int pert_index1 = 0; pert_index1<3; pert_index1++)
        {
            for(int pert_index2 = 0; pert_index2<3; pert_index2++)
            {
                //perturate initial state
                perturbated_origin_state = initial_origin_state;
                perturbated_origin_state(pert_index0) += 1.0;
                perturbated_origin_state(pert_index1) += 1.0;
                perturbated_origin_state(pert_index2) += 1.0;

                origin_KF->setState(perturbated_origin_state);
                middle_KF->setState(initial_middle_state);
                last_KF->setState(initial_final_state);

                origin_KF->unfix();
                middle_KF->unfix();
                originStateBlock_vec[3]->fix();
                middleStateBlock_vec[3]->fix();
                last_KF->fix();

                summary = ceres_manager_wolf_diff->solve();
                //std::cout << summary.BriefReport() << std::endl;

                //test last against origin
                ASSERT_TRUE( (last_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )) << 
                "last position state : " << last_KF->getPPtr()->getVector().transpose() << "\n origin position state : " << origin_KF->getPPtr()->getVector().transpose() << std::endl;;
                ASSERT_TRUE( (last_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*1000 ));
                ASSERT_TRUE( (last_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1,  wolf::Constants::EPS ));
                ASSERT_TRUE( (last_KF->getAccBiasPtr()->getVector() - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS ));
                ASSERT_TRUE( (last_KF->getGyroBiasPtr()->getVector() - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS ));
                ASSERT_TRUE( (last_KF->getPPtr()->getVector() - (Eigen::Vector3s()<<0,0,0).finished()).isMuchSmallerThan(1, wolf::Constants::EPS )) << 
                "last position state : " << last_KF->getPPtr()->getVector().transpose() << std::endl;
                ASSERT_TRUE( (last_KF->getOPtr()->getVector() - (Eigen::Vector4s()<<0,0,0,1).finished()).isMuchSmallerThan(1, wolf::Constants::EPS ));
                ASSERT_TRUE( (last_KF->getVPtr()->getVector() - (Eigen::Vector3s()<<0,0,0).finished()).isMuchSmallerThan(1, wolf::Constants::EPS ));

                //test middle against origin
                ASSERT_TRUE( (middle_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )) << 
                "middle position state : " << middle_KF->getPPtr()->getVector().transpose() << "\n origin position state : " << origin_KF->getPPtr()->getVector().transpose() << std::endl;;
                ASSERT_TRUE( (middle_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*1000 ));
                ASSERT_TRUE( (middle_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1,  wolf::Constants::EPS ));

                EXPECT_TRUE( (middle_KF->getAccBiasPtr()->getVector() - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )) <<
                "middle_KF acc bias : " << middle_KF->getAccBiasPtr()->getVector().transpose() << "\n origin_KF acc bias : " << origin_KF->getAccBiasPtr()->getVector().transpose() <<
                "\n perturbation index : " << pert_index0 << "." << pert_index1 << "." << pert_index2 << std::endl;
                ASSERT_TRUE( (middle_KF->getGyroBiasPtr()->getVector() - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS ));
                ASSERT_TRUE( (middle_KF->getPPtr()->getVector() - (Eigen::Vector3s()<<0,0,0).finished()).isMuchSmallerThan(1, wolf::Constants::EPS )) << 
                "middle position state : " << middle_KF->getPPtr()->getVector().transpose() << std::endl;
                ASSERT_TRUE( (middle_KF->getOPtr()->getVector() - (Eigen::Vector4s()<<0,0,0,1).finished()).isMuchSmallerThan(1, wolf::Constants::EPS ));
                ASSERT_TRUE( (middle_KF->getVPtr()->getVector() - (Eigen::Vector3s()<<0,0,0).finished()).isMuchSmallerThan(1, wolf::Constants::EPS ));

            }
        }
    }
}

TEST_F(ProcessorIMU_Odom_tests_details3KF, static_optim_IMUOdom_perturbateVelocityOrigin_fixLast)
{
    /* We perturbate the origin KF (velocity) and fix the last KF. (origin_KF and middle_KF are unfixed)
     * We want tomake sure that the middle KF will not have undesired behaviour.
     */

    for(int pert_index0 = 7; pert_index0<10; pert_index0++)
    {
        for(int pert_index1 = 7; pert_index1<10; pert_index1++)
        {
            for(int pert_index2 = 7; pert_index2<10; pert_index2++)
            {
                //perturate initial state
                perturbated_origin_state = initial_origin_state;
                perturbated_origin_state(pert_index0) += 1.0;
                perturbated_origin_state(pert_index1) += 1.0;
                perturbated_origin_state(pert_index2) += 1.0;

                origin_KF->setState(perturbated_origin_state);
                middle_KF->setState(initial_middle_state);
                last_KF->setState(initial_final_state);

                origin_KF->unfix();
                middle_KF->unfix();
                last_KF->fix();

                summary = ceres_manager_wolf_diff->solve();
                //std::cout << summary.BriefReport() << std::endl;

                //test last against origin
                ASSERT_TRUE( (last_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )) << 
                "last position state : " << last_KF->getPPtr()->getVector().transpose() << "\n origin position state : " << origin_KF->getPPtr()->getVector().transpose() << std::endl;;
                ASSERT_TRUE( (last_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*1000 ));
                ASSERT_TRUE( (last_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1,  wolf::Constants::EPS ));
                ASSERT_TRUE( (last_KF->getAccBiasPtr()->getVector() - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS ));
                ASSERT_TRUE( (last_KF->getGyroBiasPtr()->getVector() - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS ));
                ASSERT_TRUE( (last_KF->getPPtr()->getVector() - (Eigen::Vector3s()<<0,0,0).finished()).isMuchSmallerThan(1, wolf::Constants::EPS )) << 
                "last position state : " << last_KF->getPPtr()->getVector().transpose() << std::endl;
                ASSERT_TRUE( (last_KF->getOPtr()->getVector() - (Eigen::Vector4s()<<0,0,0,1).finished()).isMuchSmallerThan(1, wolf::Constants::EPS ));
                ASSERT_TRUE( (last_KF->getVPtr()->getVector() - (Eigen::Vector3s()<<0,0,0).finished()).isMuchSmallerThan(1, wolf::Constants::EPS ));

                //test middle against origin
                ASSERT_TRUE( (middle_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )) << 
                "middle position state : " << middle_KF->getPPtr()->getVector().transpose() << "\n origin position state : " << origin_KF->getPPtr()->getVector().transpose() << std::endl;;
                ASSERT_TRUE( (middle_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*1000 ));
                ASSERT_TRUE( (middle_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1,  wolf::Constants::EPS ));
                EXPECT_TRUE( (middle_KF->getAccBiasPtr()->getVector() - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )) <<
                "middle_KF acc bias : " << middle_KF->getAccBiasPtr()->getVector().transpose() << "\n origin_KF acc bias : " << origin_KF->getAccBiasPtr()->getVector().transpose() <<
                "\n perturbation index : " << pert_index0 << "." << pert_index1 << "." << pert_index2 << std::endl;
                ASSERT_TRUE( (middle_KF->getGyroBiasPtr()->getVector() - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS ));
                ASSERT_TRUE( (middle_KF->getPPtr()->getVector() - (Eigen::Vector3s()<<0,0,0).finished()).isMuchSmallerThan(1, wolf::Constants::EPS )) << 
                "middle position state : " << middle_KF->getPPtr()->getVector().transpose() << std::endl;
                ASSERT_TRUE( (middle_KF->getOPtr()->getVector() - (Eigen::Vector4s()<<0,0,0,1).finished()).isMuchSmallerThan(1, wolf::Constants::EPS ));
                ASSERT_TRUE( (middle_KF->getVPtr()->getVector() - (Eigen::Vector3s()<<0,0,0).finished()).isMuchSmallerThan(1, wolf::Constants::EPS ));
            }
        }
    }
}

TEST_F(ProcessorIMU_Odom_tests_details3KF, static_optim_IMUOdom_perturbateOrientationOrigin_fixLast)
{
    /* We perturbate the origin KF (orientation) and fix the last KF. (origin_KF and middle_KF are unfixed)
     * We want tomake sure that the middle KF will not have undesired behaviour.
     *
     * We notice that acceleration biases are sometimes equal in origin_KF and middle_KF but sometimes not. Still have to understand why.
     * perturbation index resulting in false : 0.1.2, 0.2.1
     */

     WOLF_WARN("investigation required here ...")

    for(int pert_index0 = 0; pert_index0<3; pert_index0++)
    {
        for(int pert_index1 = 0; pert_index1<3; pert_index1++)
        {
            for(int pert_index2 = 0; pert_index2<3; pert_index2++)
            {
                //perturate initial state
                Eigen::Vector3s orientation_perturbation((Eigen::Vector3s()<<0,0,0).finished());
                orientation_perturbation(pert_index0) += 0.5;
                orientation_perturbation(pert_index1) += 0.5;
                orientation_perturbation(pert_index2) += 0.5;

                perturbated_origin_state = initial_origin_state;
                Eigen::Map<Eigen::Quaternions> quat_map(perturbated_origin_state.data() + 3);

                //introduce the perturbation directly in the quaternion StateBlock
                quat_map = quat_map * v2q(orientation_perturbation);

                origin_KF->setState(perturbated_origin_state);
                middle_KF->setState(initial_middle_state);
                last_KF->setState(initial_final_state);

                origin_KF->unfix();
                middle_KF->unfix();
                last_KF->fix();

                summary = ceres_manager_wolf_diff->solve();
                //std::cout << summary.BriefReport() << std::endl;

                //test last against origin
                ASSERT_TRUE( (last_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )) << 
                "last position state : " << last_KF->getPPtr()->getVector().transpose() << "\n origin position state : " << origin_KF->getPPtr()->getVector().transpose() << std::endl;;
                ASSERT_TRUE( (last_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*1000 ));
                ASSERT_TRUE( (last_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1,  wolf::Constants::EPS ));
                ASSERT_TRUE( (last_KF->getAccBiasPtr()->getVector() - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS ));
                ASSERT_TRUE( (last_KF->getGyroBiasPtr()->getVector() - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS ));
                ASSERT_TRUE( (last_KF->getPPtr()->getVector() - (Eigen::Vector3s()<<0,0,0).finished()).isMuchSmallerThan(1, wolf::Constants::EPS )) << 
                "last position state : " << last_KF->getPPtr()->getVector().transpose() << std::endl;
                ASSERT_TRUE( (last_KF->getOPtr()->getVector() - (Eigen::Vector4s()<<0,0,0,1).finished()).isMuchSmallerThan(1, wolf::Constants::EPS ));
                ASSERT_TRUE( (last_KF->getVPtr()->getVector() - (Eigen::Vector3s()<<0,0,0).finished()).isMuchSmallerThan(1, wolf::Constants::EPS ));

                //test middle against origin
                ASSERT_TRUE( (middle_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )) << 
                "middle position state : " << middle_KF->getPPtr()->getVector().transpose() << "\n origin position state : " << origin_KF->getPPtr()->getVector().transpose() << std::endl;;
                ASSERT_TRUE( (middle_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*1000 ));
                ASSERT_TRUE( (middle_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1,  wolf::Constants::EPS ));
                EXPECT_TRUE( (middle_KF->getAccBiasPtr()->getVector() - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )) <<
                "middle_KF acc bias : " << middle_KF->getAccBiasPtr()->getVector().transpose() << "\n origin_KF acc bias : " << origin_KF->getAccBiasPtr()->getVector().transpose() <<
                "\n perturbation index : " << pert_index0 << "." << pert_index1 << "." << pert_index2 << std::endl;
                ASSERT_TRUE( (middle_KF->getGyroBiasPtr()->getVector() - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS ));
                ASSERT_TRUE( (middle_KF->getPPtr()->getVector() - (Eigen::Vector3s()<<0,0,0).finished()).isMuchSmallerThan(1, wolf::Constants::EPS )) << 
                "middle position state : " << middle_KF->getPPtr()->getVector().transpose() << std::endl;
                ASSERT_TRUE( (middle_KF->getOPtr()->getVector() - (Eigen::Vector4s()<<0,0,0,1).finished()).isMuchSmallerThan(1, wolf::Constants::EPS ));
                ASSERT_TRUE( (middle_KF->getVPtr()->getVector() - (Eigen::Vector3s()<<0,0,0).finished()).isMuchSmallerThan(1, wolf::Constants::EPS ));
            }
        }
    }
}

TEST_F(ProcessorIMU_Odom_tests_details3KF, static_optim_IMUOdom_perturbateAccBiasOrigin_fixLast)
{
    /* We perturbate the origin KF (Acc bias) and fix the last KF. (origin_KF and middle_KF are unfixed)
     * We want to make sure that the middle KF will not have undesired behaviour.
     */

    for(int pert_index0 = 10; pert_index0<13; pert_index0++)
    {
        for(int pert_index1 = 10; pert_index1<13; pert_index1++)
        {
            for(int pert_index2 = 10; pert_index2<13; pert_index2++)
            {
                //perturate initial state
                perturbated_origin_state = initial_origin_state;
                perturbated_origin_state(pert_index0) += 1.0;
                perturbated_origin_state(pert_index1) += 1.0;
                perturbated_origin_state(pert_index2) += 1.0;

                origin_KF->setState(perturbated_origin_state);
                middle_KF->setState(initial_middle_state);
                last_KF->setState(initial_final_state);

                origin_KF->unfix();
                middle_KF->unfix();
                last_KF->fix();

                summary = ceres_manager_wolf_diff->solve();
                //std::cout << summary.BriefReport() << std::endl;

                //test last against origin
                ASSERT_TRUE( (last_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )) << 
                "last position state : " << last_KF->getPPtr()->getVector().transpose() << "\n origin position state : " << origin_KF->getPPtr()->getVector().transpose() << std::endl;;
                ASSERT_TRUE( (last_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*1000 ));
                ASSERT_TRUE( (last_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1,  wolf::Constants::EPS ));
                ASSERT_TRUE( (last_KF->getAccBiasPtr()->getVector() - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS ));
                ASSERT_TRUE( (last_KF->getGyroBiasPtr()->getVector() - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS ));
                ASSERT_TRUE( (last_KF->getPPtr()->getVector() - (Eigen::Vector3s()<<0,0,0).finished()).isMuchSmallerThan(1, wolf::Constants::EPS )) << 
                "last position state : " << last_KF->getPPtr()->getVector().transpose() << std::endl;
                ASSERT_TRUE( (last_KF->getOPtr()->getVector() - (Eigen::Vector4s()<<0,0,0,1).finished()).isMuchSmallerThan(1, wolf::Constants::EPS ));
                ASSERT_TRUE( (last_KF->getVPtr()->getVector() - (Eigen::Vector3s()<<0,0,0).finished()).isMuchSmallerThan(1, wolf::Constants::EPS ));

                //test middle against origin
                ASSERT_TRUE( (middle_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )) << 
                "middle position state : " << middle_KF->getPPtr()->getVector().transpose() << "\n origin position state : " << origin_KF->getPPtr()->getVector().transpose() << std::endl;;
                ASSERT_TRUE( (middle_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*1000 ));
                ASSERT_TRUE( (middle_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1,  wolf::Constants::EPS ));
                EXPECT_TRUE( (middle_KF->getAccBiasPtr()->getVector() - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )) <<
                "middle_KF acc bias : " << middle_KF->getAccBiasPtr()->getVector().transpose() << "\n origin_KF acc bias : " << origin_KF->getAccBiasPtr()->getVector().transpose() << std::endl;
                ASSERT_TRUE( (middle_KF->getGyroBiasPtr()->getVector() - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS ));
                ASSERT_TRUE( (middle_KF->getPPtr()->getVector() - (Eigen::Vector3s()<<0,0,0).finished()).isMuchSmallerThan(1, wolf::Constants::EPS )) << 
                "middle position state : " << middle_KF->getPPtr()->getVector().transpose() << std::endl;
                ASSERT_TRUE( (middle_KF->getOPtr()->getVector() - (Eigen::Vector4s()<<0,0,0,1).finished()).isMuchSmallerThan(1, wolf::Constants::EPS ));
                ASSERT_TRUE( (middle_KF->getVPtr()->getVector() - (Eigen::Vector3s()<<0,0,0).finished()).isMuchSmallerThan(1, wolf::Constants::EPS ));
            }
        }
    }
}

TEST_F(ProcessorIMU_Odom_tests_details3KF, static_optim_IMUOdom_perturbateGyroBiasOrigin_fixLast)
{
    /* We perturbate the origin KF (Acc bias) and fix the last KF. (origin_KF and middle_KF are unfixed)
     * We want to make sure that the middle KF will not have undesired behaviour.
     */

    for(int pert_index0 = 13; pert_index0<16; pert_index0++)
    {
        for(int pert_index1 = 13; pert_index1<16; pert_index1++)
        {
            for(int pert_index2 = 13; pert_index2<16; pert_index2++)
            {
                //perturate initial state
                perturbated_origin_state = initial_origin_state;
                perturbated_origin_state(pert_index0) += 0.5;
                perturbated_origin_state(pert_index1) += 0.5;
                perturbated_origin_state(pert_index2) += 0.5;

                origin_KF->setState(perturbated_origin_state);
                middle_KF->setState(initial_middle_state);
                last_KF->setState(initial_final_state);

                origin_KF->unfix();
                middle_KF->unfix();
                last_KF->fix();

                summary = ceres_manager_wolf_diff->solve();
                //std::cout << summary.BriefReport() << std::endl;

                //test last against origin
                ASSERT_TRUE( (last_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )) << 
                "last position state : " << last_KF->getPPtr()->getVector().transpose() << "\n origin position state : " << origin_KF->getPPtr()->getVector().transpose() << std::endl;;
                ASSERT_TRUE( (last_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*1000 ));
                ASSERT_TRUE( (last_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1,  wolf::Constants::EPS ));
                ASSERT_TRUE( (last_KF->getAccBiasPtr()->getVector() - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS ));
                ASSERT_TRUE( (last_KF->getGyroBiasPtr()->getVector() - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS ));
                ASSERT_TRUE( (last_KF->getPPtr()->getVector() - (Eigen::Vector3s()<<0,0,0).finished()).isMuchSmallerThan(1, wolf::Constants::EPS )) << 
                "last position state : " << last_KF->getPPtr()->getVector().transpose() << std::endl;
                ASSERT_TRUE( (last_KF->getOPtr()->getVector() - (Eigen::Vector4s()<<0,0,0,1).finished()).isMuchSmallerThan(1, wolf::Constants::EPS ));
                ASSERT_TRUE( (last_KF->getVPtr()->getVector() - (Eigen::Vector3s()<<0,0,0).finished()).isMuchSmallerThan(1, wolf::Constants::EPS ));

                //test middle against origin
                ASSERT_TRUE( (middle_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )) << 
                "middle position state : " << middle_KF->getPPtr()->getVector().transpose() << "\n origin position state : " << origin_KF->getPPtr()->getVector().transpose() << std::endl;;
                ASSERT_TRUE( (middle_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*1000 ));
                ASSERT_TRUE( (middle_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1,  wolf::Constants::EPS ));
                EXPECT_TRUE( (middle_KF->getAccBiasPtr()->getVector() - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )) <<
                "middle_KF acc bias : " << middle_KF->getAccBiasPtr()->getVector().transpose() << "\n origin_KF acc bias : " << origin_KF->getAccBiasPtr()->getVector().transpose() << std::endl;
                ASSERT_TRUE( (middle_KF->getGyroBiasPtr()->getVector() - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS ));
                ASSERT_TRUE( (middle_KF->getPPtr()->getVector() - (Eigen::Vector3s()<<0,0,0).finished()).isMuchSmallerThan(1, wolf::Constants::EPS )) << 
                "middle position state : " << middle_KF->getPPtr()->getVector().transpose() << std::endl;
                ASSERT_TRUE( (middle_KF->getOPtr()->getVector() - (Eigen::Vector4s()<<0,0,0,1).finished()).isMuchSmallerThan(1, wolf::Constants::EPS ));
                ASSERT_TRUE( (middle_KF->getVPtr()->getVector() - (Eigen::Vector3s()<<0,0,0).finished()).isMuchSmallerThan(1, wolf::Constants::EPS ));
            }
        }
    }
}

TEST_F(ProcessorIMU_Odom_tests_details3KF, static_optim_IMUOdom_perturbatePositionLast_fixOrigin)
{
    /* We perturbate the origin KF and fix the last KF. (last_KF and middle_KF are unfixed)
     * We want to make sure that the middle KF will not have undesired behaviour.
     */

    for(int pert_index0 = 0; pert_index0<3; pert_index0++)
    {
        for(int pert_index1 = 0; pert_index1<3; pert_index1++)
        {
            for(int pert_index2 = 0; pert_index2<3; pert_index2++)
            {
                //perturate initial state
                perturbated_final_state = initial_final_state;
                perturbated_final_state(pert_index0) += 1.0;
                perturbated_final_state(pert_index1) += 1.0;
                perturbated_final_state(pert_index2) += 1.0;

                origin_KF->setState(initial_origin_state);
                middle_KF->setState(initial_middle_state);
                last_KF->setState(perturbated_final_state);

                origin_KF->fix();
                middle_KF->unfix();
                last_KF->unfix();

                summary = ceres_manager_wolf_diff->solve();
                //std::cout << summary.BriefReport() << std::endl;

                //test last against origin
                ASSERT_TRUE( (last_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )) << 
                "last position state : " << last_KF->getPPtr()->getVector().transpose() << "\n origin position state : " << origin_KF->getPPtr()->getVector().transpose() << std::endl;;
                ASSERT_TRUE( (last_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*1000 ));
                ASSERT_TRUE( (last_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1,  wolf::Constants::EPS ));
                ASSERT_TRUE( (last_KF->getAccBiasPtr()->getVector() - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS ));
                ASSERT_TRUE( (last_KF->getGyroBiasPtr()->getVector() - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS ));
                ASSERT_TRUE( (last_KF->getPPtr()->getVector() - (Eigen::Vector3s()<<0,0,0).finished()).isMuchSmallerThan(1, wolf::Constants::EPS )) << 
                "last position state : " << last_KF->getPPtr()->getVector().transpose() << std::endl;
                ASSERT_TRUE( (last_KF->getOPtr()->getVector() - (Eigen::Vector4s()<<0,0,0,1).finished()).isMuchSmallerThan(1, wolf::Constants::EPS ));
                ASSERT_TRUE( (last_KF->getVPtr()->getVector() - (Eigen::Vector3s()<<0,0,0).finished()).isMuchSmallerThan(1, wolf::Constants::EPS ));

                //test middle against origin
                ASSERT_TRUE( (middle_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )) << 
                "middle position state : " << middle_KF->getPPtr()->getVector().transpose() << "\n origin position state : " << origin_KF->getPPtr()->getVector().transpose() << std::endl;;
                ASSERT_TRUE( (middle_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*1000 ));
                ASSERT_TRUE( (middle_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1,  wolf::Constants::EPS ));

                EXPECT_TRUE( (middle_KF->getAccBiasPtr()->getVector() - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )) <<
                "middle_KF acc bias : " << middle_KF->getAccBiasPtr()->getVector().transpose() << "\n origin_KF acc bias : " << origin_KF->getAccBiasPtr()->getVector().transpose() <<
                "\n perturbation index : " << pert_index0 << "." << pert_index1 << "." << pert_index2 << std::endl;
                ASSERT_TRUE( (middle_KF->getGyroBiasPtr()->getVector() - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS ));
                ASSERT_TRUE( (middle_KF->getPPtr()->getVector() - (Eigen::Vector3s()<<0,0,0).finished()).isMuchSmallerThan(1, wolf::Constants::EPS )) << 
                "middle position state : " << middle_KF->getPPtr()->getVector().transpose() << std::endl;
                ASSERT_TRUE( (middle_KF->getOPtr()->getVector() - (Eigen::Vector4s()<<0,0,0,1).finished()).isMuchSmallerThan(1, wolf::Constants::EPS ));
                ASSERT_TRUE( (middle_KF->getVPtr()->getVector() - (Eigen::Vector3s()<<0,0,0).finished()).isMuchSmallerThan(1, wolf::Constants::EPS ));

            }
        }
    }
}

TEST_F(ProcessorIMU_Odom_tests_details3KF, static_optim_IMUOdom_perturbateVelocityLast_fixOrigin)
{
    /* We perturbate the last KF (velocity) and fix the origin KF. (last_KF and middle_KF are unfixed)
     * We want tomake sure that the middle KF will not have undesired behaviour.
     */

    for(int pert_index0 = 7; pert_index0<10; pert_index0++)
    {
        for(int pert_index1 = 7; pert_index1<10; pert_index1++)
        {
            for(int pert_index2 = 7; pert_index2<10; pert_index2++)
            {
                //perturate initial state
                perturbated_final_state = initial_final_state;
                perturbated_final_state(pert_index0) += 1.0;
                perturbated_final_state(pert_index1) += 1.0;
                perturbated_final_state(pert_index2) += 1.0;

                origin_KF->setState(initial_origin_state);
                middle_KF->setState(initial_middle_state);
                last_KF->setState(perturbated_final_state);

                origin_KF->fix();
                middle_KF->unfix();
                last_KF->unfix();

                summary = ceres_manager_wolf_diff->solve();
                //std::cout << summary.BriefReport() << std::endl;

                //test last against origin
                ASSERT_TRUE( (last_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )) << 
                "last position state : " << last_KF->getPPtr()->getVector().transpose() << "\n origin position state : " << origin_KF->getPPtr()->getVector().transpose() << std::endl;;
                ASSERT_TRUE( (last_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*1000 ));
                ASSERT_TRUE( (last_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1,  wolf::Constants::EPS ));
                ASSERT_TRUE( (last_KF->getAccBiasPtr()->getVector() - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS ));
                ASSERT_TRUE( (last_KF->getGyroBiasPtr()->getVector() - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS ));
                ASSERT_TRUE( (last_KF->getPPtr()->getVector() - (Eigen::Vector3s()<<0,0,0).finished()).isMuchSmallerThan(1, wolf::Constants::EPS )) << 
                "last position state : " << last_KF->getPPtr()->getVector().transpose() << std::endl;
                ASSERT_TRUE( (last_KF->getOPtr()->getVector() - (Eigen::Vector4s()<<0,0,0,1).finished()).isMuchSmallerThan(1, wolf::Constants::EPS ));
                ASSERT_TRUE( (last_KF->getVPtr()->getVector() - (Eigen::Vector3s()<<0,0,0).finished()).isMuchSmallerThan(1, wolf::Constants::EPS ));

                //test middle against origin
                ASSERT_TRUE( (middle_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )) << 
                "middle position state : " << middle_KF->getPPtr()->getVector().transpose() << "\n origin position state : " << origin_KF->getPPtr()->getVector().transpose() << std::endl;;
                ASSERT_TRUE( (middle_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*1000 ));
                ASSERT_TRUE( (middle_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1,  wolf::Constants::EPS ));
                EXPECT_TRUE( (middle_KF->getAccBiasPtr()->getVector() - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )) <<
                "middle_KF acc bias : " << middle_KF->getAccBiasPtr()->getVector().transpose() << "\n origin_KF acc bias : " << origin_KF->getAccBiasPtr()->getVector().transpose() <<
                "\n perturbation index : " << pert_index0 << "." << pert_index1 << "." << pert_index2 << std::endl;
                ASSERT_TRUE( (middle_KF->getGyroBiasPtr()->getVector() - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS ));
                ASSERT_TRUE( (middle_KF->getPPtr()->getVector() - (Eigen::Vector3s()<<0,0,0).finished()).isMuchSmallerThan(1, wolf::Constants::EPS )) << 
                "middle position state : " << middle_KF->getPPtr()->getVector().transpose() << std::endl;
                ASSERT_TRUE( (middle_KF->getOPtr()->getVector() - (Eigen::Vector4s()<<0,0,0,1).finished()).isMuchSmallerThan(1, wolf::Constants::EPS ));
                ASSERT_TRUE( (middle_KF->getVPtr()->getVector() - (Eigen::Vector3s()<<0,0,0).finished()).isMuchSmallerThan(1, wolf::Constants::EPS ));
            }
        }
    }
}

TEST_F(ProcessorIMU_Odom_tests_details3KF, static_optim_IMUOdom_perturbateOrientationLast_fixOrigin)
{
    /* We perturbate the last KF (orientation) and fix the last KF. (last_KF and middle_KF are unfixed)
     * We want to make sure that the middle KF will not have undesired behaviour.
     */

    for(int pert_index0 = 0; pert_index0<3; pert_index0++)
    {
        for(int pert_index1 = 0; pert_index1<3; pert_index1++)
        {
            for(int pert_index2 = 0; pert_index2<3; pert_index2++)
            {
                //perturate initial state
                Eigen::Vector3s orientation_perturbation((Eigen::Vector3s()<<0,0,0).finished());
                orientation_perturbation(pert_index0) += 0.5;
                orientation_perturbation(pert_index1) += 0.5;
                orientation_perturbation(pert_index2) += 0.5;

                perturbated_final_state = initial_final_state;
                Eigen::Map<Eigen::Quaternions> quat_map(perturbated_final_state.data() + 3);

                //introduce the perturbation directly in the quaternion StateBlock
                quat_map = quat_map * v2q(orientation_perturbation);

                origin_KF->setState(initial_origin_state);
                middle_KF->setState(initial_middle_state);
                last_KF->setState(perturbated_final_state);

                origin_KF->fix();
                middle_KF->unfix();
                last_KF->unfix();

                summary = ceres_manager_wolf_diff->solve();
                //std::cout << summary.BriefReport() << std::endl;

                //test last against origin
                ASSERT_TRUE( (last_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )) << 
                "last position state : " << last_KF->getPPtr()->getVector().transpose() << "\n origin position state : " << origin_KF->getPPtr()->getVector().transpose() << std::endl;;
                ASSERT_TRUE( (last_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS ));
                ASSERT_TRUE( (last_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1,  wolf::Constants::EPS ));
                ASSERT_TRUE( (last_KF->getAccBiasPtr()->getVector() - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS ));
                ASSERT_TRUE( (last_KF->getGyroBiasPtr()->getVector() - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS ));
                ASSERT_TRUE( (last_KF->getPPtr()->getVector() - (Eigen::Vector3s()<<0,0,0).finished()).isMuchSmallerThan(1, wolf::Constants::EPS )) << 
                "last position state : " << last_KF->getPPtr()->getVector().transpose() << std::endl;
                ASSERT_TRUE( (last_KF->getOPtr()->getVector() - (Eigen::Vector4s()<<0,0,0,1).finished()).isMuchSmallerThan(1, wolf::Constants::EPS ));
                ASSERT_TRUE( (last_KF->getVPtr()->getVector() - (Eigen::Vector3s()<<0,0,0).finished()).isMuchSmallerThan(1, wolf::Constants::EPS ));

                //test middle against origin
                ASSERT_TRUE( (middle_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )) << 
                "middle position state : " << middle_KF->getPPtr()->getVector().transpose() << "\n origin position state : " << origin_KF->getPPtr()->getVector().transpose() << std::endl;;
                ASSERT_TRUE( (middle_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS ));
                ASSERT_TRUE( (middle_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1,  wolf::Constants::EPS ));
                EXPECT_TRUE( (middle_KF->getAccBiasPtr()->getVector() - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )) <<
                "middle_KF acc bias : " << middle_KF->getAccBiasPtr()->getVector().transpose() << "\n origin_KF acc bias : " << origin_KF->getAccBiasPtr()->getVector().transpose() << std::endl;
                ASSERT_TRUE( (middle_KF->getGyroBiasPtr()->getVector() - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS ));
                ASSERT_TRUE( (middle_KF->getPPtr()->getVector() - (Eigen::Vector3s()<<0,0,0).finished()).isMuchSmallerThan(1, wolf::Constants::EPS )) << 
                "middle position state : " << middle_KF->getPPtr()->getVector().transpose() << std::endl;
                ASSERT_TRUE( (middle_KF->getOPtr()->getVector() - (Eigen::Vector4s()<<0,0,0,1).finished()).isMuchSmallerThan(1, wolf::Constants::EPS ));
                ASSERT_TRUE( (middle_KF->getVPtr()->getVector() - (Eigen::Vector3s()<<0,0,0).finished()).isMuchSmallerThan(1, wolf::Constants::EPS ));
            }
        }
    }
}

TEST_F(ProcessorIMU_Odom_tests_details3KF, static_optim_IMUOdom_perturbateAccBiasLast_fixOrigin)
{
    /* We perturbate the origin KF (Acc bias) and fix the last KF. (origin_KF and middle_KF are unfixed)
     * We want to make sure that the middle KF will not have undesired behaviour.
     *
     * Bias at timeStamp t does not have influence on keyFrames from timeStamp before t.
     * Hence we cannot expect the perturbated acc bias in last KF to converge toward the acc bias of origin_KF
     */

    for(int pert_index0 = 10; pert_index0<13; pert_index0++)
    {
        for(int pert_index1 = 10; pert_index1<13; pert_index1++)
        {
            for(int pert_index2 = 10; pert_index2<13; pert_index2++)
            {
                //perturate initial state
                perturbated_final_state = initial_final_state;
                perturbated_final_state(pert_index0) += 1.0;
                perturbated_final_state(pert_index1) += 1.0;
                perturbated_final_state(pert_index2) += 1.0;

                origin_KF->setState(initial_origin_state);
                middle_KF->setState(initial_middle_state);
                last_KF->setState(perturbated_final_state);

                origin_KF->fix();
                middle_KF->unfix();
                last_KF->unfix();

                summary = ceres_manager_wolf_diff->solve();
                //std::cout << summary.BriefReport() << std::endl;

                //test last against origin
                ASSERT_TRUE( (last_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )) << 
                "last position state : " << last_KF->getPPtr()->getVector().transpose() << "\n origin position state : " << origin_KF->getPPtr()->getVector().transpose() << std::endl;;
                ASSERT_TRUE( (last_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*1000 ));
                ASSERT_TRUE( (last_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1,  wolf::Constants::EPS ));

                //See description above to understand why we expect this test in ac bias to be false.
                EXPECT_FALSE( (last_KF->getAccBiasPtr()->getVector() - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS ));
                ASSERT_TRUE( (last_KF->getGyroBiasPtr()->getVector() - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS ));
                ASSERT_TRUE( (last_KF->getPPtr()->getVector() - (Eigen::Vector3s()<<0,0,0).finished()).isMuchSmallerThan(1, wolf::Constants::EPS )) << 
                "last position state : " << last_KF->getPPtr()->getVector().transpose() << std::endl;
                ASSERT_TRUE( (last_KF->getOPtr()->getVector() - (Eigen::Vector4s()<<0,0,0,1).finished()).isMuchSmallerThan(1, wolf::Constants::EPS ));
                ASSERT_TRUE( (last_KF->getVPtr()->getVector() - (Eigen::Vector3s()<<0,0,0).finished()).isMuchSmallerThan(1, wolf::Constants::EPS ));

                //test middle against origin
                ASSERT_TRUE( (middle_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )) << 
                "middle position state : " << middle_KF->getPPtr()->getVector().transpose() << "\n origin position state : " << origin_KF->getPPtr()->getVector().transpose() << std::endl;;
                ASSERT_TRUE( (middle_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*1000 ));
                ASSERT_TRUE( (middle_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1,  wolf::Constants::EPS ));
                EXPECT_TRUE( (middle_KF->getAccBiasPtr()->getVector() - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )) <<
                "middle_KF acc bias : " << middle_KF->getAccBiasPtr()->getVector().transpose() << "\n origin_KF acc bias : " << origin_KF->getAccBiasPtr()->getVector().transpose() << std::endl;
                ASSERT_TRUE( (middle_KF->getGyroBiasPtr()->getVector() - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS ));
                ASSERT_TRUE( (middle_KF->getPPtr()->getVector() - (Eigen::Vector3s()<<0,0,0).finished()).isMuchSmallerThan(1, wolf::Constants::EPS )) << 
                "middle position state : " << middle_KF->getPPtr()->getVector().transpose() << std::endl;
                ASSERT_TRUE( (middle_KF->getOPtr()->getVector() - (Eigen::Vector4s()<<0,0,0,1).finished()).isMuchSmallerThan(1, wolf::Constants::EPS ));
                ASSERT_TRUE( (middle_KF->getVPtr()->getVector() - (Eigen::Vector3s()<<0,0,0).finished()).isMuchSmallerThan(1, wolf::Constants::EPS ));
            }
        }
    }
}

TEST_F(ProcessorIMU_Odom_tests_details3KF, static_optim_IMUOdom_perturbateGyroBiasLast_fixOrigin)
{
    /* We perturbate the origin KF (Acc bias) and fix the last KF. (origin_KF and middle_KF are unfixed)
     * We want to make sure that the middle KF will not have undesired behaviour.
     *
     * Bias at timeStamp t does not have influence on keyFrames from timeStamp before t.
     * Hence we cannot expect the perturbated gyro bias in last KF to converge toward the gyro bias of origin_KF
     */

    for(int pert_index0 = 13; pert_index0<16; pert_index0++)
    {
        for(int pert_index1 = 13; pert_index1<16; pert_index1++)
        {
            for(int pert_index2 = 13; pert_index2<16; pert_index2++)
            {
                //perturate initial state
                perturbated_final_state = initial_final_state;
                perturbated_final_state(pert_index0) += 0.5;
                perturbated_final_state(pert_index1) += 0.5;
                perturbated_final_state(pert_index2) += 0.5;

                origin_KF->setState(initial_origin_state);
                middle_KF->setState(initial_middle_state);
                last_KF->setState(perturbated_final_state);

                origin_KF->fix();
                middle_KF->unfix();
                last_KF->unfix();

                summary = ceres_manager_wolf_diff->solve();
                //std::cout << summary.BriefReport() << std::endl;

                //test last against origin
                ASSERT_TRUE( (last_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )) << 
                "last position state : " << last_KF->getPPtr()->getVector().transpose() << "\n origin position state : " << origin_KF->getPPtr()->getVector().transpose() << std::endl;;
                ASSERT_TRUE( (last_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*1000 ));
                ASSERT_TRUE( (last_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1,  wolf::Constants::EPS ));
                ASSERT_TRUE( (last_KF->getAccBiasPtr()->getVector() - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS ));

                // See description to understand why this we expect the gyroscope bias test below to be false
                EXPECT_FALSE( (last_KF->getGyroBiasPtr()->getVector() - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS ));
                ASSERT_TRUE( (last_KF->getPPtr()->getVector() - (Eigen::Vector3s()<<0,0,0).finished()).isMuchSmallerThan(1, wolf::Constants::EPS )) << 
                "last position state : " << last_KF->getPPtr()->getVector().transpose() << std::endl;
                ASSERT_TRUE( (last_KF->getOPtr()->getVector() - (Eigen::Vector4s()<<0,0,0,1).finished()).isMuchSmallerThan(1, wolf::Constants::EPS ));
                ASSERT_TRUE( (last_KF->getVPtr()->getVector() - (Eigen::Vector3s()<<0,0,0).finished()).isMuchSmallerThan(1, wolf::Constants::EPS ));

                //test middle against origin
                ASSERT_TRUE( (middle_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )) << 
                "middle position state : " << middle_KF->getPPtr()->getVector().transpose() << "\n origin position state : " << origin_KF->getPPtr()->getVector().transpose() << std::endl;;
                ASSERT_TRUE( (middle_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL*1000 ));
                ASSERT_TRUE( (middle_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1,  wolf::Constants::EPS ));
                ASSERT_TRUE( (middle_KF->getAccBiasPtr()->getVector() - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )) <<
                "middle_KF acc bias : " << middle_KF->getAccBiasPtr()->getVector().transpose() << "\n origin_KF acc bias : " << origin_KF->getAccBiasPtr()->getVector().transpose() << std::endl;
                ASSERT_TRUE( (middle_KF->getGyroBiasPtr()->getVector() - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS ));
                ASSERT_TRUE( (middle_KF->getPPtr()->getVector() - (Eigen::Vector3s()<<0,0,0).finished()).isMuchSmallerThan(1, wolf::Constants::EPS )) << 
                "middle position state : " << middle_KF->getPPtr()->getVector().transpose() << std::endl;
                ASSERT_TRUE( (middle_KF->getOPtr()->getVector() - (Eigen::Vector4s()<<0,0,0,1).finished()).isMuchSmallerThan(1, wolf::Constants::EPS ));
                ASSERT_TRUE( (middle_KF->getVPtr()->getVector() - (Eigen::Vector3s()<<0,0,0).finished()).isMuchSmallerThan(1, wolf::Constants::EPS ));
            }
        }
    }
}

//_______________________________________________________________________________________________________________
// END ##################################### static_Optim_IMUOdom_3KF TESTS #####################################
//_______________________________________________________________________________________________________________

//_______________________________________________________________________________________________________________
// ##################################### Plateform TESTS #####################################
//_______________________________________________________________________________________________________________

/* Tests above were designed to make sure everything workds well with a static IMU. This means that the imu should record no move, the odometry says we
 * did not move and did not turn. As a consequence anything measured by IMU is likely to affect biases or initial conditions of the problem.
 * 
 * In the following test we  use a 3D printed IMU plateform to check the results of the optimization. This plateform imposes a final odometry linking
 * origin_KF and last_KF with a motion of [Px=0    Py=0.06    Pz=0   Ox=0   Oy=0   Oz=0] or a null motion (our choice)
 *
 * Final graph is :
 *      KF0 ---- constraintIMU ---- KF1
 *        \____constraintOdom3D____/
 *
 * Needed : The test will use 1 data files which shall meet the following requirements :
 *
 * - a file containing only imu data in IMU frame will be provided. IMU data shall be written in the form [ax, ay, az, wx, wy, wz].
 *      The IMU measurements must include the measurement of the gravity.
 *      First line of this file will contain the initial condition in position, orientation and velocity only. (PQV formulation)
 *      Each line of the file correspond to a new set of IMU data.
 *      Each data will be separated from the previous one with a tabulation (\t).
 *      Finally, the following should give a clear idea of how the file is and summarizes the previous points :
 *          (line1) px_initial\t   py_initial\t   pz_initial\t   qx_initial\t   qy_initial\t   qz_initial\t   qw_initial\t   vx_initial\t   vy_initial\t   vz_initial\t 
 *          (line2) TimesTamp1\t   ax1\t   ay1\t   az1\t   wx1\t   wy1\t   wz1\t   
 *          (line3) TimesTamp2\t   ax2\t   ay2\t   az2\t   wx2\t   wy2\t   wz2\t   
 *          (.           .           .       .       .       .       .       .  )
 *          (lineN) TimesTampN\t   axN\t   ayN\t   azN\t   wxN\t   wyN\t   wzN\t
 *
 * We first integrate all the IMU data provided by the file.
 * Filepaths are hardcoded in the code and must be placed in src folder. Once all IMU data have been integrated we use the last timestamp to feed
 * ProcessorOdom3D with a last data (that will link origin_KF to last_KF)
 */


TEST_F(ProcessorIMU_Odom_tests,Plateform_2s_move)
{
    /* last_KF is unfixed. PQV of origin_state are fixed but Acc and Gyro bias StateBlocks are unfixed.
     *
     * result : 
     *  KF1  <-- c1 	c2 	
     * Estim, ts=0,	 x = ( 0          0          0          0          0          0          1          0          0          0          0.034      -0.16      0.086      0.11       0.12       -0.041    )
     * sb: Fix Fix Fix Est Est
     * 
     * KF3  <-- 
     * sb: Est Est Est Est Est
     * Estim, ts=2.0001,	 x = ( -2.7e-11    0.06        6.6e-12     3.1e-10     3.3e-10     5.6e-12     1           0.086       0.15        0.17        0           0           0           0           0           0          )
     *
     * Initial state is OK. bias values are small enough to be coherent. However, we can see that last_KF's Velocity state is non-Zero.
     * The use of the plateforme fixes this velocity to be Zero. However we would need information about the future to estimate this velocity StateBlock as Zero
     */

    using std::shared_ptr;
    using std::make_shared;
    using std::static_pointer_cast;
    
    std::string wolf_root = _WOLF_ROOT_DIR;
    char* imu_filepath;
    std::string filepath_string(wolf_root + "/src/test/data/IMU/Test_plateforme/imu_plateform_2s.txt");
    imu_filepath   = new char[filepath_string.length() + 1];
    std::strcpy(imu_filepath, filepath_string.c_str());
    std::ifstream imu_data_input;

    imu_data_input.open(imu_filepath);
    WOLF_INFO("imu file: ", imu_filepath)
    if(!imu_data_input.is_open()){
        std::cerr << "Failed to open data files... Exiting" << std::endl;
        ADD_FAILURE();
    }

    //prepare creation of file if DEBUG_RESULTS activated
    #ifdef DEBUG_RESULTS
        std::ofstream debug_results;
        debug_results.open("debug_results.dat");
        if(debug_results)
            debug_results << "%%TimeStamp\t"
                        << "dp_x\t" << "dp_y\t" << "dp_z\t" << "dq_x\t" << "dq_y\t" << "dq_z\t" << "dq_w\t" << "dv_x\t" << "dv_y\t" << "dv_z\t"
                        << "Dp_x\t" << "Dp_y\t" << "Dp_z\t" << "Dq_x\t" << "Dq_y\t" << "Dq_z\t" << "Dq_w\t" << "Dv_x\t" << "Dv_y\t" << "Dv_z\t"
                        << "X_x\t" << "X_y\t" << "X_z\t" << "Xq_x\t" << "Xq_y\t" << "Xq_z\t" << "Xq_w\t" << "Xv_x\t" << "Xv_y\t" << "Xv_z\t" << std::endl;
    #endif


    //===================================================== SETTING PROBLEM

    // reset origin of problem
    Eigen::VectorXs x_origin((Eigen::Matrix<wolf::Scalar,16,1>()<<0,0,0, 0,0,0,1, 0,0,0, 0,0,0, 0,0,0).finished());

    // initial conditions defined from data file
    // remember that matlab's quaternion is W,X,Y,Z and the one in Eigen has X,Y,Z,W form
    imu_data_input >> x_origin[0] >> x_origin[1] >> x_origin[2] >> x_origin[6] >> x_origin[3] >> x_origin[4] >> x_origin[5] >> x_origin[7] >> x_origin[8] >> x_origin[9];

    t.set(0);
    FrameBasePtr origin_KF = processor_ptr_imu->setOrigin(x_origin, t);
    processor_ptr_odom3D->setOrigin(origin_KF);
    FrameIMUPtr origin_imuKF = std::static_pointer_cast<FrameIMU>(origin_KF);

    origin_imuKF->getPPtr()->fix();
    origin_imuKF->getOPtr()->fix();
    origin_imuKF->getVPtr()->fix();
    origin_imuKF->getAccBiasPtr()->unfix();
    origin_imuKF->getGyroBiasPtr()->unfix();
    
    //===================================================== END{SETTING PROBLEM}

    //===================================================== PROCESS DATA
    // PROCESS DATA

    Eigen::Vector6s data_imu, data_odom3D;
    data_imu << 0,0,-wolf::gravity()(2), 0,0,0;
    data_odom3D << 0,0.06,0, 0,0,0;

    Scalar input_clock;
    TimeStamp ts(0);
    wolf::CaptureIMUPtr imu_ptr = std::make_shared<CaptureIMU>(ts, sen_imu, data_imu);
    wolf::CaptureMotionPtr mot_ptr = std::make_shared<CaptureMotion>(t, sen_odom3D, data_odom3D);
    
    //when we find a IMU timestamp corresponding with this odometry timestamp then we process odometry measurement

    while( !imu_data_input.eof() )
    {
        // PROCESS IMU DATA
        // Time and data variables
        imu_data_input >> input_clock >> data_imu[0] >> data_imu[1] >> data_imu[2] >> data_imu[3] >> data_imu[4] >> data_imu[5]; //Ax, Ay, Az, Gx, Gy, Gz

        ts.set(input_clock);
        imu_ptr->setTimeStamp(ts);
        imu_ptr->setData(data_imu);

        // process data in capture
        imu_ptr->getTimeStamp();
        sen_imu->process(imu_ptr);
    }

    //IMU data have all been processed. Now we process the odom3D data
    // PROCESS ODOM 3D DATA
    mot_ptr->setTimeStamp(ts);
    mot_ptr->setData(data_odom3D);
    sen_odom3D->process(mot_ptr);

    FrameIMUPtr last_KF = std::static_pointer_cast<FrameIMU>(wolf_problem_ptr_->getTrajectoryPtr()->closestKeyFrameToTimeStamp(ts));

    //closing file
    imu_data_input.close();

    //===================================================== END{PROCESS DATA}

    //===================================================== SOLVER PART

    //Check and print wolf tree

    wolf_problem_ptr_->print(4,1,1,1);
     
    std::cout << "\t\t\t ______solving______" << std::endl;
    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
    std::cout << summary.FullReport() << std::endl;
    std::cout << "\t\t\t ______solved______" << std::endl;

    //test last against origin
    ASSERT_TRUE( (last_KF->getPPtr()->getVector() - data_odom3D.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS )) << 
    "last position state : " << last_KF->getPPtr()->getVector().transpose() << std::endl;
    ASSERT_TRUE( (last_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS ));
    //ASSERT_TRUE( (last_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1,  wolf::Constants::EPS ));
    ASSERT_TRUE( (last_KF->getOPtr()->getVector() - (Eigen::Vector4s()<<0,0,0,1).finished()).isMuchSmallerThan(1, wolf::Constants::EPS ));
    EXPECT_TRUE( (last_KF->getVPtr()->getVector() - (Eigen::Vector3s()<<0,0,0).finished()).isMuchSmallerThan(1, wolf::Constants::EPS ));
    
    wolf_problem_ptr_->print(4,1,1,1);

    // COMPUTE COVARIANCES
    std::cout << "\t\t\t ______computing covariances______" << std::endl;
    ceres_manager_wolf_diff->computeCovariances(ALL);//ALL_MARGINALS, ALL
    std::cout << "\t\t\t ______computed!______" << std::endl;

    //===================================================== END{SOLVER PART}
}

TEST_F(ProcessorIMU_Odom_tests,Plateform_5s_move)
{
    /* 
     * Trajectory
     * KF1  <-- c1 	c2 	
     * Estim, ts=0,	 x = ( 0          0          0          0          0          0          1          0          0          0          0.35       -0.23      0.57       0.13       0.12       -0.017    )
     * sb: Fix Fix Fix Est Est

     * KF3  <-- 
     * Estim, ts=5.0009,	 x = ( -7e-15       0.06         2.1e-15      -7.2e-14     3.4e-13      3.8e-14      1            0.057        -0.79        2.5          0            0            0            0            0            0           )
     * sb: Est Est Est Est Est
     *
     * Initial state is OK. bias values are small enough to be coherent. However, we can see that last_KF's Velocity state is non-Zero.
     * The use of the plateforme fixes this velocity to be Zero. However we would need information about the future to estimate this velocity StateBlock as Zero
     * remember the final velocity state in test above (same test but for 2s instead of 5s) : 0.086       0.15        0.17
     * We realize that final velocity state is farther from Zero this time.
     */

    using std::shared_ptr;
    using std::make_shared;
    using std::static_pointer_cast;
    
    std::string wolf_root = _WOLF_ROOT_DIR;
    char* imu_filepath;
    std::string filepath_string(wolf_root + "/src/test/data/IMU/Test_plateforme/imu_plateform_5s.txt");
    imu_filepath   = new char[filepath_string.length() + 1];
    std::strcpy(imu_filepath, filepath_string.c_str());
    std::ifstream imu_data_input;

    imu_data_input.open(imu_filepath);
    WOLF_INFO("imu file: ", imu_filepath)
    if(!imu_data_input.is_open()){
        std::cerr << "Failed to open data files... Exiting" << std::endl;
        ADD_FAILURE();
    }

    //prepare creation of file if DEBUG_RESULTS activated
    #ifdef DEBUG_RESULTS
        std::ofstream debug_results;
        debug_results.open("debug_results.dat");
        if(debug_results)
            debug_results << "%%TimeStamp\t"
                        << "dp_x\t" << "dp_y\t" << "dp_z\t" << "dq_x\t" << "dq_y\t" << "dq_z\t" << "dq_w\t" << "dv_x\t" << "dv_y\t" << "dv_z\t"
                        << "Dp_x\t" << "Dp_y\t" << "Dp_z\t" << "Dq_x\t" << "Dq_y\t" << "Dq_z\t" << "Dq_w\t" << "Dv_x\t" << "Dv_y\t" << "Dv_z\t"
                        << "X_x\t" << "X_y\t" << "X_z\t" << "Xq_x\t" << "Xq_y\t" << "Xq_z\t" << "Xq_w\t" << "Xv_x\t" << "Xv_y\t" << "Xv_z\t" << std::endl;
    #endif


    //===================================================== SETTING PROBLEM

    // reset origin of problem
    Eigen::VectorXs x_origin((Eigen::Matrix<wolf::Scalar,16,1>()<<0,0,0, 0,0,0,1, 0,0,0, 0,0,0, 0,0,0).finished());

    // initial conditions defined from data file
    // remember that matlab's quaternion is W,X,Y,Z and the one in Eigen has X,Y,Z,W form
    imu_data_input >> x_origin[0] >> x_origin[1] >> x_origin[2] >> x_origin[6] >> x_origin[3] >> x_origin[4] >> x_origin[5] >> x_origin[7] >> x_origin[8] >> x_origin[9];

    t.set(0);
    FrameBasePtr origin_KF = processor_ptr_imu->setOrigin(x_origin, t);
    processor_ptr_odom3D->setOrigin(origin_KF);
    FrameIMUPtr origin_imuKF = std::static_pointer_cast<FrameIMU>(origin_KF);

    origin_imuKF->getPPtr()->fix();
    origin_imuKF->getOPtr()->fix();
    origin_imuKF->getVPtr()->fix();
    origin_imuKF->getAccBiasPtr()->unfix();
    origin_imuKF->getGyroBiasPtr()->unfix();
    
    //===================================================== END{SETTING PROBLEM}

    //===================================================== PROCESS DATA
    // PROCESS DATA

    Eigen::Vector6s data_imu, data_odom3D;
    data_imu << 0,0,-wolf::gravity()(2), 0,0,0;
    data_odom3D << 0,0.06,0, 0,0,0;

    Scalar input_clock;
    TimeStamp ts(0);
    wolf::CaptureIMUPtr imu_ptr = std::make_shared<CaptureIMU>(ts, sen_imu, data_imu);
    wolf::CaptureMotionPtr mot_ptr = std::make_shared<CaptureMotion>(t, sen_odom3D, data_odom3D);
    
    //when we find a IMU timestamp corresponding with this odometry timestamp then we process odometry measurement

    while( !imu_data_input.eof() )
    {
        // PROCESS IMU DATA
        // Time and data variables
        imu_data_input >> input_clock >> data_imu[0] >> data_imu[1] >> data_imu[2] >> data_imu[3] >> data_imu[4] >> data_imu[5]; //Ax, Ay, Az, Gx, Gy, Gz

        ts.set(input_clock);
        imu_ptr->setTimeStamp(ts);
        imu_ptr->setData(data_imu);

        // process data in capture
        imu_ptr->getTimeStamp();
        sen_imu->process(imu_ptr);
    }

    //IMU data have all been processed. Now we process the odom3D data
    // PROCESS ODOM 3D DATA
    mot_ptr->setTimeStamp(ts);
    mot_ptr->setData(data_odom3D);
    sen_odom3D->process(mot_ptr);

    //closing file
    imu_data_input.close();

    FrameIMUPtr last_KF = std::static_pointer_cast<FrameIMU>(wolf_problem_ptr_->getTrajectoryPtr()->closestKeyFrameToTimeStamp(ts));

    //===================================================== END{PROCESS DATA}

    //===================================================== SOLVER PART

    //Check and print wolf tree

    wolf_problem_ptr_->print(4,1,1,1);
     
    std::cout << "\t\t\t ______solving______" << std::endl;
    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
    std::cout << summary.FullReport() << std::endl;
    std::cout << "\t\t\t ______solved______" << std::endl;
    
    wolf_problem_ptr_->print(4,1,1,1);

    //test last against origin
    ASSERT_TRUE( (last_KF->getPPtr()->getVector() - data_odom3D.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS )) << 
    "last position state : " << last_KF->getPPtr()->getVector().transpose() << std::endl;
    ASSERT_TRUE( (last_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS ));
    //ASSERT_TRUE( (last_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1,  wolf::Constants::EPS ));
    ASSERT_TRUE( (last_KF->getOPtr()->getVector() - (Eigen::Vector4s()<<0,0,0,1).finished()).isMuchSmallerThan(1, wolf::Constants::EPS ));
    EXPECT_TRUE( (last_KF->getVPtr()->getVector() - (Eigen::Vector3s()<<0,0,0).finished()).isMuchSmallerThan(1, wolf::Constants::EPS ));

    // COMPUTE COVARIANCES
    std::cout << "\t\t\t ______computing covariances______" << std::endl;
    ceres_manager_wolf_diff->computeCovariances(ALL);//ALL_MARGINALS, ALL
    std::cout << "\t\t\t ______computed!______" << std::endl;

    //===================================================== END{SOLVER PART}
}

//let's fix the final velocity to 0 and check

TEST_F(ProcessorIMU_Odom_tests,Plateform_2s_move_fixLastVelocity)
{
    /* 
     * Trajectory
     * KF1  <-- c1 	c2 	
     * Estim, ts=0,	 x = ( 0           0           0           0           0           0           1           0           0           0           -0.0048     -0.3        0.12        0.088       0.12        -0.037     )
     * sb: Fix Fix Fix Est Est

     * KF3  <-- 
     * Estim, ts=2.0001,	 x = ( -0.00013     0.06         -0.00089     0.00025      -8e-05       -3.6e-05     1            0            0            0            0            0            0            0            0            0           )
     * sb: Est Est Fix Est Est
     *
     * Fixing last velocity state to 0 makes us loose precision in final position and orientation stateBlocks
     *
     * We could try a 10 seconds experiment using the plateforme such as the final trajectory contains 3 KF. Odometry between origin and middle KF would suggest we moved on the plateforme
     * Whereas odometry between middle_KF and last_KF would suggest that we did not move.
     */

    using std::shared_ptr;
    using std::make_shared;
    using std::static_pointer_cast;
    
    std::string wolf_root = _WOLF_ROOT_DIR;
    char* imu_filepath;
    std::string filepath_string(wolf_root + "/src/test/data/IMU/Test_plateforme/imu_plateform_2s.txt");
    imu_filepath   = new char[filepath_string.length() + 1];
    std::strcpy(imu_filepath, filepath_string.c_str());
    std::ifstream imu_data_input;

    imu_data_input.open(imu_filepath);
    WOLF_INFO("imu file: ", imu_filepath)
    if(!imu_data_input.is_open()){
        std::cerr << "Failed to open data files... Exiting" << std::endl;
        ADD_FAILURE();
    }

    //prepare creation of file if DEBUG_RESULTS activated
    #ifdef DEBUG_RESULTS
        std::ofstream debug_results;
        debug_results.open("debug_results.dat");
        if(debug_results)
            debug_results << "%%TimeStamp\t"
                        << "dp_x\t" << "dp_y\t" << "dp_z\t" << "dq_x\t" << "dq_y\t" << "dq_z\t" << "dq_w\t" << "dv_x\t" << "dv_y\t" << "dv_z\t"
                        << "Dp_x\t" << "Dp_y\t" << "Dp_z\t" << "Dq_x\t" << "Dq_y\t" << "Dq_z\t" << "Dq_w\t" << "Dv_x\t" << "Dv_y\t" << "Dv_z\t"
                        << "X_x\t" << "X_y\t" << "X_z\t" << "Xq_x\t" << "Xq_y\t" << "Xq_z\t" << "Xq_w\t" << "Xv_x\t" << "Xv_y\t" << "Xv_z\t" << std::endl;
    #endif


    //===================================================== SETTING PROBLEM

    // reset origin of problem
    Eigen::VectorXs x_origin((Eigen::Matrix<wolf::Scalar,16,1>()<<0,0,0, 0,0,0,1, 0,0,0, 0,0,0, 0,0,0).finished());

    // initial conditions defined from data file
    // remember that matlab's quaternion is W,X,Y,Z and the one in Eigen has X,Y,Z,W form
    imu_data_input >> x_origin[0] >> x_origin[1] >> x_origin[2] >> x_origin[6] >> x_origin[3] >> x_origin[4] >> x_origin[5] >> x_origin[7] >> x_origin[8] >> x_origin[9];

    t.set(0);
    FrameBasePtr origin_KF = processor_ptr_imu->setOrigin(x_origin, t);
    processor_ptr_odom3D->setOrigin(origin_KF);
    FrameIMUPtr origin_imuKF = std::static_pointer_cast<FrameIMU>(origin_KF);

    origin_imuKF->getPPtr()->fix();
    origin_imuKF->getOPtr()->fix();
    origin_imuKF->getVPtr()->fix();
    origin_imuKF->getAccBiasPtr()->unfix();
    origin_imuKF->getGyroBiasPtr()->unfix();
    
    //===================================================== END{SETTING PROBLEM}

    //===================================================== PROCESS DATA
    // PROCESS DATA

    Eigen::Vector6s data_imu, data_odom3D;
    data_imu << 0,0,-wolf::gravity()(2), 0,0,0;
    data_odom3D << 0,0.06,0, 0,0,0;

    Scalar input_clock;
    TimeStamp ts(0);
    wolf::CaptureIMUPtr imu_ptr = std::make_shared<CaptureIMU>(ts, sen_imu, data_imu);
    wolf::CaptureMotionPtr mot_ptr = std::make_shared<CaptureMotion>(t, sen_odom3D, data_odom3D);
    
    //when we find a IMU timestamp corresponding with this odometry timestamp then we process odometry measurement

    while( !imu_data_input.eof() )
    {
        // PROCESS IMU DATA
        // Time and data variables
        imu_data_input >> input_clock >> data_imu[0] >> data_imu[1] >> data_imu[2] >> data_imu[3] >> data_imu[4] >> data_imu[5]; //Ax, Ay, Az, Gx, Gy, Gz

        ts.set(input_clock);
        imu_ptr->setTimeStamp(ts);
        imu_ptr->setData(data_imu);

        // process data in capture
        imu_ptr->getTimeStamp();
        sen_imu->process(imu_ptr);
    }

    //IMU data have all been processed. Now we process the odom3D data
    // PROCESS ODOM 3D DATA
    mot_ptr->setTimeStamp(ts);
    mot_ptr->setData(data_odom3D);
    sen_odom3D->process(mot_ptr);

    //closing file
    imu_data_input.close();

    //===================================================== END{PROCESS DATA}

    //===================================================== SOLVER PART

    FrameIMUPtr last_KF = std::static_pointer_cast<FrameIMU>(wolf_problem_ptr_->getTrajectoryPtr()->closestKeyFrameToTimeStamp(ts));
    last_KF->getVPtr()->fix();
    //Check and print wolf tree

    wolf_problem_ptr_->print(4,1,1,1);
     
    std::cout << "\t\t\t ______solving______" << std::endl;
    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
    std::cout << summary.FullReport() << std::endl;
    std::cout << "\t\t\t ______solved______" << std::endl;

    wolf_problem_ptr_->print(4,1,1,1);

    ASSERT_TRUE( (last_KF->getPPtr()->getVector() - data_odom3D.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS )) << 
    "last position state : " << last_KF->getPPtr()->getVector().transpose() << std::endl;
    ASSERT_TRUE( (last_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS ));
    ASSERT_TRUE( (last_KF->getOPtr()->getVector() - (Eigen::Vector4s()<<0,0,0,1).finished()).isMuchSmallerThan(1, wolf::Constants::EPS ));
    

    // COMPUTE COVARIANCES
    std::cout << "\t\t\t ______computing covariances______" << std::endl;
    ceres_manager_wolf_diff->computeCovariances(ALL);//ALL_MARGINALS, ALL
    std::cout << "\t\t\t ______computed!______" << std::endl;

    //===================================================== END{SOLVER PART}
}

//Fix final position

TEST_F(ProcessorIMU_Odom_tests,Plateform_2s_move_fixLastPosition)
{
    /* 
     * Trajectory
     * KF1  <-- c1 	c2 	
     * Estim, ts=0,	 x = ( 0          0          0          0          0          0          1          0          0          0          0.034      -0.16      0.086      0.11       0.12       -0.041    )
     * sb: Fix Fix Fix Est Est
     *
     * KF3  <-- 
     * Estim, ts=2.0001,	 x = ( 6.6e-317    0.06        0           3.1e-10     3.3e-10     5.6e-12     1           0.086       0.15        0.17        0           0           0           0           0           0          )
     * sb: Fix Est Est Est Est
     *
     * Fixing last position has no noticeable consequence in this case compared to the non-fixed case.
     */

    using std::shared_ptr;
    using std::make_shared;
    using std::static_pointer_cast;
    
    std::string wolf_root = _WOLF_ROOT_DIR;
    char* imu_filepath;
    std::string filepath_string(wolf_root + "/src/test/data/IMU/Test_plateforme/imu_plateform_2s.txt");
    imu_filepath   = new char[filepath_string.length() + 1];
    std::strcpy(imu_filepath, filepath_string.c_str());
    std::ifstream imu_data_input;

    imu_data_input.open(imu_filepath);
    WOLF_INFO("imu file: ", imu_filepath)
    if(!imu_data_input.is_open()){
        std::cerr << "Failed to open data files... Exiting" << std::endl;
        ADD_FAILURE();
    }

    //prepare creation of file if DEBUG_RESULTS activated
    #ifdef DEBUG_RESULTS
        std::ofstream debug_results;
        debug_results.open("debug_results.dat");
        if(debug_results)
            debug_results << "%%TimeStamp\t"
                        << "dp_x\t" << "dp_y\t" << "dp_z\t" << "dq_x\t" << "dq_y\t" << "dq_z\t" << "dq_w\t" << "dv_x\t" << "dv_y\t" << "dv_z\t"
                        << "Dp_x\t" << "Dp_y\t" << "Dp_z\t" << "Dq_x\t" << "Dq_y\t" << "Dq_z\t" << "Dq_w\t" << "Dv_x\t" << "Dv_y\t" << "Dv_z\t"
                        << "X_x\t" << "X_y\t" << "X_z\t" << "Xq_x\t" << "Xq_y\t" << "Xq_z\t" << "Xq_w\t" << "Xv_x\t" << "Xv_y\t" << "Xv_z\t" << std::endl;
    #endif


    //===================================================== SETTING PROBLEM

    // reset origin of problem
    Eigen::VectorXs x_origin((Eigen::Matrix<wolf::Scalar,16,1>()<<0,0,0, 0,0,0,1, 0,0,0, 0,0,0, 0,0,0).finished());

    // initial conditions defined from data file
    // remember that matlab's quaternion is W,X,Y,Z and the one in Eigen has X,Y,Z,W form
    imu_data_input >> x_origin[0] >> x_origin[1] >> x_origin[2] >> x_origin[6] >> x_origin[3] >> x_origin[4] >> x_origin[5] >> x_origin[7] >> x_origin[8] >> x_origin[9];

    t.set(0);
    FrameBasePtr origin_KF = processor_ptr_imu->setOrigin(x_origin, t);
    processor_ptr_odom3D->setOrigin(origin_KF);
    FrameIMUPtr origin_imuKF = std::static_pointer_cast<FrameIMU>(origin_KF);

    origin_imuKF->getPPtr()->fix();
    origin_imuKF->getOPtr()->fix();
    origin_imuKF->getVPtr()->fix();
    origin_imuKF->getAccBiasPtr()->unfix();
    origin_imuKF->getGyroBiasPtr()->unfix();
    
    //===================================================== END{SETTING PROBLEM}

    //===================================================== PROCESS DATA
    // PROCESS DATA

    Eigen::Vector6s data_imu, data_odom3D;
    data_imu << 0,0,-wolf::gravity()(2), 0,0,0;
    data_odom3D << 0,0.06,0, 0,0,0;

    Scalar input_clock;
    TimeStamp ts(0);
    wolf::CaptureIMUPtr imu_ptr = std::make_shared<CaptureIMU>(ts, sen_imu, data_imu);
    wolf::CaptureMotionPtr mot_ptr = std::make_shared<CaptureMotion>(t, sen_odom3D, data_odom3D);
    
    //when we find a IMU timestamp corresponding with this odometry timestamp then we process odometry measurement

    while( !imu_data_input.eof() )
    {
        // PROCESS IMU DATA
        // Time and data variables
        imu_data_input >> input_clock >> data_imu[0] >> data_imu[1] >> data_imu[2] >> data_imu[3] >> data_imu[4] >> data_imu[5]; //Ax, Ay, Az, Gx, Gy, Gz

        ts.set(input_clock);
        imu_ptr->setTimeStamp(ts);
        imu_ptr->setData(data_imu);

        // process data in capture
        imu_ptr->getTimeStamp();
        sen_imu->process(imu_ptr);
    }

    //IMU data have all been processed. Now we process the odom3D data
    // PROCESS ODOM 3D DATA
    mot_ptr->setTimeStamp(ts);
    mot_ptr->setData(data_odom3D);
    sen_odom3D->process(mot_ptr);

    //closing file
    imu_data_input.close();

    //===================================================== END{PROCESS DATA}

    //===================================================== SOLVER PART

    FrameIMUPtr last_KF = std::static_pointer_cast<FrameIMU>(wolf_problem_ptr_->getTrajectoryPtr()->closestKeyFrameToTimeStamp(ts));
    last_KF->getPPtr()->fix();
    //Check and print wolf tree

    wolf_problem_ptr_->print(4,1,1,1);
     
    std::cout << "\t\t\t ______solving______" << std::endl;
    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
    std::cout << summary.FullReport() << std::endl;
    std::cout << "\t\t\t ______solved______" << std::endl;

    wolf_problem_ptr_->print(4,1,1,1);

    ASSERT_TRUE( (last_KF->getPPtr()->getVector() - data_odom3D.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS )) << 
    "last position state : " << last_KF->getPPtr()->getVector().transpose() << std::endl;
    ASSERT_TRUE( (last_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS ));
    ASSERT_TRUE( (last_KF->getOPtr()->getVector() - (Eigen::Vector4s()<<0,0,0,1).finished()).isMuchSmallerThan(1, wolf::Constants::EPS ));
    

    // COMPUTE COVARIANCES
    std::cout << "\t\t\t ______computing covariances______" << std::endl;
    ceres_manager_wolf_diff->computeCovariances(ALL);//ALL_MARGINALS, ALL
    std::cout << "\t\t\t ______computed!______" << std::endl;

    //===================================================== END{SOLVER PART}
}

//Fix final position and velocity

TEST_F(ProcessorIMU_Odom_tests,Plateform_2s_move_fixLastPositionVelocity)
{
    /*
     * Trajectory
     * KF1  <-- c1 	c2 	
     * Estim, ts=0,	 x = ( 0           0           0           0           0           0           1           0           0           0           -0.0048     -0.3        0.12        0.088       0.12        -0.037     )
     * sb: Fix Fix Fix Est Est

     * KF3  <-- 
     * Estim, ts=2.0001,	 x = ( 0            0.06         0            0.00025      -8e-05       -3.6e-05     1            0            0            0            0            0            0            0            0            0           )
     * sb: Fix Est Fix Est Est
     *
     * Fixing both lsat velocity and position has some undesired consequence on the final orientation test. The error here is greater than 1e-6.
     */

    using std::shared_ptr;
    using std::make_shared;
    using std::static_pointer_cast;
    
    std::string wolf_root = _WOLF_ROOT_DIR;
    char* imu_filepath;
    std::string filepath_string(wolf_root + "/src/test/data/IMU/Test_plateforme/imu_plateform_2s.txt");
    imu_filepath   = new char[filepath_string.length() + 1];
    std::strcpy(imu_filepath, filepath_string.c_str());
    std::ifstream imu_data_input;

    imu_data_input.open(imu_filepath);
    WOLF_INFO("imu file: ", imu_filepath)
    if(!imu_data_input.is_open()){
        std::cerr << "Failed to open data files... Exiting" << std::endl;
        ADD_FAILURE();
    }

    //prepare creation of file if DEBUG_RESULTS activated
    #ifdef DEBUG_RESULTS
        std::ofstream debug_results;
        debug_results.open("debug_results.dat");
        if(debug_results)
            debug_results << "%%TimeStamp\t"
                        << "dp_x\t" << "dp_y\t" << "dp_z\t" << "dq_x\t" << "dq_y\t" << "dq_z\t" << "dq_w\t" << "dv_x\t" << "dv_y\t" << "dv_z\t"
                        << "Dp_x\t" << "Dp_y\t" << "Dp_z\t" << "Dq_x\t" << "Dq_y\t" << "Dq_z\t" << "Dq_w\t" << "Dv_x\t" << "Dv_y\t" << "Dv_z\t"
                        << "X_x\t" << "X_y\t" << "X_z\t" << "Xq_x\t" << "Xq_y\t" << "Xq_z\t" << "Xq_w\t" << "Xv_x\t" << "Xv_y\t" << "Xv_z\t" << std::endl;
    #endif


    //===================================================== SETTING PROBLEM

    // reset origin of problem
    Eigen::VectorXs x_origin((Eigen::Matrix<wolf::Scalar,16,1>()<<0,0,0, 0,0,0,1, 0,0,0, 0,0,0, 0,0,0).finished());

    // initial conditions defined from data file
    // remember that matlab's quaternion is W,X,Y,Z and the one in Eigen has X,Y,Z,W form
    imu_data_input >> x_origin[0] >> x_origin[1] >> x_origin[2] >> x_origin[6] >> x_origin[3] >> x_origin[4] >> x_origin[5] >> x_origin[7] >> x_origin[8] >> x_origin[9];

    t.set(0);
    FrameBasePtr origin_KF = processor_ptr_imu->setOrigin(x_origin, t);
    processor_ptr_odom3D->setOrigin(origin_KF);
    FrameIMUPtr origin_imuKF = std::static_pointer_cast<FrameIMU>(origin_KF);

    origin_imuKF->getPPtr()->fix();
    origin_imuKF->getOPtr()->fix();
    origin_imuKF->getVPtr()->fix();
    origin_imuKF->getAccBiasPtr()->unfix();
    origin_imuKF->getGyroBiasPtr()->unfix();
    
    //===================================================== END{SETTING PROBLEM}

    //===================================================== PROCESS DATA
    // PROCESS DATA

    Eigen::Vector6s data_imu, data_odom3D;
    data_imu << 0,0,-wolf::gravity()(2), 0,0,0;
    data_odom3D << 0,0.06,0, 0,0,0;

    Scalar input_clock;
    TimeStamp ts(0);
    wolf::CaptureIMUPtr imu_ptr = std::make_shared<CaptureIMU>(ts, sen_imu, data_imu);
    wolf::CaptureMotionPtr mot_ptr = std::make_shared<CaptureMotion>(t, sen_odom3D, data_odom3D);
    
    //when we find a IMU timestamp corresponding with this odometry timestamp then we process odometry measurement

    while( !imu_data_input.eof() )
    {
        // PROCESS IMU DATA
        // Time and data variables
        imu_data_input >> input_clock >> data_imu[0] >> data_imu[1] >> data_imu[2] >> data_imu[3] >> data_imu[4] >> data_imu[5]; //Ax, Ay, Az, Gx, Gy, Gz

        ts.set(input_clock);
        imu_ptr->setTimeStamp(ts);
        imu_ptr->setData(data_imu);

        // process data in capture
        imu_ptr->getTimeStamp();
        sen_imu->process(imu_ptr);
    }

    //IMU data have all been processed. Now we process the odom3D data
    // PROCESS ODOM 3D DATA
    mot_ptr->setTimeStamp(ts);
    mot_ptr->setData(data_odom3D);
    sen_odom3D->process(mot_ptr);

    //closing file
    imu_data_input.close();

    //===================================================== END{PROCESS DATA}

    //===================================================== SOLVER PART

    FrameIMUPtr last_KF = std::static_pointer_cast<FrameIMU>(wolf_problem_ptr_->getTrajectoryPtr()->closestKeyFrameToTimeStamp(ts));
    last_KF->getPPtr()->fix();
    last_KF->getVPtr()->fix();
    //Check and print wolf tree

    wolf_problem_ptr_->print(4,1,1,1);
     
    std::cout << "\t\t\t ______solving______" << std::endl;
    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
    std::cout << summary.FullReport() << std::endl;
    std::cout << "\t\t\t ______solved______" << std::endl;

    wolf_problem_ptr_->print(4,1,1,1);

    ASSERT_TRUE( (last_KF->getPPtr()->getVector() - data_odom3D.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS )) << 
    "last position state : " << last_KF->getPPtr()->getVector().transpose() << std::endl;
    ASSERT_TRUE( (last_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS ));
    ASSERT_TRUE( (last_KF->getOPtr()->getVector() - (Eigen::Vector4s()<<0,0,0,1).finished()).isMuchSmallerThan(1, wolf::Constants::EPS ));
    

    // COMPUTE COVARIANCES
    std::cout << "\t\t\t ______computing covariances______" << std::endl;
    ceres_manager_wolf_diff->computeCovariances(ALL);//ALL_MARGINALS, ALL
    std::cout << "\t\t\t ______computed!______" << std::endl;

    //===================================================== END{SOLVER PART}
}

//Fix final position and velocity and orientation
// -> optimization affects origin_KF's biases

TEST_F(ProcessorIMU_Odom_tests,Plateform_2s_move_fixLastPQV)
{
    /* Trajectory
     * KF1  <-- c1 	c2 	
     * Estim, ts=0,	 x = ( 0           0           0           0           0           0           1           0           0           0           -0.0047     -0.3        0.12        0.088       0.12        -0.037     )
     * sb: Fix Fix Fix Est Est

     * KF3  <-- 
     * Estim, ts=2.0001,	 x = ( 0    0.06 0    0    0    0    1    0    0    0    0    0    0    0    0    0   )
     * sb: Fix Fix Fix Est Est
     *
     * This test passes. Bias estimates are not incoherent.
     * 
     */

    using std::shared_ptr;
    using std::make_shared;
    using std::static_pointer_cast;
    
    std::string wolf_root = _WOLF_ROOT_DIR;
    char* imu_filepath;
    std::string filepath_string(wolf_root + "/src/test/data/IMU/Test_plateforme/imu_plateform_2s.txt");
    imu_filepath   = new char[filepath_string.length() + 1];
    std::strcpy(imu_filepath, filepath_string.c_str());
    std::ifstream imu_data_input;

    std::cout << "opening" << std::endl;
    imu_data_input.open(imu_filepath);
    std::cout << "imu file: " << imu_filepath << std::endl;
    if(!imu_data_input.is_open()){
        std::cerr << "Failed to open data files... Exiting" << std::endl;
        ADD_FAILURE();
    }
    std::cout << "opened" << std::endl;

    //prepare creation of file if DEBUG_RESULTS activated
    #ifdef DEBUG_RESULTS
        std::ofstream debug_results;
        debug_results.open("debug_results.dat");
        if(debug_results)
            debug_results << "%%TimeStamp\t"
                        << "dp_x\t" << "dp_y\t" << "dp_z\t" << "dq_x\t" << "dq_y\t" << "dq_z\t" << "dq_w\t" << "dv_x\t" << "dv_y\t" << "dv_z\t"
                        << "Dp_x\t" << "Dp_y\t" << "Dp_z\t" << "Dq_x\t" << "Dq_y\t" << "Dq_z\t" << "Dq_w\t" << "Dv_x\t" << "Dv_y\t" << "Dv_z\t"
                        << "X_x\t" << "X_y\t" << "X_z\t" << "Xq_x\t" << "Xq_y\t" << "Xq_z\t" << "Xq_w\t" << "Xv_x\t" << "Xv_y\t" << "Xv_z\t" << std::endl;
    #endif


    //===================================================== SETTING PROBLEM

    // reset origin of problem
    Eigen::VectorXs x_origin((Eigen::Matrix<wolf::Scalar,16,1>()<<0,0,0, 0,0,0,1, 0,0,0, 0,0,0, 0,0,0).finished());

    // initial conditions defined from data file
    // remember that matlab's quaternion is W,X,Y,Z and the one in Eigen has X,Y,Z,W form
    imu_data_input >> x_origin[0] >> x_origin[1] >> x_origin[2] >> x_origin[6] >> x_origin[3] >> x_origin[4] >> x_origin[5] >> x_origin[7] >> x_origin[8] >> x_origin[9];

    t.set(0);
    FrameBasePtr origin_KF = processor_ptr_imu->setOrigin(x_origin, t);
    processor_ptr_odom3D->setOrigin(origin_KF);
    FrameIMUPtr origin_imuKF = std::static_pointer_cast<FrameIMU>(origin_KF);

    origin_imuKF->getPPtr()->fix();
    origin_imuKF->getOPtr()->fix();
    origin_imuKF->getVPtr()->fix();
    origin_imuKF->getAccBiasPtr()->unfix();
    origin_imuKF->getGyroBiasPtr()->unfix();
    
    //===================================================== END{SETTING PROBLEM}

    //===================================================== PROCESS DATA
    // PROCESS DATA

    Eigen::Vector6s data_imu, data_odom3D;
    data_imu << 0,0,-wolf::gravity()(2), 0,0,0;
    data_odom3D << 0,0.06,0, 0,0,0;

    Scalar input_clock;
    TimeStamp ts(0);
    wolf::CaptureIMUPtr imu_ptr = std::make_shared<CaptureIMU>(ts, sen_imu, data_imu);
    wolf::CaptureMotionPtr mot_ptr = std::make_shared<CaptureMotion>(t, sen_odom3D, data_odom3D);
    
    //when we find a IMU timestamp corresponding with this odometry timestamp then we process odometry measurement

    while( !imu_data_input.eof() )
    {
        // PROCESS IMU DATA
        // Time and data variables
        imu_data_input >> input_clock >> data_imu[0] >> data_imu[1] >> data_imu[2] >> data_imu[3] >> data_imu[4] >> data_imu[5]; //Ax, Ay, Az, Gx, Gy, Gz

        ts.set(input_clock);
        imu_ptr->setTimeStamp(ts);
        imu_ptr->setData(data_imu);

        // process data in capture
        imu_ptr->getTimeStamp();
        sen_imu->process(imu_ptr);
    }

    //IMU data have all been processed. Now we process the odom3D data
    // PROCESS ODOM 3D DATA
    mot_ptr->setTimeStamp(ts);
    mot_ptr->setData(data_odom3D);
    sen_odom3D->process(mot_ptr);

    //closing file
    imu_data_input.close();

    //===================================================== END{PROCESS DATA}

    //===================================================== SOLVER PART

    FrameIMUPtr last_KF = std::static_pointer_cast<FrameIMU>(wolf_problem_ptr_->getTrajectoryPtr()->closestKeyFrameToTimeStamp(ts));
    last_KF->getPPtr()->fix();
    last_KF->getVPtr()->fix();
    last_KF->getOPtr()->fix();
    //Check and print wolf tree

    wolf_problem_ptr_->print(4,1,1,1);
     
    std::cout << "\t\t\t ______solving______" << std::endl;
    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
    std::cout << summary.FullReport() << std::endl;
    std::cout << "\t\t\t ______solved______" << std::endl;

    wolf_problem_ptr_->print(4,1,1,1);

    ASSERT_TRUE( (last_KF->getPPtr()->getVector() - data_odom3D.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS )) << 
    "last position state : " << last_KF->getPPtr()->getVector().transpose() << std::endl;
    ASSERT_TRUE( (last_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS ));
    ASSERT_TRUE( (last_KF->getOPtr()->getVector() - (Eigen::Vector4s()<<0,0,0,1).finished()).isMuchSmallerThan(1, wolf::Constants::EPS ));
    

    // COMPUTE COVARIANCES
    std::cout << "\t\t\t ______computing covariances______" << std::endl;
    ceres_manager_wolf_diff->computeCovariances(ALL);//ALL_MARGINALS, ALL
    std::cout << "\t\t\t ______computed!______" << std::endl;

    //===================================================== END{SOLVER PART}
}

/* Introduce a perturbation in Origin_KF position and fix the last KF. 
 * Ideally the optimization should be able to make origin_KF position converge to its correct value (value it would have taken if it had not been perturbated). 
 * 
 * However, this is not exact. We notice an error in position of 1e-4 order and a error in quaternionsin 1e-3 order.
 */

TEST_F(ProcessorIMU_Odom_tests,Plateform_2s_move_PerturbatePositionOrigin_fixLast)
{

    using std::shared_ptr;
    using std::make_shared;
    using std::static_pointer_cast;
    
    std::string wolf_root = _WOLF_ROOT_DIR;
    char* imu_filepath;
    std::string filepath_string(wolf_root + "/src/test/data/IMU/Test_plateforme/imu_plateform_2s.txt");
    imu_filepath   = new char[filepath_string.length() + 1];
    std::strcpy(imu_filepath, filepath_string.c_str());
    std::ifstream imu_data_input;

    imu_data_input.open(imu_filepath);
    WOLF_INFO("imu file: ", imu_filepath)

    if(!imu_data_input.is_open()){
        std::cerr << "Failed to open data files... Exiting" << std::endl;
        ADD_FAILURE();
    }

    //===================================================== SETTING PROBLEM

    // reset origin of problem
    Eigen::VectorXs x_origin((Eigen::Matrix<wolf::Scalar,16,1>()<<0,0,0, 0,0,0,1, 0,0,0, 0,0,0, 0,0,0).finished());

    // initial conditions defined from data file
    // remember that matlab's quaternion is W,X,Y,Z and the one in Eigen has X,Y,Z,W form
    imu_data_input >> x_origin[0] >> x_origin[1] >> x_origin[2] >> x_origin[6] >> x_origin[3] >> x_origin[4] >> x_origin[5] >> x_origin[7] >> x_origin[8] >> x_origin[9];

    t.set(0);
    FrameBasePtr origin_KF = processor_ptr_imu->setOrigin(x_origin, t);
    processor_ptr_odom3D->setOrigin(origin_KF);
    FrameIMUPtr origin_imuKF = std::static_pointer_cast<FrameIMU>(origin_KF);
    
    //===================================================== END{SETTING PROBLEM}

    //===================================================== PROCESS DATA
    // PROCESS DATA

    Eigen::Vector6s data_imu, data_odom3D;
    data_imu << 0,0,-wolf::gravity()(2), 0,0,0;
    data_odom3D << 0,0.06,0, 0,0,0;

    Scalar input_clock;
    TimeStamp ts(0);
    wolf::CaptureIMUPtr imu_ptr = std::make_shared<CaptureIMU>(ts, sen_imu, data_imu);
    wolf::CaptureMotionPtr mot_ptr = std::make_shared<CaptureMotion>(t, sen_odom3D, data_odom3D);
    

    while( !imu_data_input.eof() )
    {
        // PROCESS IMU DATA
        // Time and data variables
        imu_data_input >> input_clock >> data_imu[0] >> data_imu[1] >> data_imu[2] >> data_imu[3] >> data_imu[4] >> data_imu[5]; //Ax, Ay, Az, Gx, Gy, Gz

        ts.set(input_clock);
        imu_ptr->setTimeStamp(ts);
        imu_ptr->setData(data_imu);

        // process data in capture
        imu_ptr->getTimeStamp();
        sen_imu->process(imu_ptr);
    }

    //IMU data have all been processed. Now we process the odom3D data
    // PROCESS ODOM 3D DATA
    mot_ptr->setTimeStamp(ts);
    mot_ptr->setData(data_odom3D);
    sen_odom3D->process(mot_ptr);

    //closing file
    imu_data_input.close();

    FrameIMUPtr last_KF = std::static_pointer_cast<FrameIMU>(wolf_problem_ptr_->getTrajectoryPtr()->closestKeyFrameToTimeStamp(ts));
    last_KF->fix();
    Eigen::VectorXs initial_final_state(16);
    initial_final_state = last_KF->getState();

    // call solver to get values after optimization. Wemwill compare the following output to these
    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
    std::cout << summary.BriefReport() << std::endl;

    Eigen::VectorXs origin_state_afterCeres(16);
    origin_state_afterCeres = origin_KF->getState();

    // velocity is modified : due to bias non initialized and noises. And due to motion constraint and fixing KF1's Position, velocity is used to satisfy conditions
    EXPECT_FALSE( (last_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS*10 ) ) <<
    "last_KF velocity state : " << last_KF->getVPtr()->getVector().transpose() << "\n origin velocity : " << origin_KF->getVPtr()->getVector().transpose() << std::endl;
    EXPECT_TRUE( (last_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector() - data_odom3D.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS)) << 
    "last_KF position state : " << last_KF->getPPtr()->getVector().transpose() << "\n origin position state : " << origin_KF->getPPtr()->getVector().transpose() << std::endl;
    ASSERT_TRUE( (last_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS)) <<
    "last_KF quaternion : " << last_KF->getOPtr()->getVector().transpose() << "\n origin quaternion state : " << origin_KF->getOPtr()->getVector().transpose() << std::endl;

    //===================================================== END{PROCESS DATA}

    Eigen::VectorXs initial_origin_state = origin_KF->getState();
    Eigen::VectorXs perturbated_state(16);
    
    for (int i = 0 ; i < 3 ; i++)
    {
        for (int j = 0 ; j < 3 ; j++)
        {
            for (int k = 1 ; k < 3 ; k++)
            {
                perturbated_state = initial_origin_state;
                perturbated_state(i) += 1.0;
                perturbated_state(j) += 1.0;
                perturbated_state(k) += 1.0;

                origin_KF->setState(perturbated_state);
                last_KF->setState(initial_final_state);

                //===================================================== SOLVER PART
     
                summary = ceres_manager_wolf_diff->solve();
                std::cout << summary.BriefReport() << std::endl;

                EXPECT_TRUE( (origin_state_afterCeres.head(3) - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )) << 
                "origin_state_afterCeres position state : " << origin_state_afterCeres.head(3).transpose() << "\n origin position state : " << origin_KF->getPPtr()->getVector().transpose() << std::endl;
                ASSERT_TRUE( (origin_state_afterCeres.segment(3,4) - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )) <<
                "origin_state_afterCeres quaternion : " << origin_state_afterCeres.segment(3,4).transpose() << "\n origin quaternion state : " << origin_KF->getOPtr()->getVector().transpose() << std::endl;
            }
        }
    }

    // COMPUTE COVARIANCES
    //std::cout << "\t\t\t ______computing covariances______" << std::endl;
    //ceres_manager_wolf_diff->computeCovariances(ALL);//ALL_MARGINALS, ALL
    //std::cout << "\t\t\t ______computed!______" << std::endl;

    //===================================================== END{SOLVER PART}
}

/* Introduce a perturbation in Origin_KF velocity and fix the last KF. 
 * Ideally the optimization should be able to make origin_KF velocity converge to its correct value (value it would have taken if it had not been perturbated). 
 * 
 * TEST PASSED
 */

TEST_F(ProcessorIMU_Odom_tests,Plateform_2s_move_PerturbateVelocityOrigin_fixLast)
{

    using std::shared_ptr;
    using std::make_shared;
    using std::static_pointer_cast;
    
    std::string wolf_root = _WOLF_ROOT_DIR;
    char* imu_filepath;
    std::string filepath_string(wolf_root + "/src/test/data/IMU/Test_plateforme/imu_plateform_2s.txt");
    imu_filepath   = new char[filepath_string.length() + 1];
    std::strcpy(imu_filepath, filepath_string.c_str());
    std::ifstream imu_data_input;

    imu_data_input.open(imu_filepath);
    WOLF_INFO("imu file: ", imu_filepath)

    if(!imu_data_input.is_open()){
        std::cerr << "Failed to open data files... Exiting" << std::endl;
        ADD_FAILURE();
    }

    //===================================================== SETTING PROBLEM

    // reset origin of problem
    Eigen::VectorXs x_origin((Eigen::Matrix<wolf::Scalar,16,1>()<<0,0,0, 0,0,0,1, 0,0,0, 0,0,0, 0,0,0).finished());

    // initial conditions defined from data file
    // remember that matlab's quaternion is W,X,Y,Z and the one in Eigen has X,Y,Z,W form
    imu_data_input >> x_origin[0] >> x_origin[1] >> x_origin[2] >> x_origin[6] >> x_origin[3] >> x_origin[4] >> x_origin[5] >> x_origin[7] >> x_origin[8] >> x_origin[9];

    t.set(0);
    FrameBasePtr origin_KF = processor_ptr_imu->setOrigin(x_origin, t);
    processor_ptr_odom3D->setOrigin(origin_KF);
    FrameIMUPtr origin_imuKF = std::static_pointer_cast<FrameIMU>(origin_KF);
    
    //===================================================== END{SETTING PROBLEM}

    //===================================================== PROCESS DATA
    // PROCESS DATA

    Eigen::Vector6s data_imu, data_odom3D;
    data_imu << 0,0,-wolf::gravity()(2), 0,0,0;
    data_odom3D << 0,0.06,0, 0,0,0;

    Scalar input_clock;
    TimeStamp ts(0);
    wolf::CaptureIMUPtr imu_ptr = std::make_shared<CaptureIMU>(ts, sen_imu, data_imu);
    wolf::CaptureMotionPtr mot_ptr = std::make_shared<CaptureMotion>(t, sen_odom3D, data_odom3D);
    

    while( !imu_data_input.eof() )
    {
        // PROCESS IMU DATA
        // Time and data variables
        imu_data_input >> input_clock >> data_imu[0] >> data_imu[1] >> data_imu[2] >> data_imu[3] >> data_imu[4] >> data_imu[5]; //Ax, Ay, Az, Gx, Gy, Gz

        ts.set(input_clock);
        imu_ptr->setTimeStamp(ts);
        imu_ptr->setData(data_imu);

        // process data in capture
        imu_ptr->getTimeStamp();
        sen_imu->process(imu_ptr);
    }

    //IMU data have all been processed. Now we process the odom3D data
    // PROCESS ODOM 3D DATA
    mot_ptr->setTimeStamp(ts);
    mot_ptr->setData(data_odom3D);
    sen_odom3D->process(mot_ptr);

    //closing file
    imu_data_input.close();

    FrameIMUPtr last_KF = std::static_pointer_cast<FrameIMU>(wolf_problem_ptr_->getTrajectoryPtr()->closestKeyFrameToTimeStamp(ts));

    last_KF->fix();
    Eigen::VectorXs initial_final_state(16);
    initial_final_state = last_KF->getState();

    // call solver to get values after optimization. Wemwill compare the following output to these
    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
    std::cout << summary.BriefReport() << std::endl;

    Eigen::VectorXs origin_state_afterCeres(16);
    origin_state_afterCeres = origin_KF->getState();

    // velocity is modified : due to bias non initialized and noises. And due to motion constraint and fixing KF1's Position, velocity is used to satisfy conditions
    EXPECT_FALSE( (last_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS*10 ) ) <<
    "last_KF velocity state : " << last_KF->getVPtr()->getVector().transpose() << "\n origin velocity : " << origin_KF->getVPtr()->getVector().transpose() << std::endl;
    EXPECT_TRUE( (last_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector() - data_odom3D.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS)) << 
    "last_KF position state : " << last_KF->getPPtr()->getVector().transpose() << "\n origin position state : " << origin_KF->getPPtr()->getVector().transpose() << std::endl;
    ASSERT_TRUE( (last_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS)) <<
    "last_KF quaternion : " << last_KF->getOPtr()->getVector().transpose() << "\n origin quaternion state : " << origin_KF->getOPtr()->getVector().transpose() << std::endl;

    //===================================================== END{PROCESS DATA}

    Eigen::VectorXs initial_origin_state = origin_KF->getState();
    Eigen::VectorXs perturbated_state(16);
    
    for (int i = 7 ; i < 10 ; i++)
    {
        for (int j = 7 ; j < 10 ; j++)
        {
            for (int k = 7 ; k < 10 ; k++)
            {
                perturbated_state = initial_origin_state;
                perturbated_state(i) += 1.0;
                perturbated_state(j) += 2.0;
                perturbated_state(k) += 3.0;

                origin_KF->setState(perturbated_state);
                last_KF->setState(initial_final_state); //should not be needed sincce we fixed this KF

                //===================================================== SOLVER PART
     
                summary = ceres_manager_wolf_diff->solve();
                std::cout << summary.BriefReport() << std::endl;

                EXPECT_TRUE( (origin_state_afterCeres.segment(7,3) - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS ) ) <<
                "origin_state_afterCeres velocity state : " << origin_state_afterCeres.segment(7,3).transpose() << "\n origin velocity : " << origin_KF->getVPtr()->getVector().transpose() << std::endl;
                EXPECT_TRUE( (origin_state_afterCeres.head(3) - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )) << 
                "origin_state_afterCeres position state : " << origin_state_afterCeres.head(3).transpose() << "\n origin position state : " << origin_KF->getPPtr()->getVector().transpose() << std::endl;
                ASSERT_TRUE( (origin_state_afterCeres.segment(3,4) - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )) <<
                "origin_state_afterCeres quaternion : " << origin_state_afterCeres.segment(3,4).transpose() << "\n origin quaternion state : " << origin_KF->getOPtr()->getVector().transpose() << std::endl;
            }
        }
    }

    // COMPUTE COVARIANCES
    //std::cout << "\t\t\t ______computing covariances______" << std::endl;
    //ceres_manager_wolf_diff->computeCovariances(ALL);//ALL_MARGINALS, ALL
    //std::cout << "\t\t\t ______computed!______" << std::endl;

    //===================================================== END{SOLVER PART}
}

/* Introduce a perturbation in Origin_KF orientation and fix the last KF. 
 * Ideally the optimization should be able to make origin_KF velocity converge to its correct value (value it would have taken if it had not been perturbated). 
 * Origin_KF is unfixed. Last_KF is fixed.
 * 
 * TEST PASSED
 */

TEST_F(ProcessorIMU_Odom_tests,Plateform_2s_move_PerturbateOrientationOrigin_fixLast)
{

    using std::shared_ptr;
    using std::make_shared;
    using std::static_pointer_cast;
    
    std::string wolf_root = _WOLF_ROOT_DIR;
    char* imu_filepath;
    std::string filepath_string(wolf_root + "/src/test/data/IMU/Test_plateforme/imu_plateform_2s.txt");
    imu_filepath   = new char[filepath_string.length() + 1];
    std::strcpy(imu_filepath, filepath_string.c_str());
    std::ifstream imu_data_input;

    imu_data_input.open(imu_filepath);
    WOLF_INFO("imu file: ", imu_filepath)

    if(!imu_data_input.is_open()){
        std::cerr << "Failed to open data files... Exiting" << std::endl;
        ADD_FAILURE();
    }

    //===================================================== SETTING PROBLEM

    // reset origin of problem
    Eigen::VectorXs x_origin((Eigen::Matrix<wolf::Scalar,16,1>()<<0,0,0, 0,0,0,1, 0,0,0, 0,0,0, 0,0,0).finished());

    // initial conditions defined from data file
    // remember that matlab's quaternion is W,X,Y,Z and the one in Eigen has X,Y,Z,W form
    imu_data_input >> x_origin[0] >> x_origin[1] >> x_origin[2] >> x_origin[6] >> x_origin[3] >> x_origin[4] >> x_origin[5] >> x_origin[7] >> x_origin[8] >> x_origin[9];

    t.set(0);
    FrameBasePtr origin_KF = processor_ptr_imu->setOrigin(x_origin, t);
    processor_ptr_odom3D->setOrigin(origin_KF);
    FrameIMUPtr origin_imuKF = std::static_pointer_cast<FrameIMU>(origin_KF);
    
    //===================================================== END{SETTING PROBLEM}

    //===================================================== PROCESS DATA
    // PROCESS DATA

    Eigen::Vector6s data_imu, data_odom3D;
    data_imu << 0,0,-wolf::gravity()(2), 0,0,0;
    data_odom3D << 0,0.06,0, 0,0,0;

    Scalar input_clock;
    TimeStamp ts(0);
    wolf::CaptureIMUPtr imu_ptr = std::make_shared<CaptureIMU>(ts, sen_imu, data_imu);
    wolf::CaptureMotionPtr mot_ptr = std::make_shared<CaptureMotion>(t, sen_odom3D, data_odom3D);


    while( !imu_data_input.eof() )
    {
        // PROCESS IMU DATA
        // Time and data variables
        imu_data_input >> input_clock >> data_imu[0] >> data_imu[1] >> data_imu[2] >> data_imu[3] >> data_imu[4] >> data_imu[5]; //Ax, Ay, Az, Gx, Gy, Gz

        ts.set(input_clock);
        imu_ptr->setTimeStamp(ts);
        imu_ptr->setData(data_imu);

        // process data in capture
        imu_ptr->getTimeStamp();
        sen_imu->process(imu_ptr);
    }

    //IMU data have all been processed. Now we process the odom3D data
    // PROCESS ODOM 3D DATA
    mot_ptr->setTimeStamp(ts);
    mot_ptr->setData(data_odom3D);
    sen_odom3D->process(mot_ptr);

    //closing file
    imu_data_input.close();

    FrameIMUPtr last_KF = std::static_pointer_cast<FrameIMU>(wolf_problem_ptr_->getTrajectoryPtr()->closestKeyFrameToTimeStamp(ts));
    last_KF->fix();
    origin_KF->unfix();
    
    Eigen::VectorXs initial_final_state(16);
    initial_final_state = last_KF->getState();

    // call solver to get values after optimization. Wemwill compare the following output to these
    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
    std::cout << summary.BriefReport() << std::endl;

    Eigen::VectorXs origin_state_afterCeres(16);
    origin_state_afterCeres = origin_KF->getState();

    // velocity is modified : due to bias non initialized and noises. And due to motion constraint and fixing KF1's Position, velocity is used to satisfy conditions
    EXPECT_FALSE( (last_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS*10 ) ) <<
    "last_KF velocity state : " << last_KF->getVPtr()->getVector().transpose() << "\n origin velocity : " << origin_KF->getVPtr()->getVector().transpose() << std::endl;
    EXPECT_TRUE( (last_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector() - data_odom3D.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS)) << 
    "last_KF position state : " << last_KF->getPPtr()->getVector().transpose() << "\n origin position state : " << origin_KF->getPPtr()->getVector().transpose() << std::endl;
    ASSERT_TRUE( (last_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS)) <<
    "last_KF quaternion : " << last_KF->getOPtr()->getVector().transpose() << "\n origin quaternion state : " << origin_KF->getOPtr()->getVector().transpose() << std::endl;

    //===================================================== END{PROCESS DATA}

    Eigen::VectorXs initial_origin_state = origin_KF->getState();
    Eigen::VectorXs perturbated_state(16);
    Eigen::Map<Eigen::Quaternions> quat_map(perturbated_state.data() + 3);

    
    for (int i = 0 ; i < 3 ; i++)
    {
        for (int j = 0 ; j < 3 ; j++)
        {
            for (int k = 0 ; k < 3 ; k++)
            {
                Eigen::Vector3s orientation_perturbation((Eigen::Vector3s()<<0,0,0).finished());
                orientation_perturbation(0) += i*0.2;
                orientation_perturbation(1) += j*0.1;
                orientation_perturbation(2) += k*0.15;

                perturbated_state = initial_origin_state;
                quat_map = quat_map * v2q(orientation_perturbation);

                origin_KF->setState(perturbated_state);
                last_KF->setState(initial_final_state);

                //===================================================== SOLVER PART
     
                summary = ceres_manager_wolf_diff->solve();
                std::cout << summary.BriefReport() << std::endl;

                EXPECT_TRUE( (origin_state_afterCeres.segment(7,3) - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS*10 ) ) << "iteration " << i<<"."<<j<<"."<<k <<
                "\norigin_state_afterCeres velocity state : " << origin_state_afterCeres.segment(7,3).transpose() << "\n origin velocity : " << origin_KF->getVPtr()->getVector().transpose() << std::endl;
                EXPECT_TRUE( (origin_state_afterCeres.head(3) - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, 0.000000001 )) << 
                "origin_state_afterCeres position state : " << origin_state_afterCeres.head(3).transpose() << "\n origin position state : " << origin_KF->getPPtr()->getVector().transpose() << std::endl;
                ASSERT_TRUE( (origin_state_afterCeres.segment(3,4) - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, 0.000000001 )) <<
                "origin_state_afterCeres quaternion : " << origin_state_afterCeres.segment(3,4).transpose() << "\n origin quaternion state : " << origin_KF->getOPtr()->getVector().transpose() << std::endl;
            }
        }
    }

    // COMPUTE COVARIANCES
    //std::cout << "\t\t\t ______computing covariances______" << std::endl;
    //ceres_manager_wolf_diff->computeCovariances(ALL);//ALL_MARGINALS, ALL
    //std::cout << "\t\t\t ______computed!______" << std::endl;

    //===================================================== END{SOLVER PART}
}


/* Introduce a perturbation in Origin_KF orientation and fix the last KF. 
 * Ideally the optimization should be able to make origin_KF velocity converge to its correct value (value it would have taken if it had not been perturbated). 
 * Origin_KF is unfixed. Last_KF is fixed.
 * 
 * TEST PASSED
 */

 TEST_F(ProcessorIMU_Odom_tests,Plateform_2s_move_PerturbateAccBiasOrigin_fixLast)
{

    using std::shared_ptr;
    using std::make_shared;
    using std::static_pointer_cast;
    
    std::string wolf_root = _WOLF_ROOT_DIR;
    char* imu_filepath;
    std::string filepath_string(wolf_root + "/src/test/data/IMU/Test_plateforme/imu_plateform_2s.txt");
    imu_filepath   = new char[filepath_string.length() + 1];
    std::strcpy(imu_filepath, filepath_string.c_str());
    std::ifstream imu_data_input;

    imu_data_input.open(imu_filepath);
    WOLF_INFO("imu file: ", imu_filepath)

    if(!imu_data_input.is_open()){
        std::cerr << "Failed to open data files... Exiting" << std::endl;
        ADD_FAILURE();
    }

    //===================================================== SETTING PROBLEM

    // reset origin of problem
    Eigen::VectorXs x_origin((Eigen::Matrix<wolf::Scalar,16,1>()<<0,0,0, 0,0,0,1, 0,0,0, 0,0,0, 0,0,0).finished());

    // initial conditions defined from data file
    // remember that matlab's quaternion is W,X,Y,Z and the one in Eigen has X,Y,Z,W form
    imu_data_input >> x_origin[0] >> x_origin[1] >> x_origin[2] >> x_origin[6] >> x_origin[3] >> x_origin[4] >> x_origin[5] >> x_origin[7] >> x_origin[8] >> x_origin[9];

    t.set(0);
    FrameBasePtr origin_KF = processor_ptr_imu->setOrigin(x_origin, t);
    processor_ptr_odom3D->setOrigin(origin_KF);
    FrameIMUPtr origin_imuKF = std::static_pointer_cast<FrameIMU>(origin_KF);
    
    //===================================================== END{SETTING PROBLEM}

    //===================================================== PROCESS DATA
    // PROCESS DATA

    Eigen::Vector6s data_imu, data_odom3D;
    data_imu << 0,0,-wolf::gravity()(2), 0,0,0;
    data_odom3D << 0,0.06,0, 0,0,0;

    Scalar input_clock;
    TimeStamp ts(0);
    wolf::CaptureIMUPtr imu_ptr = std::make_shared<CaptureIMU>(ts, sen_imu, data_imu);
    wolf::CaptureMotionPtr mot_ptr = std::make_shared<CaptureMotion>(t, sen_odom3D, data_odom3D);
    

    while( !imu_data_input.eof() )
    {
        // PROCESS IMU DATA
        // Time and data variables
        imu_data_input >> input_clock >> data_imu[0] >> data_imu[1] >> data_imu[2] >> data_imu[3] >> data_imu[4] >> data_imu[5]; //Ax, Ay, Az, Gx, Gy, Gz

        ts.set(input_clock);
        imu_ptr->setTimeStamp(ts);
        imu_ptr->setData(data_imu);

        // process data in capture
        imu_ptr->getTimeStamp();
        sen_imu->process(imu_ptr);
    }

    //IMU data have all been processed. Now we process the odom3D data
    // PROCESS ODOM 3D DATA
    mot_ptr->setTimeStamp(ts);
    mot_ptr->setData(data_odom3D);
    sen_odom3D->process(mot_ptr);

    //closing file
    imu_data_input.close();

    FrameIMUPtr last_KF = std::static_pointer_cast<FrameIMU>(wolf_problem_ptr_->getTrajectoryPtr()->closestKeyFrameToTimeStamp(ts));

    last_KF->fix();
    Eigen::VectorXs initial_final_state(16);
    initial_final_state = last_KF->getState();

    // call solver to get values after optimization. Wemwill compare the following output to these
    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
    std::cout << summary.BriefReport() << std::endl;

    Eigen::VectorXs origin_state_afterCeres(16);
    origin_state_afterCeres = origin_KF->getState();

    // velocity is modified : due to bias non initialized and noises. And due to motion constraint and fixing KF1's Position, velocity is used to satisfy conditions
    EXPECT_FALSE( (last_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS*10 ) ) <<
    "last_KF velocity state : " << last_KF->getVPtr()->getVector().transpose() << "\n origin velocity : " << origin_KF->getVPtr()->getVector().transpose() << std::endl;
    EXPECT_TRUE( (last_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector() - data_odom3D.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS)) << 
    "last_KF position state : " << last_KF->getPPtr()->getVector().transpose() << "\n origin position state : " << origin_KF->getPPtr()->getVector().transpose() << std::endl;
    ASSERT_TRUE( (last_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS)) <<
    "last_KF quaternion : " << last_KF->getOPtr()->getVector().transpose() << "\n origin quaternion state : " << origin_KF->getOPtr()->getVector().transpose() << std::endl;

    //===================================================== END{PROCESS DATA}

    Eigen::VectorXs initial_origin_state = origin_KF->getState();
    Eigen::VectorXs perturbated_state(16);
    
    for (int i = 10 ; i < 13 ; i++)
    {
        for (int j = 10 ; j < 13 ; j++)
        {
            for (int k = 10 ; k < 13 ; k++)
            {
                perturbated_state = initial_origin_state;
                perturbated_state(i) += 1.0;
                perturbated_state(j) += 1.0;
                perturbated_state(k) += 1.0;

                origin_KF->setState(perturbated_state);
                last_KF->setState(initial_final_state); //should not be needed sincce we fixed this KF

                //===================================================== SOLVER PART
     
                summary = ceres_manager_wolf_diff->solve();
                std::cout << summary.BriefReport() << std::endl;

                EXPECT_TRUE( (origin_state_afterCeres.segment(7,3) - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS*100 ) ) <<
                "origin_state_afterCeres velocity state : " << origin_state_afterCeres.segment(7,3).transpose() << "\n origin velocity : " << origin_KF->getVPtr()->getVector().transpose() << std::endl;
                EXPECT_TRUE( (origin_state_afterCeres.head(3) - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )) << 
                "origin_state_afterCeres position state : " << origin_state_afterCeres.head(3).transpose() << "\n origin position state : " << origin_KF->getPPtr()->getVector().transpose() << std::endl;
                ASSERT_TRUE( (origin_state_afterCeres.segment(3,4) - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )) <<
                "origin_state_afterCeres quaternion : " << origin_state_afterCeres.segment(3,4).transpose() << "\n origin quaternion state : " << origin_KF->getOPtr()->getVector().transpose() << std::endl;
                EXPECT_TRUE( (origin_state_afterCeres.segment(10,3) - origin_imuKF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) <<
                "origin_state_afterCeres acc bias : " << origin_state_afterCeres.segment(10,3).transpose() << "\n origin acc bias state : " << origin_imuKF->getAccBiasPtr()->getVector().transpose() << std::endl;
            }
        }
    }

    // COMPUTE COVARIANCES
    //std::cout << "\t\t\t ______computing covariances______" << std::endl;
    //ceres_manager_wolf_diff->computeCovariances(ALL);//ALL_MARGINALS, ALL
    //std::cout << "\t\t\t ______computed!______" << std::endl;

    //===================================================== END{SOLVER PART}
}


TEST_F(ProcessorIMU_Odom_tests,Plateform_2s_move_PerturbateGyroBiasOrigin_fixLast)
{

    using std::shared_ptr;
    using std::make_shared;
    using std::static_pointer_cast;
    
    std::string wolf_root = _WOLF_ROOT_DIR;
    char* imu_filepath;
    std::string filepath_string(wolf_root + "/src/test/data/IMU/Test_plateforme/imu_plateform_2s.txt");
    imu_filepath   = new char[filepath_string.length() + 1];
    std::strcpy(imu_filepath, filepath_string.c_str());
    std::ifstream imu_data_input;

    imu_data_input.open(imu_filepath);
    WOLF_INFO("imu file: ", imu_filepath)

    if(!imu_data_input.is_open()){
        std::cerr << "Failed to open data files... Exiting" << std::endl;
        ADD_FAILURE();
    }

    //===================================================== SETTING PROBLEM

    // reset origin of problem
    Eigen::VectorXs x_origin((Eigen::Matrix<wolf::Scalar,16,1>()<<0,0,0, 0,0,0,1, 0,0,0, 0,0,0, 0,0,0).finished());

    // initial conditions defined from data file
    // remember that matlab's quaternion is W,X,Y,Z and the one in Eigen has X,Y,Z,W form
    imu_data_input >> x_origin[0] >> x_origin[1] >> x_origin[2] >> x_origin[6] >> x_origin[3] >> x_origin[4] >> x_origin[5] >> x_origin[7] >> x_origin[8] >> x_origin[9];

    t.set(0);
    FrameBasePtr origin_KF = processor_ptr_imu->setOrigin(x_origin, t);
    processor_ptr_odom3D->setOrigin(origin_KF);
    FrameIMUPtr origin_imuKF = std::static_pointer_cast<FrameIMU>(origin_KF);
    
    //===================================================== END{SETTING PROBLEM}

    //===================================================== PROCESS DATA
    // PROCESS DATA

    Eigen::Vector6s data_imu, data_odom3D;
    data_imu << 0,0,-wolf::gravity()(2), 0,0,0;
    data_odom3D << 0,0.06,0, 0,0,0;

    Scalar input_clock;
    TimeStamp ts(0);
    wolf::CaptureIMUPtr imu_ptr = std::make_shared<CaptureIMU>(ts, sen_imu, data_imu);
    wolf::CaptureMotionPtr mot_ptr = std::make_shared<CaptureMotion>(t, sen_odom3D, data_odom3D);
    

    while( !imu_data_input.eof() )
    {
        // PROCESS IMU DATA
        // Time and data variables
        imu_data_input >> input_clock >> data_imu[0] >> data_imu[1] >> data_imu[2] >> data_imu[3] >> data_imu[4] >> data_imu[5]; //Ax, Ay, Az, Gx, Gy, Gz

        ts.set(input_clock);
        imu_ptr->setTimeStamp(ts);
        imu_ptr->setData(data_imu);

        // process data in capture
        imu_ptr->getTimeStamp();
        sen_imu->process(imu_ptr);
    }

    //IMU data have all been processed. Now we process the odom3D data
    // PROCESS ODOM 3D DATA
    mot_ptr->setTimeStamp(ts);
    mot_ptr->setData(data_odom3D);
    sen_odom3D->process(mot_ptr);

    //closing file
    imu_data_input.close();

    FrameIMUPtr last_KF = std::static_pointer_cast<FrameIMU>(wolf_problem_ptr_->getTrajectoryPtr()->closestKeyFrameToTimeStamp(ts));

    last_KF->fix();
    Eigen::VectorXs initial_final_state(16);
    initial_final_state = last_KF->getState();

    // call solver to get values after optimization. Wemwill compare the following output to these
    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
    std::cout << summary.BriefReport() << std::endl;

    Eigen::VectorXs origin_state_afterCeres(16);
    origin_state_afterCeres = origin_KF->getState();

    // velocity is modified : due to bias non initialized and noises. And due to motion constraint and fixing KF1's Position, velocity is used to satisfy conditions
    EXPECT_FALSE( (last_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS*10 ) ) <<
    "last_KF velocity state : " << last_KF->getVPtr()->getVector().transpose() << "\n origin velocity : " << origin_KF->getVPtr()->getVector().transpose() << std::endl;
    EXPECT_TRUE( (last_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector() - data_odom3D.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS)) << 
    "last_KF position state : " << last_KF->getPPtr()->getVector().transpose() << "\n origin position state : " << origin_KF->getPPtr()->getVector().transpose() << std::endl;
    ASSERT_TRUE( (last_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS)) <<
    "last_KF quaternion : " << last_KF->getOPtr()->getVector().transpose() << "\n origin quaternion state : " << origin_KF->getOPtr()->getVector().transpose() << std::endl;

    //===================================================== END{PROCESS DATA}

    Eigen::VectorXs initial_origin_state = origin_KF->getState();
    Eigen::VectorXs perturbated_state(16);
    
    for (int i = 13 ; i < 16 ; i++)
    {
        for (int j = 13 ; j < 16 ; j++)
        {
            for (int k = 13 ; k < 16 ; k++)
            {
                perturbated_state = initial_origin_state;
                perturbated_state(i) += 1.0;
                perturbated_state(j) += 1.0;
                perturbated_state(k) += 1.0;

                origin_KF->setState(perturbated_state);
                last_KF->setState(initial_final_state); //should not be needed sincce we fixed this KF

                //===================================================== SOLVER PART
     
                summary = ceres_manager_wolf_diff->solve();
                std::cout << summary.BriefReport() << std::endl;

                EXPECT_TRUE( (origin_state_afterCeres.segment(7,3) - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS ) ) <<
                "origin_state_afterCeres velocity state : " << origin_state_afterCeres.segment(7,3).transpose() << "\n origin velocity : " << origin_KF->getVPtr()->getVector().transpose() << std::endl;
                EXPECT_TRUE( (origin_state_afterCeres.head(3) - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )) << 
                "origin_state_afterCeres position state : " << origin_state_afterCeres.head(3).transpose() << "\n origin position state : " << origin_KF->getPPtr()->getVector().transpose() << std::endl;
                ASSERT_TRUE( (origin_state_afterCeres.segment(3,4) - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )) <<
                "origin_state_afterCeres quaternion : " << origin_state_afterCeres.segment(3,4).transpose() << "\n origin quaternion state : " << origin_KF->getOPtr()->getVector().transpose() << std::endl;
                EXPECT_TRUE( (origin_state_afterCeres.segment(10,3) - origin_imuKF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )) <<
                "origin_state_afterCeres acc bias : " << origin_state_afterCeres.segment(10,3).transpose() << "\n origin acc bias state : " << origin_imuKF->getAccBiasPtr()->getVector().transpose() << std::endl;
                EXPECT_TRUE( (origin_state_afterCeres.segment(13,3) - origin_imuKF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )) <<
                "origin_state_afterCeres gyro bias : " << origin_state_afterCeres.segment(13,3).transpose() << "\n origin gyro bias state : " << origin_imuKF->getGyroBiasPtr()->getVector().transpose() << std::endl;
            }
        }
    }
}

/* Introduce a perturbation in Origin_KF
 * Ideally the optimization should be able to make origin_KF position converge to its correct value (value it would have taken if it had not been perturbated). 
 * 
 * Quaternion error depends on the perturbation that was introduced.
 * TEST PASSED
 */

TEST_F(ProcessorIMU_Odom_tests,Plateform_2s_move_PerturbatePositionOrigin_UnfixPerturbatedOnly)
{

    using std::shared_ptr;
    using std::make_shared;
    using std::static_pointer_cast;
    
    std::string wolf_root = _WOLF_ROOT_DIR;
    char* imu_filepath;
    std::string filepath_string(wolf_root + "/src/test/data/IMU/Test_plateforme/imu_plateform_2s.txt");
    imu_filepath   = new char[filepath_string.length() + 1];
    std::strcpy(imu_filepath, filepath_string.c_str());
    std::ifstream imu_data_input;

    imu_data_input.open(imu_filepath);
    WOLF_INFO("imu file: ", imu_filepath)

    if(!imu_data_input.is_open()){
        std::cerr << "Failed to open data files... Exiting" << std::endl;
        ADD_FAILURE();
    }

    //===================================================== SETTING PROBLEM

    // reset origin of problem
    Eigen::VectorXs x_origin((Eigen::Matrix<wolf::Scalar,16,1>()<<0,0,0, 0,0,0,1, 0,0,0, 0,0,0, 0,0,0).finished());

    // initial conditions defined from data file
    // remember that matlab's quaternion is W,X,Y,Z and the one in Eigen has X,Y,Z,W form
    imu_data_input >> x_origin[0] >> x_origin[1] >> x_origin[2] >> x_origin[6] >> x_origin[3] >> x_origin[4] >> x_origin[5] >> x_origin[7] >> x_origin[8] >> x_origin[9];

    t.set(0);
    FrameBasePtr origin_KF = processor_ptr_imu->setOrigin(x_origin, t);
    processor_ptr_odom3D->setOrigin(origin_KF);
    FrameIMUPtr origin_imuKF = std::static_pointer_cast<FrameIMU>(origin_KF);
    
    //===================================================== END{SETTING PROBLEM}

    //===================================================== PROCESS DATA
    // PROCESS DATA

    Eigen::Vector6s data_imu, data_odom3D;
    data_imu << 0,0,-wolf::gravity()(2), 0,0,0;
    data_odom3D << 0,0.06,0, 0,0,0;

    Scalar input_clock;
    TimeStamp ts(0);
    wolf::CaptureIMUPtr imu_ptr = std::make_shared<CaptureIMU>(ts, sen_imu, data_imu);
    wolf::CaptureMotionPtr mot_ptr = std::make_shared<CaptureMotion>(t, sen_odom3D, data_odom3D);


    while( !imu_data_input.eof() )
    {
        // PROCESS IMU DATA
        // Time and data variables
        imu_data_input >> input_clock >> data_imu[0] >> data_imu[1] >> data_imu[2] >> data_imu[3] >> data_imu[4] >> data_imu[5]; //Ax, Ay, Az, Gx, Gy, Gz

        ts.set(input_clock);
        imu_ptr->setTimeStamp(ts);
        imu_ptr->setData(data_imu);

        // process data in capture
        imu_ptr->getTimeStamp();
        sen_imu->process(imu_ptr);
    }

    //IMU data have all been processed. Now we process the odom3D data
    // PROCESS ODOM 3D DATA
    mot_ptr->setTimeStamp(ts);
    mot_ptr->setData(data_odom3D);
    sen_odom3D->process(mot_ptr);

    //closing file
    imu_data_input.close();

    FrameIMUPtr last_KF = std::static_pointer_cast<FrameIMU>(wolf_problem_ptr_->getTrajectoryPtr()->closestKeyFrameToTimeStamp(ts));
    Eigen::VectorXs initial_final_state(16);
    initial_final_state = last_KF->getState();

    // call solver to get values after optimization. Wemwill compare the following output to these
    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
    std::cout << summary.BriefReport() << std::endl;

    Eigen::VectorXs origin_state_afterCeres(16);
    origin_state_afterCeres = origin_KF->getState();

    //Unfix only StateBlcok of interest
    last_KF->fix();
    origin_KF->fix();
    origin_KF->getPPtr()->unfix();
    //===================================================== END{PROCESS DATA}

    Eigen::VectorXs initial_origin_state = origin_KF->getState();
    Eigen::VectorXs perturbated_state(16);
    
    for (int i = 0 ; i < 3 ; i++)
    {
        for (int j = 0 ; j < 3 ; j++)
        {
            for (int k = 1 ; k < 3 ; k++)
            {
                perturbated_state = initial_origin_state;
                perturbated_state(i) += 1.0;
                perturbated_state(j) += 1.0;
                perturbated_state(k) += 1.0;

                origin_KF->setState(perturbated_state);
                last_KF->setState(initial_final_state);

                //===================================================== SOLVER PART
     
                summary = ceres_manager_wolf_diff->solve();
                std::cout << summary.BriefReport() << std::endl;

                EXPECT_TRUE( (origin_state_afterCeres.head(3) - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )) << 
                "origin_state_afterCeres position state : " << origin_state_afterCeres.head(3).transpose() << "\n origin position state : " << origin_KF->getPPtr()->getVector().transpose() << std::endl;
            }
        }
    }

    // COMPUTE COVARIANCES
    //std::cout << "\t\t\t ______computing covariances______" << std::endl;
    //ceres_manager_wolf_diff->computeCovariances(ALL);//ALL_MARGINALS, ALL
    //std::cout << "\t\t\t ______computed!______" << std::endl;

    //===================================================== END{SOLVER PART}
}

/* Introduce a perturbation in Origin_KF
 * Ideally the optimization should be able to make origin_KF velocity converge to its correct value (value it would have taken if it had not been perturbated). 
 * 
 * The perturbated StateBlock is the only one unfixed in this test.
 * Error in velocity StateBlock is in 1e-4
 */

TEST_F(ProcessorIMU_Odom_tests,Plateform_2s_move_PerturbateVelocityOrigin_UnfixPerturbatedOnly)
{
    using std::shared_ptr;
    using std::make_shared;
    using std::static_pointer_cast;
    
    std::string wolf_root = _WOLF_ROOT_DIR;
    char* imu_filepath;
    std::string filepath_string(wolf_root + "/src/test/data/IMU/Test_plateforme/imu_plateform_2s.txt");
    imu_filepath   = new char[filepath_string.length() + 1];
    std::strcpy(imu_filepath, filepath_string.c_str());
    std::ifstream imu_data_input;

    imu_data_input.open(imu_filepath);
    WOLF_INFO("imu file: ", imu_filepath)

    if(!imu_data_input.is_open()){
        std::cerr << "Failed to open data files... Exiting" << std::endl;
        ADD_FAILURE();
    }

    //===================================================== SETTING PROBLEM

    // reset origin of problem
    Eigen::VectorXs x_origin((Eigen::Matrix<wolf::Scalar,16,1>()<<0,0,0, 0,0,0,1, 0,0,0, 0,0,0, 0,0,0).finished());

    // initial conditions defined from data file
    // remember that matlab's quaternion is W,X,Y,Z and the one in Eigen has X,Y,Z,W form
    imu_data_input >> x_origin[0] >> x_origin[1] >> x_origin[2] >> x_origin[6] >> x_origin[3] >> x_origin[4] >> x_origin[5] >> x_origin[7] >> x_origin[8] >> x_origin[9];

    t.set(0);
    FrameBasePtr origin_KF = processor_ptr_imu->setOrigin(x_origin, t);
    processor_ptr_odom3D->setOrigin(origin_KF);
    FrameIMUPtr origin_imuKF = std::static_pointer_cast<FrameIMU>(origin_KF);
    
    //===================================================== END{SETTING PROBLEM}

    //===================================================== PROCESS DATA
    // PROCESS DATA

    Eigen::Vector6s data_imu, data_odom3D;
    data_imu << 0,0,-wolf::gravity()(2), 0,0,0;
    data_odom3D << 0,0.06,0, 0,0,0;

    Scalar input_clock;
    TimeStamp ts(0);
    wolf::CaptureIMUPtr imu_ptr = std::make_shared<CaptureIMU>(ts, sen_imu, data_imu);
    wolf::CaptureMotionPtr mot_ptr = std::make_shared<CaptureMotion>(t, sen_odom3D, data_odom3D);

    while( !imu_data_input.eof() )
    {
        // PROCESS IMU DATA
        // Time and data variables
        imu_data_input >> input_clock >> data_imu[0] >> data_imu[1] >> data_imu[2] >> data_imu[3] >> data_imu[4] >> data_imu[5]; //Ax, Ay, Az, Gx, Gy, Gz

        ts.set(input_clock);
        imu_ptr->setTimeStamp(ts);
        imu_ptr->setData(data_imu);

        // process data in capture
        imu_ptr->getTimeStamp();
        sen_imu->process(imu_ptr);
    }

    //IMU data have all been processed. Now we process the odom3D data
    // PROCESS ODOM 3D DATA
    mot_ptr->setTimeStamp(ts);
    mot_ptr->setData(data_odom3D);
    sen_odom3D->process(mot_ptr);

    //closing file
    imu_data_input.close();

    FrameIMUPtr last_KF = std::static_pointer_cast<FrameIMU>(wolf_problem_ptr_->getTrajectoryPtr()->closestKeyFrameToTimeStamp(ts));

    Eigen::VectorXs initial_final_state(16);
    initial_final_state = last_KF->getState();
    Eigen::VectorXs initial_origin_state = origin_KF->getState();
    Eigen::VectorXs perturbated_state(16);

    //Unfix only StateBlock of interest
    last_KF->fix();
    origin_KF->fix();
    origin_KF->getVPtr()->unfix();

    // call solver to get values after optimization. Wemwill compare the following output to these
    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
    std::cout << summary.BriefReport() << std::endl;

    Eigen::VectorXs origin_state_afterCeres(16);
    origin_state_afterCeres = origin_KF->getState();

    //===================================================== END{PROCESS DATA}
    
    for (int i = 7 ; i < 10 ; i++)
    {
        for (int j = 7 ; j < 10 ; j++)
        {
            for (int k = 7 ; k < 10 ; k++)
            {
                perturbated_state = initial_origin_state;
                perturbated_state(i) += 1.0;
                perturbated_state(j) += 2.0;
                perturbated_state(k) += 3.0;

                origin_KF->setState(perturbated_state);
                last_KF->setState(initial_final_state);

                //===================================================== SOLVER PART
     
                summary = ceres_manager_wolf_diff->solve();
                std::cout << summary.BriefReport() << std::endl;

                EXPECT_TRUE( (origin_state_afterCeres.segment(7,3) - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1, 0.001 ) ) <<
                "origin_state_afterCeres velocity state : " << origin_state_afterCeres.segment(7,3).transpose() << "\n origin velocity : " << origin_KF->getVPtr()->getVector().transpose() << std::endl;
            }
        }
    }

    // COMPUTE COVARIANCES
    //std::cout << "\t\t\t ______computing covariances______" << std::endl;
    //ceres_manager_wolf_diff->computeCovariances(ALL);//ALL_MARGINALS, ALL
    //std::cout << "\t\t\t ______computed!______" << std::endl;

    //===================================================== END{SOLVER PART}
}

/* Introduce a perturbation in Origin_KF
 * Ideally the optimization should be able to make origin_KF orientation converge to its correct value (value it would have taken if it had not been perturbated). 
 * 
 * The perturbated StateBlock is the only one unfixed in this test.
 */
TEST_F(ProcessorIMU_Odom_tests,Plateform_2s_move_PerturbateOrientationOrigin_UnfixPerturbatedOnly)
{

    using std::shared_ptr;
    using std::make_shared;
    using std::static_pointer_cast;
    
    std::string wolf_root = _WOLF_ROOT_DIR;
    char* imu_filepath;
    std::string filepath_string(wolf_root + "/src/test/data/IMU/Test_plateforme/imu_plateform_2s.txt");
    imu_filepath   = new char[filepath_string.length() + 1];
    std::strcpy(imu_filepath, filepath_string.c_str());
    std::ifstream imu_data_input;

    imu_data_input.open(imu_filepath);
    WOLF_INFO("imu file: ", imu_filepath)

    if(!imu_data_input.is_open()){
        std::cerr << "Failed to open data files... Exiting" << std::endl;
        ADD_FAILURE();
    }

    //===================================================== SETTING PROBLEM

    // reset origin of problem
    Eigen::VectorXs x_origin((Eigen::Matrix<wolf::Scalar,16,1>()<<0,0,0, 0,0,0,1, 0,0,0, 0,0,0, 0,0,0).finished());

    // initial conditions defined from data file
    // remember that matlab's quaternion is W,X,Y,Z and the one in Eigen has X,Y,Z,W form
    imu_data_input >> x_origin[0] >> x_origin[1] >> x_origin[2] >> x_origin[6] >> x_origin[3] >> x_origin[4] >> x_origin[5] >> x_origin[7] >> x_origin[8] >> x_origin[9];

    t.set(0);
    FrameBasePtr origin_KF = processor_ptr_imu->setOrigin(x_origin, t);
    processor_ptr_odom3D->setOrigin(origin_KF);
    FrameIMUPtr origin_imuKF = std::static_pointer_cast<FrameIMU>(origin_KF);
    
    //===================================================== END{SETTING PROBLEM}

    //===================================================== PROCESS DATA
    // PROCESS DATA

    Eigen::Vector6s data_imu, data_odom3D;
    data_imu << 0,0,-wolf::gravity()(2), 0,0,0;
    data_odom3D << 0,0.06,0, 0,0,0;

    Scalar input_clock;
    TimeStamp ts(0);
    wolf::CaptureIMUPtr imu_ptr = std::make_shared<CaptureIMU>(ts, sen_imu, data_imu);
    wolf::CaptureMotionPtr mot_ptr = std::make_shared<CaptureMotion>(t, sen_odom3D, data_odom3D);

    while( !imu_data_input.eof() )
    {
        // PROCESS IMU DATA
        // Time and data variables
        imu_data_input >> input_clock >> data_imu[0] >> data_imu[1] >> data_imu[2] >> data_imu[3] >> data_imu[4] >> data_imu[5]; //Ax, Ay, Az, Gx, Gy, Gz

        ts.set(input_clock);
        imu_ptr->setTimeStamp(ts);
        imu_ptr->setData(data_imu);

        // process data in capture
        imu_ptr->getTimeStamp();
        sen_imu->process(imu_ptr);
    }

    //IMU data have all been processed. Now we process the odom3D data
    // PROCESS ODOM 3D DATA
    mot_ptr->setTimeStamp(ts);
    mot_ptr->setData(data_odom3D);
    sen_odom3D->process(mot_ptr);

    //closing file
    imu_data_input.close();

    FrameIMUPtr last_KF = std::static_pointer_cast<FrameIMU>(wolf_problem_ptr_->getTrajectoryPtr()->closestKeyFrameToTimeStamp(ts));

    last_KF->getOPtr()->fix();
    Eigen::VectorXs initial_final_state(16);
    initial_final_state = last_KF->getState();

    Eigen::VectorXs initial_origin_state = origin_KF->getState();
    Eigen::VectorXs perturbated_state(initial_origin_state);
    Eigen::Map<Eigen::Quaternions> quat_map(perturbated_state.data() + 3);

    //unfix only StateBlock of interest
    last_KF->fix();
    origin_KF->fix();
    origin_KF->getOPtr()->unfix();

    // call solver to get values after optimization. Wemwill compare the following output to these
    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
    std::cout << summary.BriefReport() << std::endl;

    Eigen::VectorXs origin_state_afterCeres(16);
    origin_state_afterCeres = origin_KF->getState();

    //===================================================== END{PROCESS DATA}
    
    for (int i = 0 ; i < 3 ; i++)
    {
        for (int j = 0 ; j < 3 ; j++)
        {
            for (int k = 0 ; k < 3 ; k++)
            {
                Eigen::Vector3s orientation_perturbation((Eigen::Vector3s()<<0,0,0).finished());
                orientation_perturbation(0) += i*0.2;
                orientation_perturbation(1) += j*0.1;
                orientation_perturbation(2) += k*0.15;

                perturbated_state = initial_origin_state;
                quat_map = quat_map * v2q(orientation_perturbation);

                origin_KF->setState(perturbated_state);
                last_KF->setState(initial_final_state);

                //===================================================== SOLVER PART
     
                summary = ceres_manager_wolf_diff->solve();
                std::cout << summary.BriefReport() << std::endl;

                ASSERT_TRUE( (origin_state_afterCeres.segment(3,4) - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, 0.0001 )) <<
                "origin_state_afterCeres quaternion : " << origin_state_afterCeres.segment(3,4).transpose() << "\n origin quaternion state : " << origin_KF->getOPtr()->getVector().transpose() << std::endl;
            }
        }
    }

    // COMPUTE COVARIANCES
    //std::cout << "\t\t\t ______computing covariances______" << std::endl;
    //ceres_manager_wolf_diff->computeCovariances(ALL);//ALL_MARGINALS, ALL
    //std::cout << "\t\t\t ______computed!______" << std::endl;

    //===================================================== END{SOLVER PART}
}

TEST_F(ProcessorIMU_Odom_tests,Plateform_2s_move_PerturbateAccBiasOrigin_UnfixPerturbatedOnly)
{
    using std::shared_ptr;
    using std::make_shared;
    using std::static_pointer_cast;
    
    std::string wolf_root = _WOLF_ROOT_DIR;
    char* imu_filepath;
    std::string filepath_string(wolf_root + "/src/test/data/IMU/Test_plateforme/imu_plateform_2s.txt");
    imu_filepath   = new char[filepath_string.length() + 1];
    std::strcpy(imu_filepath, filepath_string.c_str());
    std::ifstream imu_data_input;

    imu_data_input.open(imu_filepath);
    WOLF_INFO("imu file: ", imu_filepath)

    if(!imu_data_input.is_open()){
        std::cerr << "Failed to open data files... Exiting" << std::endl;
        ADD_FAILURE();
    }

    //===================================================== SETTING PROBLEM

    // reset origin of problem
    Eigen::VectorXs x_origin((Eigen::Matrix<wolf::Scalar,16,1>()<<0,0,0, 0,0,0,1, 0,0,0, 0,0,0, 0,0,0).finished());

    // initial conditions defined from data file
    // remember that matlab's quaternion is W,X,Y,Z and the one in Eigen has X,Y,Z,W form
    imu_data_input >> x_origin[0] >> x_origin[1] >> x_origin[2] >> x_origin[6] >> x_origin[3] >> x_origin[4] >> x_origin[5] >> x_origin[7] >> x_origin[8] >> x_origin[9];

    t.set(0);
    FrameBasePtr origin_KF = processor_ptr_imu->setOrigin(x_origin, t);
    processor_ptr_odom3D->setOrigin(origin_KF);
    FrameIMUPtr origin_imuKF = std::static_pointer_cast<FrameIMU>(origin_KF);
    
    //===================================================== END{SETTING PROBLEM}

    //===================================================== PROCESS DATA
    // PROCESS DATA

    Eigen::Vector6s data_imu, data_odom3D;
    data_imu << 0,0,-wolf::gravity()(2), 0,0,0;
    data_odom3D << 0,0.06,0, 0,0,0;

    Scalar input_clock;
    TimeStamp ts(0);
    wolf::CaptureIMUPtr imu_ptr = std::make_shared<CaptureIMU>(ts, sen_imu, data_imu);
    wolf::CaptureMotionPtr mot_ptr = std::make_shared<CaptureMotion>(t, sen_odom3D, data_odom3D);

    while( !imu_data_input.eof() )
    {
        // PROCESS IMU DATA
        // Time and data variables
        imu_data_input >> input_clock >> data_imu[0] >> data_imu[1] >> data_imu[2] >> data_imu[3] >> data_imu[4] >> data_imu[5]; //Ax, Ay, Az, Gx, Gy, Gz

        ts.set(input_clock);
        imu_ptr->setTimeStamp(ts);
        imu_ptr->setData(data_imu);

        // process data in capture
        imu_ptr->getTimeStamp();
        sen_imu->process(imu_ptr);
    }

    //IMU data have all been processed. Now we process the odom3D data
    // PROCESS ODOM 3D DATA
    mot_ptr->setTimeStamp(ts);
    mot_ptr->setData(data_odom3D);
    sen_odom3D->process(mot_ptr);

    //closing file
    imu_data_input.close();

    FrameIMUPtr last_KF = std::static_pointer_cast<FrameIMU>(wolf_problem_ptr_->getTrajectoryPtr()->closestKeyFrameToTimeStamp(ts));

    Eigen::VectorXs initial_final_state(16);
    initial_final_state = last_KF->getState();
    Eigen::VectorXs initial_origin_state = origin_KF->getState();
    Eigen::VectorXs perturbated_state(16);

    //Unfix only StateBlock of interest
    last_KF->fix();
    origin_KF->fix();
    origin_imuKF->getAccBiasPtr()->unfix();

    // call solver to get values after optimization. Wemwill compare the following output to these
    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
    std::cout << summary.BriefReport() << std::endl;

    Eigen::VectorXs origin_state_afterCeres(16);
    origin_state_afterCeres = origin_KF->getState();

    //===================================================== END{PROCESS DATA}
    
    for (int i = 10 ; i < 13 ; i++)
    {
        for (int j = 10 ; j < 13 ; j++)
        {
            for (int k = 10 ; k < 13 ; k++)
            {
                perturbated_state = initial_origin_state;
                perturbated_state(i) += 1.0;
                perturbated_state(j) += 2.0;
                perturbated_state(k) += 3.0;

                origin_KF->setState(perturbated_state);
                last_KF->setState(initial_final_state);

                //===================================================== SOLVER PART
     
                summary = ceres_manager_wolf_diff->solve();
                std::cout << summary.BriefReport() << std::endl;

                EXPECT_TRUE( (origin_state_afterCeres.segment(10,3) - origin_imuKF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1,0.001 ) ) <<
                "origin_state_afterCeres acc_bias state : " << origin_state_afterCeres.segment(10,3).transpose() << "\n origin acc_bias : " << origin_imuKF->getAccBiasPtr()->getVector().transpose() << std::endl;
            }
        }
    }
}

TEST_F(ProcessorIMU_Odom_tests,Plateform_2s_move_PerturbateGyroBiasOrigin_UnfixPerturbatedOnly)
{
    using std::shared_ptr;
    using std::make_shared;
    using std::static_pointer_cast;
    
    std::string wolf_root = _WOLF_ROOT_DIR;
    char* imu_filepath;
    std::string filepath_string(wolf_root + "/src/test/data/IMU/Test_plateforme/imu_plateform_2s.txt");
    imu_filepath   = new char[filepath_string.length() + 1];
    std::strcpy(imu_filepath, filepath_string.c_str());
    std::ifstream imu_data_input;

    imu_data_input.open(imu_filepath);
    WOLF_INFO("imu file: ", imu_filepath)

    if(!imu_data_input.is_open()){
        std::cerr << "Failed to open data files... Exiting" << std::endl;
        ADD_FAILURE();
    }

    //===================================================== SETTING PROBLEM

    // reset origin of problem
    Eigen::VectorXs x_origin((Eigen::Matrix<wolf::Scalar,16,1>()<<0,0,0, 0,0,0,1, 0,0,0, 0,0,0, 0,0,0).finished());

    // initial conditions defined from data file
    // remember that matlab's quaternion is W,X,Y,Z and the one in Eigen has X,Y,Z,W form
    imu_data_input >> x_origin[0] >> x_origin[1] >> x_origin[2] >> x_origin[6] >> x_origin[3] >> x_origin[4] >> x_origin[5] >> x_origin[7] >> x_origin[8] >> x_origin[9];

    t.set(0);
    FrameBasePtr origin_KF = processor_ptr_imu->setOrigin(x_origin, t);
    processor_ptr_odom3D->setOrigin(origin_KF);
    FrameIMUPtr origin_imuKF = std::static_pointer_cast<FrameIMU>(origin_KF);
    
    //===================================================== END{SETTING PROBLEM}

    //===================================================== PROCESS DATA
    // PROCESS DATA

    Eigen::Vector6s data_imu, data_odom3D;
    data_imu << 0,0,-wolf::gravity()(2), 0,0,0;
    data_odom3D << 0,0.06,0, 0,0,0;

    Scalar input_clock;
    TimeStamp ts(0);
    wolf::CaptureIMUPtr imu_ptr = std::make_shared<CaptureIMU>(ts, sen_imu, data_imu);
    wolf::CaptureMotionPtr mot_ptr = std::make_shared<CaptureMotion>(t, sen_odom3D, data_odom3D);

    while( !imu_data_input.eof() )
    {
        // PROCESS IMU DATA
        // Time and data variables
        imu_data_input >> input_clock >> data_imu[0] >> data_imu[1] >> data_imu[2] >> data_imu[3] >> data_imu[4] >> data_imu[5]; //Ax, Ay, Az, Gx, Gy, Gz

        ts.set(input_clock);
        imu_ptr->setTimeStamp(ts);
        imu_ptr->setData(data_imu);

        // process data in capture
        imu_ptr->getTimeStamp();
        sen_imu->process(imu_ptr);
    }

    //IMU data have all been processed. Now we process the odom3D data
    // PROCESS ODOM 3D DATA
    mot_ptr->setTimeStamp(ts);
    mot_ptr->setData(data_odom3D);
    sen_odom3D->process(mot_ptr);

    //closing file
    imu_data_input.close();

    FrameIMUPtr last_KF = std::static_pointer_cast<FrameIMU>(wolf_problem_ptr_->getTrajectoryPtr()->closestKeyFrameToTimeStamp(ts));

    Eigen::VectorXs initial_final_state(16);
    initial_final_state = last_KF->getState();
    Eigen::VectorXs initial_origin_state = origin_KF->getState();
    Eigen::VectorXs perturbated_state(16);

    //Unfix only StateBlock of interest
    last_KF->fix();
    origin_KF->fix();
    origin_imuKF->getGyroBiasPtr()->unfix();

    // call solver to get values after optimization. Wemwill compare the following output to these
    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
    std::cout << summary.BriefReport() << std::endl;

    Eigen::VectorXs origin_state_afterCeres(16);
    origin_state_afterCeres = origin_KF->getState();

    //===================================================== END{PROCESS DATA}
    
    for (int i = 13 ; i < 16 ; i++)
    {
        for (int j = 13 ; j < 16 ; j++)
        {
            for (int k = 13 ; k < 16 ; k++)
            {
                perturbated_state = initial_origin_state;
                perturbated_state(i) += 0.2;
                perturbated_state(j) += 0.15;
                perturbated_state(k) += 0.3;

                origin_KF->setState(perturbated_state);
                last_KF->setState(initial_final_state);

                //===================================================== SOLVER PART
     
                summary = ceres_manager_wolf_diff->solve();
                std::cout << summary.BriefReport() << std::endl;

                EXPECT_TRUE( (origin_state_afterCeres.segment(13,3) - origin_imuKF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, 0.00001 ) ) <<
                "origin_state_afterCeres gyro_bias state : " << origin_state_afterCeres.segment(13,3).transpose() << "\n origin gyro_bias : " << origin_imuKF->getGyroBiasPtr()->getVector().transpose() << std::endl;
            }
        }
    }
}

TEST_F(ProcessorIMU_Odom_tests,Plateform_10s_move_fixOriginPQV)
{
    /* 
     * Trajectory
     * KF1  <-- c1 	c2 	
     * Estim, ts=0,	 x = ( 0          0          0          0          0          0          1          0          0          0          0.35       -0.023     0.52       0.11       0.13       -0.048    )
     * sb: Fix Fix Fix Est Est
     *
     * KF3  <-- c3 	c4 	
     * Estim, ts=5.0003,	 x = ( 1.3e-11      0.06         -1.3e-12     8.4e-11      5e-10        2.2e-11      1            0.3          -0.73        2.7          -1.4         1.5          0.85         0.056        0.065        -0.036      )
     * sb: Est Est Est Est Est
     * 
     * KF4  <-- 
     * Estim, ts=10,	 x = ( 7.5e-12      0.06         6.7e-13      1.2e-10      1.5e-09      -4.8e-11     1            1.2          0.19         1.8          0            0            0            0            0            0           )
     * sb: Est Est Est Est Est
     *
     * Estimation of position and orientation are coherent. But velocity estimation does not fit with the real experiment. We would have no velocity in last_KF.
     * We also notice that bias change between the first 2 KF, separated in time by 5s, is important.
     * check if this is physically OK. 
     */

    using std::shared_ptr;
    using std::make_shared;
    using std::static_pointer_cast;
    
    std::string wolf_root = _WOLF_ROOT_DIR;
    char* imu_filepath;
    char* odom_filepath;
    std::string imu_filepath_string(wolf_root + "/src/test/data/IMU/Test_plateforme/imu_plateforme_10s.txt");
    std::string odom_filepath_string(wolf_root + "/src/test/data/IMU/Test_plateforme/odom_plateforme_10s.txt");
    imu_filepath   = new char[imu_filepath_string.length() + 1];
    odom_filepath   = new char[odom_filepath_string.length() + 1];
    std::strcpy(imu_filepath, imu_filepath_string.c_str());
    std::strcpy(odom_filepath, odom_filepath_string.c_str());
    std::ifstream imu_data_input;
    std::ifstream odom_data_input;

    imu_data_input.open(imu_filepath);
    odom_data_input.open(odom_filepath);
    WOLF_INFO("imu file: ", imu_filepath)
    if(!imu_data_input.is_open()){
        std::cerr << "Failed to open data files... Exiting" << std::endl;
        ADD_FAILURE();
    }

    //prepare creation of file if DEBUG_RESULTS activated
    #ifdef DEBUG_RESULTS
        std::ofstream debug_results;
        debug_results.open("debug_results.dat");
        if(debug_results)
            debug_results << "%%TimeStamp\t"
                        << "dp_x\t" << "dp_y\t" << "dp_z\t" << "dq_x\t" << "dq_y\t" << "dq_z\t" << "dq_w\t" << "dv_x\t" << "dv_y\t" << "dv_z\t"
                        << "Dp_x\t" << "Dp_y\t" << "Dp_z\t" << "Dq_x\t" << "Dq_y\t" << "Dq_z\t" << "Dq_w\t" << "Dv_x\t" << "Dv_y\t" << "Dv_z\t"
                        << "X_x\t" << "X_y\t" << "X_z\t" << "Xq_x\t" << "Xq_y\t" << "Xq_z\t" << "Xq_w\t" << "Xv_x\t" << "Xv_y\t" << "Xv_z\t" << std::endl;
    #endif


    //===================================================== SETTING PROBLEM

    // reset origin of problem
    Eigen::VectorXs x_origin((Eigen::Matrix<wolf::Scalar,16,1>()<<0,0,0, 0,0,0,1, 0,0,0, 0,0,0, 0,0,0).finished());

    // initial conditions defined from data file
    // remember that matlab's quaternion is W,X,Y,Z and the one in Eigen has X,Y,Z,W form
    imu_data_input >> x_origin[0] >> x_origin[1] >> x_origin[2] >> x_origin[6] >> x_origin[3] >> x_origin[4] >> x_origin[5] >> x_origin[7] >> x_origin[8] >> x_origin[9];

    t.set(0);
    FrameBasePtr origin_KF = processor_ptr_imu->setOrigin(x_origin, t);
    processor_ptr_odom3D->setOrigin(origin_KF);
    FrameIMUPtr origin_imuKF = std::static_pointer_cast<FrameIMU>(origin_KF);

    origin_imuKF->getPPtr()->fix();
    origin_imuKF->getOPtr()->fix();
    origin_imuKF->getVPtr()->fix();
    origin_imuKF->getAccBiasPtr()->unfix();
    origin_imuKF->getGyroBiasPtr()->unfix();
    
    //===================================================== END{SETTING PROBLEM}

    //===================================================== PROCESS DATA
    // PROCESS DATA

    Eigen::Vector6s data_imu, data_odom3D;
    data_imu << 0,0,-wolf::gravity()(2), 0,0,0;
    data_odom3D << 0,0.06,0, 0,0,0;

    Scalar input_clock;
    TimeStamp ts(0), ts_odom(0);
    wolf::CaptureIMUPtr imu_ptr = std::make_shared<CaptureIMU>(ts, sen_imu, data_imu);
    wolf::CaptureMotionPtr mot_ptr = std::make_shared<CaptureMotion>(t, sen_odom3D, data_odom3D);
    
    //when we find a IMU timestamp corresponding with this odometry timestamp then we process odometry measurement
    odom_data_input >> input_clock >> data_odom3D[0] >> data_odom3D[1] >> data_odom3D[2] >> data_odom3D[3] >> data_odom3D[4] >> data_odom3D[5]; //Ax, Ay, Az, Gx, Gy, Gz
    ts_odom.set(input_clock);

    while( !imu_data_input.eof() || !odom_data_input.eof() )
    {
        // PROCESS IMU DATA
        // Time and data variables
        imu_data_input >> input_clock >> data_imu[0] >> data_imu[1] >> data_imu[2] >> data_imu[3] >> data_imu[4] >> data_imu[5]; //Ax, Ay, Az, Gx, Gy, Gz

        ts.set(input_clock);
        imu_ptr->setTimeStamp(ts);
        imu_ptr->setData(data_imu);

        // process data in capture
        imu_ptr->getTimeStamp();
        sen_imu->process(imu_ptr);

        if(ts.get() == ts_odom.get())
        {
            // PROCESS ODOM 3D DATA
            mot_ptr->setTimeStamp(ts_odom);
            mot_ptr->setData(data_odom3D);
            sen_odom3D->process(mot_ptr);

            odom_data_input >> input_clock >> data_odom3D[0] >> data_odom3D[1] >> data_odom3D[2] >> data_odom3D[3] >> data_odom3D[4] >> data_odom3D[5]; //Ax, Ay, Az, Gx, Gy, Gz
            ts_odom.set(input_clock);
        }
    }

    //closing file
    imu_data_input.close();

    FrameIMUPtr last_KF = std::static_pointer_cast<FrameIMU>(wolf_problem_ptr_->getTrajectoryPtr()->closestKeyFrameToTimeStamp(ts));

    //===================================================== END{PROCESS DATA}

    //===================================================== SOLVER PART

    //Check and print wolf tree

    wolf_problem_ptr_->print(4,1,1,1);
     
    std::cout << "\t\t\t ______solving______" << std::endl;
    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
    std::cout << summary.FullReport() << std::endl;
    std::cout << "\t\t\t ______solved______" << std::endl;
    
    wolf_problem_ptr_->print(4,1,1,1);

    //test last against origin
    ASSERT_TRUE( (last_KF->getPPtr()->getVector() - data_odom3D.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS )) << 
    "last position state : " << last_KF->getPPtr()->getVector().transpose() << std::endl;
    ASSERT_TRUE( (last_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS ));
    //ASSERT_TRUE( (last_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1,  wolf::Constants::EPS ));
    ASSERT_TRUE( (last_KF->getOPtr()->getVector() - (Eigen::Vector4s()<<0,0,0,1).finished()).isMuchSmallerThan(1, wolf::Constants::EPS ));
    EXPECT_TRUE( (last_KF->getVPtr()->getVector() - (Eigen::Vector3s()<<0,0,0).finished()).isMuchSmallerThan(1, wolf::Constants::EPS ));

    // COMPUTE COVARIANCES
    std::cout << "\t\t\t ______computing covariances______" << std::endl;
    ceres_manager_wolf_diff->computeCovariances(ALL);//ALL_MARGINALS, ALL
    std::cout << "\t\t\t ______computed!______" << std::endl;

    //===================================================== END{SOLVER PART}
}

//___________________________________________
// PLATEFORM TESTS BIS - simulated data
//___________________________________________

TEST_F(ProcessorIMU_Odom_tests_plateform_simulation, No_Perturbation)
{
    origin_KF->fix();

    wolf_problem_ptr_->print(4,1,1,1);
     
    std::cout << "\t\t\t ______solving______" << std::endl;
    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
    std::cout << summary.FullReport() << std::endl;
    std::cout << "\t\t\t ______solved______" << std::endl;
    
    wolf_problem_ptr_->print(4,1,1,1);

    ASSERT_TRUE( (expected_final_state.head(3) - last_KF->getPPtr()->getVector()).isMuchSmallerThan(1,wolf::Constants::EPS*100 ) ) <<
    "expected_final_state position : " << expected_final_state.head(3).transpose() << "\n last_KF position : " << last_KF->getPPtr()->getVector().transpose() << std::endl;
    ASSERT_TRUE( (expected_final_state.segment(3,4) - last_KF->getOPtr()->getVector()).isMuchSmallerThan(1,wolf::Constants::EPS ) ) <<
    "expected_final_state quaternion : " << expected_final_state.segment(3,4).transpose() << "\n last_KF quaternion : " << last_KF->getOPtr()->getVector().transpose() << std::endl;
    ASSERT_TRUE( (expected_final_state.tail(3) - last_KF->getVPtr()->getVector()).isMuchSmallerThan(1,wolf::Constants::EPS*100) ) <<
    "expected_final_state velocity : " << expected_final_state.tail(3).transpose() << "\n last_KF velocity : " << last_KF->getVPtr()->getVector().transpose() << std::endl;
}

TEST_F(ProcessorIMU_Odom_tests_plateform_simulation, PerturbOriginPosition_UnfixPertubedOnly)
{
    origin_KF->fix();
    last_KF->unfix();

    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();

    // fix / unfix StateBlocks
    last_KF->fix();
    origin_KF->unfix();
    origin_KF->getPPtr()->unfix();
    origin_KF->getOPtr()->fix();
    origin_KF->getVPtr()->fix();
    origin_KF->getAccBiasPtr()->fix();
    origin_KF->getGyroBiasPtr()->fix();

    wolf_problem_ptr_->print(4,1,1,1);

    Eigen::VectorXs initial_origin_state = origin_KF->getState();
    Eigen::VectorXs perturbed_state(16);
    
    for (int i = 0 ; i < 3 ; i++)
    {
        for (int j = 0 ; j < 3 ; j++)
        {
            for (int k = 1 ; k < 3 ; k++)
            {
                perturbed_state = initial_origin_state;
                perturbed_state(i) += 0.05; //variation between 5 cm and 15 cm
                perturbed_state(j) += 0.05;
                perturbed_state(k) += 0.05;

                origin_KF->setState(perturbed_state);
                //last_KF is fixed. There is no need to reinitialize its state

                //===================================================== SOLVER PART
     
                ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
                std::cout << summary.BriefReport() << std::endl;

                EXPECT_TRUE( (initial_origin_state.head(3) - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )) << 
                "initial_origin_state position state : " << initial_origin_state.head(3).transpose() << "\n origin position state : " << origin_KF->getPPtr()->getVector().transpose() << std::endl;
            }
        }
    }

    wolf_problem_ptr_->print(4,1,1,1);
}

TEST_F(ProcessorIMU_Odom_tests_plateform_simulation, PerturbOriginVelocity_UnfixPertubedOnly)
{
    origin_KF->fix();
    last_KF->unfix();

    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
    last_KF->fix();

    // fix / unfix StateBlocks
    origin_KF->unfix();
    origin_KF->getPPtr()->fix();
    origin_KF->getOPtr()->fix();
    origin_KF->getVPtr()->unfix();
    origin_KF->getAccBiasPtr()->fix();
    origin_KF->getGyroBiasPtr()->fix();

    wolf_problem_ptr_->print(4,1,1,1);

    Eigen::VectorXs initial_origin_state = origin_KF->getState();
    Eigen::VectorXs perturbed_state(16);
    
    for (int i = 7 ; i < 10 ; i++)
    {
        for (int j = 7 ; j < 10 ; j++)
        {
            for (int k = 7 ; k < 10 ; k++)
            {
                perturbed_state = initial_origin_state;
                perturbed_state(i) += 0.5;
                perturbed_state(j) += 0.5;
                perturbed_state(k) += 0.5;

                origin_KF->setState(perturbed_state);
                //last_KF is fixed. There is no need to reinitialize its state

                //===================================================== SOLVER PART
     
                ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
                std::cout << summary.BriefReport() << std::endl;

                EXPECT_TRUE( (initial_origin_state.segment(7,3) - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS*10 )) << 
                "initial_origin_state position state : " << initial_origin_state.segment(7,3).transpose() << "\n origin position state : " << origin_KF->getVPtr()->getVector().transpose() << std::endl;
            }
        }
    }
}

TEST_F(ProcessorIMU_Odom_tests_plateform_simulation, PerturbOriginOrientation_UnfixPertubedOnly)
{
    origin_KF->fix();
    last_KF->unfix();

    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();

    // fix / unfix StateBlocks
    last_KF->fix();
    origin_KF->unfix();
    origin_KF->getPPtr()->fix();
    origin_KF->getOPtr()->unfix();
    origin_KF->getVPtr()->fix();
    origin_KF->getAccBiasPtr()->fix();
    origin_KF->getGyroBiasPtr()->fix();

    wolf_problem_ptr_->print(4,1,1,1);

    Eigen::VectorXs initial_origin_state = origin_KF->getState();
    Eigen::VectorXs perturbed_state(16);
    Eigen::Map<Eigen::Quaternions> quat_map(perturbed_state.data() + 3);
    
    for (int i = 7 ; i < 10 ; i++)
    {
        for (int j = 7 ; j < 10 ; j++)
        {
            for (int k = 7 ; k < 10 ; k++)
            {
                Eigen::Vector3s orientation_perturbation((Eigen::Vector3s()<<0,0,0).finished());
                orientation_perturbation(0) += i*0.2;
                orientation_perturbation(1) += j*0.1;
                orientation_perturbation(2) += k*0.15;

                perturbed_state = initial_origin_state;
                quat_map = quat_map * v2q(orientation_perturbation);

                origin_KF->setState(perturbed_state);

                //===================================================== SOLVER PART
     
                ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
                std::cout << summary.BriefReport() << std::endl;

                ASSERT_TRUE( (initial_origin_state.segment(3,4) - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )) <<
                "initial_origin_state quaternion : " << initial_origin_state.segment(3,4).transpose() << "\n origin quaternion state : " << origin_KF->getOPtr()->getVector().transpose() << std::endl;
            }
        }
    }
}

TEST_F(ProcessorIMU_Odom_tests_plateform_simulation, PerturbOriginAccBias_UnfixPertubedOnly)
{
    origin_KF->fix();
    last_KF->unfix();

    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();

    // fix / unfix StateBlocks
    last_KF->fix();
    origin_KF->unfix();
    origin_KF->getPPtr()->fix();
    origin_KF->getOPtr()->fix();
    origin_KF->getVPtr()->fix();
    origin_KF->getAccBiasPtr()->unfix();
    origin_KF->getGyroBiasPtr()->fix();

    wolf_problem_ptr_->print(4,1,1,1);

    Eigen::VectorXs initial_origin_state = origin_KF->getState();
    Eigen::VectorXs perturbed_state(16);
    
    for (int i = 10 ; i < 13 ; i++)
    {
        for (int j = 10 ; j < 13 ; j++)
        {
            for (int k = 10 ; k < 13 ; k++)
            {
                perturbed_state = initial_origin_state;
                perturbed_state(i) += 0.5;
                perturbed_state(j) += 0.5;
                perturbed_state(k) += 0.5;

                origin_KF->setState(perturbed_state);
                //last_KF is fixed. There is no need to reinitialize its state
                //std::cout << "pertubed state's Acc bias : " << perturbed_state.segment(10,3).transpose() << std::endl;

                //===================================================== SOLVER PART
     
                ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
                std::cout << summary.BriefReport() << std::endl;

                EXPECT_TRUE( (initial_origin_state.segment(10,3) - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS*10 )) << 
                "initial_origin_state position state : " << initial_origin_state.segment(10,3).transpose() << "\n origin position state : " << origin_KF->getAccBiasPtr()->getVector().transpose() << std::endl;
            }
        }
    }
}

TEST_F(ProcessorIMU_Odom_tests_plateform_simulation, PerturbOriginGyroBias_UnfixPertubedOnly)
{
    origin_KF->fix();
    last_KF->unfix();

    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();

    // fix / unfix StateBlocks
    last_KF->fix();
    origin_KF->unfix();
    origin_KF->getPPtr()->fix();
    origin_KF->getOPtr()->fix();
    origin_KF->getVPtr()->fix();
    origin_KF->getAccBiasPtr()->fix();
    origin_KF->getGyroBiasPtr()->unfix();

    wolf_problem_ptr_->print(4,1,1,1);

    Eigen::VectorXs initial_origin_state = origin_KF->getState();
    Eigen::VectorXs perturbed_state(16);
    
    for (int i = 13 ; i < 16 ; i++)
    {
        for (int j = 13 ; j < 16 ; j++)
        {
            for (int k = 13 ; k < 16 ; k++)
            {
                perturbed_state = initial_origin_state;
                perturbed_state(i) += 0.5;
                perturbed_state(j) += 0.5;
                perturbed_state(k) += 0.5;

                origin_KF->setState(perturbed_state);
                //last_KF is fixed. There is no need to reinitialize its state
                //std::cout << "pertubed state's Acc bias : " << perturbed_state.segment(10,3).transpose() << std::endl;

                //===================================================== SOLVER PART
     
                ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
                std::cout << summary.BriefReport() << std::endl;

                EXPECT_TRUE( (initial_origin_state.tail(3) - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS*10 )) << 
                "initial_origin_state position state : " << initial_origin_state.tail(3).transpose() << "\n origin position state : " << origin_KF->getGyroBiasPtr()->getVector().transpose() << std::endl;
            }
        }
    }
}

TEST_F(ProcessorIMU_Odom_tests_plateform_simulation, PerturbOriginPosition_UnfixOrigin)
{
    origin_KF->fix();
    last_KF->unfix();

    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();

    // fix / unfix StateBlocks
    last_KF->fix();
    origin_KF->unfix();

    wolf_problem_ptr_->print(4,1,1,1);

    Eigen::VectorXs initial_origin_state = origin_KF->getState();
    Eigen::VectorXs perturbed_state(16);
    
    for (int i = 0 ; i < 3 ; i++)
    {
        for (int j = 0 ; j < 3 ; j++)
        {
            for (int k = 1 ; k < 3 ; k++)
            {
                perturbed_state = initial_origin_state;
                perturbed_state(i) += 0.5; //variation between 5 cm and 15 cm
                perturbed_state(j) += 0.5;
                perturbed_state(k) += 0.5;

                origin_KF->setState(perturbed_state);
                //last_KF is fixed. There is no need to reinitialize its state

                //===================================================== SOLVER PART
     
                ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
                std::cout << summary.BriefReport() << std::endl;

                EXPECT_TRUE( (initial_origin_state.head(3) - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << 
                "initial_origin_state position state : " << initial_origin_state.head(3).transpose() << "\n origin position state : " << origin_KF->getPPtr()->getVector().transpose() << std::endl;
                EXPECT_TRUE( (initial_origin_state.segment(3,4) - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS*10 )) <<
                "initial_origin_state quaternion : " << initial_origin_state.segment(3,4).transpose() << "\n origin quaternion state : " << origin_KF->getOPtr()->getVector().transpose() << std::endl;
                EXPECT_TRUE( (initial_origin_state.segment(7,3) - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << 
                "initial_origin_state velocity state : " << initial_origin_state.segment(7,3).transpose() << "\n origin velocity state : " << origin_KF->getVPtr()->getVector().transpose() << std::endl;
                EXPECT_TRUE( (initial_origin_state.segment(10,3) - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << 
                "initial_origin_state acc bias state : " << initial_origin_state.segment(10,3).transpose() << "\n origin acc bias state : " << origin_KF->getAccBiasPtr()->getVector().transpose() << std::endl;
                EXPECT_TRUE( (initial_origin_state.tail(3) - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )) << 
                "initial_origin_state gyro bias state : " << initial_origin_state.tail(3).transpose() << "\n origin gyro bias state : " << origin_KF->getGyroBiasPtr()->getVector().transpose() << std::endl;
            }
        }
    }

    //wolf_problem_ptr_->print(4,1,1,1);
}

TEST_F(ProcessorIMU_Odom_tests_plateform_simulation, PerturbOriginVelocity_UnfixOrigin)
{
    origin_KF->fix();
    last_KF->unfix();

    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();

    // fix / unfix StateBlocks
    last_KF->fix();
    origin_KF->unfix();

    wolf_problem_ptr_->print(4,1,1,1);

    Eigen::VectorXs initial_origin_state = origin_KF->getState();
    Eigen::VectorXs perturbed_state(16);
    
    for (int i = 7 ; i < 10 ; i++)
    {
        for (int j = 7 ; j < 10 ; j++)
        {
            for (int k = 7 ; k < 10 ; k++)
            {
                perturbed_state = initial_origin_state;
                perturbed_state(i) += 0.5; //variation between 5 cm and 15 cm
                perturbed_state(j) += 0.5;
                perturbed_state(k) += 0.5;

                origin_KF->setState(perturbed_state);
                //last_KF is fixed. There is no need to reinitialize its state

                //===================================================== SOLVER PART
     
                ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
                std::cout << summary.BriefReport() << std::endl;

                EXPECT_TRUE( (initial_origin_state.head(3) - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << 
                "initial_origin_state position state : " << initial_origin_state.head(3).transpose() << "\n origin position state : " << origin_KF->getPPtr()->getVector().transpose() << std::endl;
                EXPECT_TRUE( (initial_origin_state.segment(3,4) - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS*10 )) <<
                "initial_origin_state quaternion : " << initial_origin_state.segment(3,4).transpose() << "\n origin quaternion state : " << origin_KF->getOPtr()->getVector().transpose() << std::endl;
                EXPECT_TRUE( (initial_origin_state.segment(7,3) - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << 
                "initial_origin_state velocity state : " << initial_origin_state.segment(7,3).transpose() << "\n origin velocity state : " << origin_KF->getVPtr()->getVector().transpose() << std::endl;
                EXPECT_TRUE( (initial_origin_state.segment(10,3) - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << 
                "initial_origin_state acc bias state : " << initial_origin_state.segment(10,3).transpose() << "\n origin acc bias state : " << origin_KF->getAccBiasPtr()->getVector().transpose() << std::endl;
                EXPECT_TRUE( (initial_origin_state.tail(3) - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )) << 
                "initial_origin_state gyro bias state : " << initial_origin_state.tail(3).transpose() << "\n origin gyro bias state : " << origin_KF->getGyroBiasPtr()->getVector().transpose() << std::endl;
            }
        }
    }

    //wolf_problem_ptr_->print(4,1,1,1);
}

TEST_F(ProcessorIMU_Odom_tests_plateform_simulation, PerturbOriginOrientation_UnfixOrigin)
{
    origin_KF->fix();
    last_KF->unfix();

    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();

    // fix / unfix StateBlocks
    last_KF->fix();
    origin_KF->unfix();

    wolf_problem_ptr_->print(4,1,1,1);

    Eigen::VectorXs initial_origin_state = origin_KF->getState();
    Eigen::VectorXs perturbed_state(16);
    Eigen::Map<Eigen::Quaternions> quat_map(perturbed_state.data() + 3);
    
    for (int i = 7 ; i < 10 ; i++)
    {
        for (int j = 7 ; j < 10 ; j++)
        {
            for (int k = 7 ; k < 10 ; k++)
            {
                Eigen::Vector3s orientation_perturbation((Eigen::Vector3s()<<0,0,0).finished());
                orientation_perturbation(0) += i*0.2;
                orientation_perturbation(1) += j*0.1;
                orientation_perturbation(2) += k*0.15;

                perturbed_state = initial_origin_state;
                quat_map = quat_map * v2q(orientation_perturbation);

                origin_KF->setState(perturbed_state);

                //===================================================== SOLVER PART
     
                ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
                std::cout << summary.BriefReport() << std::endl;

                EXPECT_TRUE( (initial_origin_state.head(3) - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << 
                "initial_origin_state position state : " << initial_origin_state.head(3).transpose() << "\n origin position state : " << origin_KF->getPPtr()->getVector().transpose() << std::endl;
                EXPECT_TRUE( (initial_origin_state.segment(3,4) - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS*10 )) <<
                "initial_origin_state quaternion : " << initial_origin_state.segment(3,4).transpose() << "\n origin quaternion state : " << origin_KF->getOPtr()->getVector().transpose() << std::endl;
                EXPECT_TRUE( (initial_origin_state.segment(7,3) - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << 
                "initial_origin_state velocity state : " << initial_origin_state.segment(7,3).transpose() << "\n origin velocity state : " << origin_KF->getVPtr()->getVector().transpose() << std::endl;
                EXPECT_TRUE( (initial_origin_state.segment(10,3) - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << 
                "initial_origin_state acc bias state : " << initial_origin_state.segment(10,3).transpose() << "\n origin acc bias state : " << origin_KF->getAccBiasPtr()->getVector().transpose() << std::endl;
                EXPECT_TRUE( (initial_origin_state.tail(3) - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )) << 
                "initial_origin_state gyro bias state : " << initial_origin_state.tail(3).transpose() << "\n origin gyro bias state : " << origin_KF->getGyroBiasPtr()->getVector().transpose() << std::endl;
            }
        }
    }
}

TEST_F(ProcessorIMU_Odom_tests_plateform_simulation, PerturbOriginAccBias_UnfixOrigin)
{
    origin_KF->fix();
    last_KF->unfix();

    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();

    // fix / unfix StateBlocks
    last_KF->fix();
    origin_KF->unfix();

    wolf_problem_ptr_->print(4,1,1,1);

    Eigen::VectorXs initial_origin_state = origin_KF->getState();
    Eigen::VectorXs perturbed_state(16);
    
    for (int i = 10 ; i < 13 ; i++)
    {
        for (int j = 10 ; j < 13 ; j++)
        {
            for (int k = 10 ; k < 13 ; k++)
            {
                perturbed_state = initial_origin_state;
                perturbed_state(i) += 0.5;
                perturbed_state(j) += 0.5;
                perturbed_state(k) += 0.5;

                origin_KF->setState(perturbed_state);
                //last_KF is fixed. There is no need to reinitialize its state
                //std::cout << "pertubed state's Acc bias : " << perturbed_state.segment(10,3).transpose() << std::endl;

                //===================================================== SOLVER PART
     
                ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
                std::cout << summary.BriefReport() << std::endl;

                EXPECT_TRUE( (initial_origin_state.head(3) - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << 
                "initial_origin_state position state : " << initial_origin_state.head(3).transpose() << "\n origin position state : " << origin_KF->getPPtr()->getVector().transpose() << std::endl;
                EXPECT_TRUE( (initial_origin_state.segment(3,4) - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS*10 )) <<
                "initial_origin_state quaternion : " << initial_origin_state.segment(3,4).transpose() << "\n origin quaternion state : " << origin_KF->getOPtr()->getVector().transpose() << std::endl;
                EXPECT_TRUE( (initial_origin_state.segment(7,3) - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << 
                "initial_origin_state velocity state : " << initial_origin_state.segment(7,3).transpose() << "\n origin velocity state : " << origin_KF->getVPtr()->getVector().transpose() << std::endl;
                EXPECT_TRUE( (initial_origin_state.segment(10,3) - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << 
                "initial_origin_state acc bias state : " << initial_origin_state.segment(10,3).transpose() << "\n origin acc bias state : " << origin_KF->getAccBiasPtr()->getVector().transpose() << std::endl;
                EXPECT_TRUE( (initial_origin_state.tail(3) - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )) << 
                "initial_origin_state gyro bias state : " << initial_origin_state.tail(3).transpose() << "\n origin gyro bias state : " << origin_KF->getGyroBiasPtr()->getVector().transpose() << std::endl;
            }
        }
    }
}

TEST_F(ProcessorIMU_Odom_tests_plateform_simulation, PerturbOriginGyroBias_UnfixOrigin)
{
    origin_KF->fix();
    last_KF->unfix();

    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();

    // fix / unfix StateBlocks
    last_KF->fix();
    origin_KF->unfix();

    wolf_problem_ptr_->print(4,1,1,1);

    Eigen::VectorXs initial_origin_state = origin_KF->getState();
    Eigen::VectorXs perturbed_state(16);
    
    for (int i = 13 ; i < 16 ; i++)
    {
        for (int j = 13 ; j < 16 ; j++)
        {
            for (int k = 13 ; k < 16 ; k++)
            {
                perturbed_state = initial_origin_state;
                perturbed_state(i) += 0.5;
                perturbed_state(j) += 0.5;
                perturbed_state(k) += 0.5;

                origin_KF->setState(perturbed_state);
                //last_KF is fixed. There is no need to reinitialize its state
                //std::cout << "pertubed state's Acc bias : " << perturbed_state.segment(10,3).transpose() << std::endl;

                //===================================================== SOLVER PART
     
                ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
                std::cout << summary.BriefReport() << std::endl;

                EXPECT_TRUE( (initial_origin_state.head(3) - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << 
                "initial_origin_state position state : " << initial_origin_state.head(3).transpose() << "\n origin position state : " << origin_KF->getPPtr()->getVector().transpose() << std::endl;
                EXPECT_TRUE( (initial_origin_state.segment(3,4) - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS*10 )) <<
                "initial_origin_state quaternion : " << initial_origin_state.segment(3,4).transpose() << "\n origin quaternion state : " << origin_KF->getOPtr()->getVector().transpose() << std::endl;
                EXPECT_TRUE( (initial_origin_state.segment(7,3) - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << 
                "initial_origin_state velocity state : " << initial_origin_state.segment(7,3).transpose() << "\n origin velocity state : " << origin_KF->getVPtr()->getVector().transpose() << std::endl;
                EXPECT_TRUE( (initial_origin_state.segment(10,3) - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << 
                "initial_origin_state acc bias state : " << initial_origin_state.segment(10,3).transpose() << "\n origin acc bias state : " << origin_KF->getAccBiasPtr()->getVector().transpose() << std::endl;
                EXPECT_TRUE( (initial_origin_state.tail(3) - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )) << 
                "initial_origin_state gyro bias state : " << initial_origin_state.tail(3).transpose() << "\n origin gyro bias state : " << origin_KF->getGyroBiasPtr()->getVector().transpose() << std::endl;
            }
        }
    }
}

TEST_F(ProcessorIMU_Odom_tests_plateform_simulation, PerturbOriginAll_UnfixOrigin)
{
    origin_KF->fix();
    last_KF->unfix();

    wolf_problem_ptr_->print(4,1,1,1);

    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();

    // fix / unfix StateBlocks
    last_KF->fix();
    origin_KF->unfix();

    wolf_problem_ptr_->print(4,1,1,1);

    Eigen::VectorXs initial_origin_state = origin_KF->getState();
    Eigen::VectorXs perturbed_state(16);
    Eigen::Map<Eigen::Quaternions> quat_map(perturbed_state.data() + 3);
    
    perturbed_state = initial_origin_state;
    perturbed_state(0) += 1.5;
    perturbed_state(1) += 1.0;
    perturbed_state(2) += 1.25;

    Eigen::Vector3s orientation_perturbation((Eigen::Vector3s()<<0,0,0).finished());
    orientation_perturbation(0) += 0.6;
    orientation_perturbation(1) += 0.5;
    orientation_perturbation(2) += -0.15;
    quat_map = quat_map * v2q(orientation_perturbation);

    perturbed_state(7) += 1.5;
    perturbed_state(8) += 0.7;
    perturbed_state(9) += 1.15;

    perturbed_state(10) += 0.75;
    perturbed_state(11) += 1.0;
    perturbed_state(12) += 0.6;
    perturbed_state(13) += 0.55;
    perturbed_state(14) += 0.42;
    perturbed_state(15) += 0.035;

    origin_KF->setState(perturbed_state);
    //last_KF is fixed. There is no need to reinitialize its state

    //===================================================== SOLVER PART
    summary = ceres_manager_wolf_diff->solve();
    std::cout << summary.BriefReport() << std::endl;

    EXPECT_TRUE( (initial_origin_state.head(3) - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << 
    "initial_origin_state position state : " << initial_origin_state.head(3).transpose() << "\n origin position state : " << origin_KF->getPPtr()->getVector().transpose() << std::endl;
    EXPECT_TRUE( (initial_origin_state.segment(3,4) - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS*10 )) <<
    "initial_origin_state quaternion : " << initial_origin_state.segment(3,4).transpose() << "\n origin quaternion state : " << origin_KF->getOPtr()->getVector().transpose() << std::endl;
    EXPECT_TRUE( (initial_origin_state.segment(7,3) - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << 
    "initial_origin_state velocity state : " << initial_origin_state.segment(7,3).transpose() << "\n origin velocity state : " << origin_KF->getVPtr()->getVector().transpose() << std::endl;
    EXPECT_TRUE( (initial_origin_state.segment(10,3) - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << 
    "initial_origin_state acc bias state : " << initial_origin_state.segment(10,3).transpose() << "\n origin acc bias state : " << origin_KF->getAccBiasPtr()->getVector().transpose() << std::endl;
    EXPECT_TRUE( (initial_origin_state.tail(3) - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )) << 
    "initial_origin_state gyro bias state : " << initial_origin_state.tail(3).transpose() << "\n origin gyro bias state : " << origin_KF->getGyroBiasPtr()->getVector().transpose() << std::endl;

    //wolf_problem_ptr_->print(4,1,1,1);
}

TEST_F(ProcessorIMU_Odom_tests_plateform_simulation, FixOriginPQV_UnfixLast)
{
    origin_KF->unfix();
    origin_KF->getPPtr()->fix();
    origin_KF->getOPtr()->fix();
    origin_KF->getVPtr()->fix();
    last_KF->unfix();

    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();

    ASSERT_TRUE( (expected_final_state.head(3) - last_KF->getPPtr()->getVector()).isMuchSmallerThan(1,wolf::Constants::EPS*100 ) ) <<
    "expected_final_state position : " << expected_final_state.head(3).transpose() << "\n last_KF position : " << last_KF->getPPtr()->getVector().transpose() << std::endl;
    ASSERT_TRUE( (expected_final_state.segment(3,4) - last_KF->getOPtr()->getVector()).isMuchSmallerThan(1,wolf::Constants::EPS*100 ) ) <<
    "expected_final_state quaternion : " << expected_final_state.segment(3,4).transpose() << "\n last_KF quaternion : " << last_KF->getOPtr()->getVector().transpose() << std::endl;
    ASSERT_TRUE( (expected_final_state.tail(3) - last_KF->getVPtr()->getVector()).isMuchSmallerThan(1,wolf::Constants::EPS*1000) ) <<
    "expected_final_state velocity : " << expected_final_state.tail(3).transpose() << "\n last_KF velocity : " << last_KF->getVPtr()->getVector().transpose() << std::endl;
    //wolf_problem_ptr_->print(4,1,1,1);
}

TEST_F(ProcessorIMU_Odom_tests_plateform_simulation, FixOriginPQV_UnfixLast_fixlastP)
{
    origin_KF->unfix();
    origin_KF->getPPtr()->fix();
    origin_KF->getOPtr()->fix();
    origin_KF->getVPtr()->fix();
    last_KF->unfix();
    last_KF->getPPtr()->fix();

    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();

    //ASSERT_TRUE( (expected_final_state.head(3) - last_KF->getPPtr()->getVector()).isMuchSmallerThan(1,wolf::Constants::EPS*100 ) ) <<
    //"expected_final_state position : " << expected_final_state.head(3).transpose() << "\n last_KF position : " << last_KF->getPPtr()->getVector().transpose() << std::endl;
    ASSERT_TRUE( (expected_final_state.segment(3,4) - last_KF->getOPtr()->getVector()).isMuchSmallerThan(1,wolf::Constants::EPS*100 ) ) <<
    "expected_final_state quaternion : " << expected_final_state.segment(3,4).transpose() << "\n last_KF quaternion : " << last_KF->getOPtr()->getVector().transpose() << std::endl;
    ASSERT_TRUE( (expected_final_state.tail(3) - last_KF->getVPtr()->getVector()).isMuchSmallerThan(1,wolf::Constants::EPS*1000) ) <<
    "expected_final_state velocity : " << expected_final_state.tail(3).transpose() << "\n last_KF velocity : " << last_KF->getVPtr()->getVector().transpose() << std::endl;
    //wolf_problem_ptr_->print(4,1,1,1);
}

TEST_F(ProcessorIMU_Odom_tests_plateform_simulation, FixOriginPQV_UnfixLast_fixlastV)
{
    origin_KF->unfix();
    origin_KF->getPPtr()->fix();
    origin_KF->getOPtr()->fix();
    origin_KF->getVPtr()->fix();
    last_KF->unfix();
    last_KF->getPPtr()->fix();

    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();

    ASSERT_TRUE( (expected_final_state.head(3) - last_KF->getPPtr()->getVector()).isMuchSmallerThan(1,wolf::Constants::EPS*100 ) ) <<
    "expected_final_state position : " << expected_final_state.head(3).transpose() << "\n last_KF position : " << last_KF->getPPtr()->getVector().transpose() << std::endl;
    ASSERT_TRUE( (expected_final_state.segment(3,4) - last_KF->getOPtr()->getVector()).isMuchSmallerThan(1,wolf::Constants::EPS*100 ) ) <<
    "expected_final_state quaternion : " << expected_final_state.segment(3,4).transpose() << "\n last_KF quaternion : " << last_KF->getOPtr()->getVector().transpose() << std::endl;
    //ASSERT_TRUE( (expected_final_state.tail(3) - last_KF->getVPtr()->getVector()).isMuchSmallerThan(1,wolf::Constants::EPS*1000) ) <<
    //"expected_final_state velocity : " << expected_final_state.tail(3).transpose() << "\n last_KF velocity : " << last_KF->getVPtr()->getVector().transpose() << std::endl;
    //wolf_problem_ptr_->print(4,1,1,1);
}

TEST_F(ProcessorIMU_Odom_tests_plateform_simulation, PerturbLastAll_UnfixLast_fixOrigin)
{
    origin_KF->fix();
    last_KF->unfix();

    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();

    wolf_problem_ptr_->print(4,1,1,1);

    Eigen::VectorXs initial_final_state = last_KF->getState();
    Eigen::VectorXs perturbed_state(16);
    Eigen::Map<Eigen::Quaternions> quat_map(perturbed_state.data() + 3);
    
    perturbed_state = initial_final_state;
    perturbed_state(0) += 1.5;
    perturbed_state(1) += 1.0;
    perturbed_state(2) += 1.25;

    Eigen::Vector3s orientation_perturbation((Eigen::Vector3s()<<0,0,0).finished());
    orientation_perturbation(0) += 0.6;
    orientation_perturbation(1) += 0.5;
    orientation_perturbation(2) += -0.15;
    quat_map = quat_map * v2q(orientation_perturbation);

    perturbed_state(7) += 1.5;
    perturbed_state(8) += 0.7;
    perturbed_state(9) += 1.15;

    perturbed_state(10) += 0.75;
    perturbed_state(11) += 1.0;
    perturbed_state(12) += 0.6;
    perturbed_state(13) += 0.55;
    perturbed_state(14) += 0.42;
    perturbed_state(15) += 0.035;

    last_KF->setState(perturbed_state);

    //===================================================== SOLVER PART
    summary = ceres_manager_wolf_diff->solve();
    std::cout << summary.BriefReport() << std::endl;

    EXPECT_TRUE( (initial_final_state.head(3) - last_KF->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << 
    "initial_final_state position state : " << initial_final_state.head(3).transpose() << "\n last position state : " << last_KF->getPPtr()->getVector().transpose() << std::endl;
    EXPECT_TRUE( (initial_final_state.segment(3,4) - last_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS*10 )) <<
    "initial_final_state quaternion : " << initial_final_state.segment(3,4).transpose() << "\n last quaternion state : " << last_KF->getOPtr()->getVector().transpose() << std::endl;
    EXPECT_TRUE( (initial_final_state.segment(7,3) - last_KF->getVPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << 
    "initial_final_state velocity state : " << initial_final_state.segment(7,3).transpose() << "\n last velocity state : " << last_KF->getVPtr()->getVector().transpose() << std::endl;
    EXPECT_FALSE( (initial_final_state.segment(10,3) - last_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS*100 )) << 
    "initial_final_state acc bias state : " << initial_final_state.segment(10,3).transpose() << "\n last acc bias state : " << last_KF->getAccBiasPtr()->getVector().transpose() << std::endl;
    EXPECT_FALSE( (initial_final_state.tail(3) - last_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )) << 
    "initial_final_state gyro bias state : " << initial_final_state.tail(3).transpose() << "\n last gyro bias state : " << last_KF->getGyroBiasPtr()->getVector().transpose() << std::endl;


    //wolf_problem_ptr_->print(4,1,1,1);
}

//___________________________________________
// END{ PLATEFORM TESTS BIS - simulated data }
//___________________________________________

//_______________________________________________________________________________________________________________
// END ##################################### Plateform TESTS #####################################
//_______________________________________________________________________________________________________________

//_______________________________________________________________________________________________________________
// ##################################### Imperfect IMU TESTS #####################################
//_______________________________________________________________________________________________________________

TEST_F(ProcessorIMU_Odom_tests, IMU_Biased)
{

    /* In this scenario, we simulate the integration of a static IMU biased in X Acceleration and we add an odometry measurement.
     * Initial State is [0,0,0, 0,0,0,1, 0,0,0] so we expect the Final State to be exactly the same and the bias to be identified correctly.
     * Origin KeyFrame is fixed
     * 
     * Finally, we can represent the graph as :
     *
     *  KF0 ---- constraintIMU ---- KF1
     *     \____constraintOdom3D___/
     *
     * a first test to check that bias is correctly used in state reconstruction
     */

     //===================================================== END OF SETTINGS

     // set origin of processorMotions
     Vector6s imuBias((Eigen::Vector6s() << 0.01, 0.005, 0.015, 0.05, 0.014, 0.1).finished() );
    Eigen::VectorXs x_origin((Eigen::Matrix<wolf::Scalar,16,1>()<<0,0,0, 0,0,0,1, 0,0,0, imuBias(0),imuBias(1),imuBias(2), imuBias(3),imuBias(4),imuBias(5)).finished());
    t.set(0);

    FrameBasePtr setOrigin_KF = processor_ptr_imu->setOrigin(x_origin, t);
    processor_ptr_odom3D->setOrigin(setOrigin_KF);

    //===================================================== END{END OF SETTINGS}

    //===================================================== PROCESS DATA

    // PROCESS IMU DATA

    Eigen::Vector6s data;
    data << 0.0, 0.0, -wolf::gravity()(2), 0.0, 0.0, 0.0;
    data += imuBias; //Biased IMU
    Scalar dt = t.get();
    TimeStamp ts(0);
    wolf::CaptureIMUPtr imu_ptr = std::make_shared<CaptureIMU>(ts, sen_imu, data);

    while( (dt-t.get()) < (std::static_pointer_cast<ProcessorIMU>(processor_ptr_)->getMaxTimeSpan()) ){

        // Time and data variables
        dt += 0.001;
        ts.set(dt);
        imu_ptr->setTimeStamp(ts);
        imu_ptr->setData(data);

        // process data in capture
        imu_ptr->getTimeStamp();
        sen_imu->process(imu_ptr);
    }

    // PROCESS ODOM 3D DATA
    Eigen::Vector6s data_odom3D;
    data_odom3D << 0,0,0, 0,0,0;
    
    wolf::CaptureMotionPtr mot_ptr = std::make_shared<CaptureMotion>(ts, sen_odom3D, data_odom3D);
    sen_odom3D->process(mot_ptr);

    //===================================================== END{PROCESS DATA}

    //===================================================== SOLVER PART

    FrameIMUPtr origin_KF = std::static_pointer_cast<FrameIMU>(wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front());
    FrameIMUPtr last_KF = std::static_pointer_cast<FrameIMU>(wolf_problem_ptr_->getTrajectoryPtr()->closestKeyFrameToTimeStamp(ts));

    //wolf_problem_ptr_->print(4,1,1,1);
     
    std::cout << "\t\t\t ______solving______" << std::endl;
    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
    std::cout << summary.FullReport() << std::endl;
    std::cout << "\t\t\t ______solved______" << std::endl;

    //wolf_problem_ptr_->print(4,1,1,1);

    ASSERT_TRUE( (last_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )) << 
    "origin_KF position : " << origin_KF->getPPtr()->getVector().transpose() << "\n last_KF position : " << last_KF->getPPtr()->getVector().transpose() << std::endl;
    ASSERT_TRUE( (last_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )) << 
    "origin_KF orientation : " << origin_KF->getOPtr()->getVector().transpose() << "\n last_KF orientation : " << last_KF->getOPtr()->getVector().transpose() << std::endl;
    ASSERT_TRUE( (last_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )) << 
    "origin_KF velocity : " << origin_KF->getVPtr()->getVector().transpose() << "\n last_KF velocity : " << last_KF->getVPtr()->getVector().transpose() << std::endl;
    ASSERT_TRUE( (last_KF->getAccBiasPtr()->getVector() - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )); 
    ASSERT_TRUE( (last_KF->getAccBiasPtr()->getVector() - imuBias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS )); 
    ASSERT_TRUE( (last_KF->getGyroBiasPtr()->getVector() - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS ));
    ASSERT_TRUE( (last_KF->getGyroBiasPtr()->getVector() - imuBias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS ));

    // COMPUTE COVARIANCES
    std::cout << "\t\t\t ______computing covariances______" << std::endl;
    ceres_manager_wolf_diff->computeCovariances(ALL);//ALL_MARGINALS, ALL
    std::cout << "\t\t\t ______computed!______" << std::endl;

    //===================================================== END{SOLVER PART}
}

TEST_F(ProcessorIMU_Odom_tests, IMU_Biased_perturbData)
{

    /* In this scenario, we simulate the integration of a static IMU biased in X Acceleration and we add an odometry measurement.
     * Initial State is [0,0,0, 0,0,0,1, 0,0,0] so we expect the Final State to be exactly the same and the bias to be identified correctly.
     * Origin KeyFrame is fixed
     * 
     * Finally, we can represent the graph as :
     *
     *  KF0 ---- constraintIMU ---- KF1
     *     \____constraintOdom3D___/
     *
     * Add a small bias perturbation in imu data before it is processed by the capture.
     * Expect this small increment to be corrected in the bias StateBlock of the generated KeyFrame.
     */

     //===================================================== END OF SETTINGS

     // set origin of processorMotions
     Vector6s imuBias((Eigen::Vector6s() << 0.01, 0.005, 0.015, 0.05, 0.014, 0.1).finished() );
    Eigen::VectorXs x_origin((Eigen::Matrix<wolf::Scalar,16,1>()<<0,0,0, 0,0,0,1, 0,0,0, imuBias(0),imuBias(1),imuBias(2), imuBias(3),imuBias(4),imuBias(5)).finished());
    t.set(0);

    FrameBasePtr setOrigin_KF = processor_ptr_imu->setOrigin(x_origin, t);
    processor_ptr_odom3D->setOrigin(setOrigin_KF);

    //===================================================== END{END OF SETTINGS}

    //===================================================== PROCESS DATA

    // PROCESS IMU DATA

    Eigen::Vector6s data;
    data << 0.0, 0.0, -wolf::gravity()(2), 0.0, 0.0, 0.0;
    data += imuBias; //Biased IMU
    //Eigen::Vector6s imuBias_perturb((Eigen::Vector6s() << 0.001, 0.005, 0.0015, 0.005, 0.006, 0.01).finished() );
    Eigen::Vector6s imuBias_perturb((Eigen::Vector6s() << 0.001, 0, 0, 0, 0, 0).finished() );
    data += imuBias_perturb;
    Scalar dt = t.get();
    TimeStamp ts(0);
    wolf::CaptureIMUPtr imu_ptr = std::make_shared<CaptureIMU>(ts, sen_imu, data);

    while( (dt-t.get()) < (std::static_pointer_cast<ProcessorIMU>(processor_ptr_)->getMaxTimeSpan()) ){

        // Time and data variables
        dt += 0.001;
        ts.set(dt);
        imu_ptr->setTimeStamp(ts);
        imu_ptr->setData(data);

        // process data in capture
        imu_ptr->getTimeStamp();
        sen_imu->process(imu_ptr);
    }

    // PROCESS ODOM 3D DATA
    Eigen::Vector6s data_odom3D;
    data_odom3D << 0,0,0, 0,0,0;
    
    wolf::CaptureMotionPtr mot_ptr = std::make_shared<CaptureMotion>(ts, sen_odom3D, data_odom3D);
    sen_odom3D->process(mot_ptr);

    //===================================================== END{PROCESS DATA}

    //===================================================== SOLVER PART

    // Perturb bias before calling the solver

    FrameIMUPtr origin_KF = std::static_pointer_cast<FrameIMU>(wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front());
    FrameIMUPtr last_KF = std::static_pointer_cast<FrameIMU>(wolf_problem_ptr_->getTrajectoryPtr()->closestKeyFrameToTimeStamp(ts));

    origin_KF->fix();
    last_KF->fix();
    last_KF->getAccBiasPtr()->fix();

    wolf_problem_ptr_->print(4,1,1,1);
     
    std::cout << "\t\t\t ______solving______" << std::endl;
    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
    std::cout << summary.BriefReport() << std::endl;
    std::cout << "\t\t\t ______solved______" << std::endl;

    wolf_problem_ptr_->print(4,1,1,1);

    EXPECT_TRUE( (last_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )) << 
    "origin_KF position : " << origin_KF->getPPtr()->getVector().transpose() << "\n last_KF position : " << last_KF->getPPtr()->getVector().transpose() << std::endl;
    EXPECT_TRUE( (last_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )) << 
    "origin_KF orientation : " << origin_KF->getOPtr()->getVector().transpose() << "\n last_KF orientation : " << last_KF->getOPtr()->getVector().transpose() << std::endl;
    EXPECT_TRUE( (last_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )) << 
    "origin_KF velocity : " << origin_KF->getVPtr()->getVector().transpose() << "\n last_KF velocity : " << last_KF->getVPtr()->getVector().transpose() << std::endl;
    
    // IMU data was perturbed. We expect the bias of last_KF to have changed due to this perturbation
    EXPECT_TRUE( (last_KF->getAccBiasPtr()->getVector() - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )) <<
    "origin_KF Acc bias : " << origin_KF->getAccBiasPtr()->getVector().transpose() << "\n last_KF Acc bias : " << last_KF->getAccBiasPtr()->getVector().transpose() << std::endl;
    EXPECT_TRUE( (last_KF->getAccBiasPtr()->getVector() - (imuBias.head(3) + imuBias_perturb.head(3))).isMuchSmallerThan(1, wolf::Constants::EPS )) <<
    "last_KF Acc bias :" << last_KF->getAccBiasPtr()->getVector().transpose() << "\n  expected Acc bias : " << (imuBias.head(3) + imuBias_perturb.head(3)).transpose() << std::endl; 
    EXPECT_TRUE( (last_KF->getGyroBiasPtr()->getVector() - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS ));
    EXPECT_TRUE( (last_KF->getGyroBiasPtr()->getVector() - (imuBias.tail(3) + imuBias_perturb.tail(3))).isMuchSmallerThan(1, wolf::Constants::EPS ));

    // COMPUTE COVARIANCES
    //std::cout << "\t\t\t ______computing covariances______" << std::endl;
    ceres_manager_wolf_diff->computeCovariances(ALL);//ALL_MARGINALS, ALL
    //std::cout << "\t\t\t ______computed!______" << std::endl;

    //===================================================== END{SOLVER PART}
}

//_______________________________________________________________________________________________________________
// END ##################################### Imperfect IMU TESTS #####################################
//_______________________________________________________________________________________________________________


TEST_F(ProcessorIMU_Odom_tests, static_Optim_IMUOdom_nKFs_biasUnfixed)
{
    /* In this scenario, we simulate the integration of a perfect IMU that is not moving and we add an odometry measurement.
     * Initial State is [0,0,0, 0,0,0,1, 0,0,0] so we expect the Final State to be exactly the same
     * Origin KeyFrame is fixed
     * 
     * Finally, we can represent the graph as :
     *
     *  KF0 ---- constraintIMU ---- KF1 ---- constraintIMU ---- KF2 ---- (. . . . . .) ---- KFn
     *     \____________________________________constraintOdom3D___________________________/
     *
     * IMU BIAS UNFIXED
     *
     * With IMU data only, biases are not observable ! So covariance cannot be computed due to jacobian rank deficiency.
     * We must add an odometry to make covariances computable
     */
     
    //===================================================== END OF SETTINGS

    // set origin of processorMotions
    Eigen::VectorXs x_origin((Eigen::Matrix<wolf::Scalar,16,1>()<<0,0,0, 0,0,0,1, 0,0,0, 0,0,0, 0,0,0).finished());
    t.set(0);

    FrameBasePtr setOrigin_KF = processor_ptr_imu->setOrigin(x_origin, t);
    processor_ptr_odom3D->setOrigin(setOrigin_KF);

    wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->fix();
    //There should be 3 captures at origin_frame : CaptureOdom, captureIMU
    EXPECT_EQ((wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front())->getCaptureList().size(),2);
    ASSERT_TRUE(wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->isKey()) << "origin_frame is not a KeyFrame..." << std::endl;

    //===================================================== END{END OF SETTINGS}

    //===================================================== PROCESS DATA
    // PROCESS IMU DATA

    Eigen::Vector6s data;

    data << 0.00, 0.000, -wolf::gravity()(2), 0.0, 0.0, 0.0;
    Scalar dt = t.get();
    TimeStamp ts(0.001);
    wolf::CaptureIMUPtr imu_ptr = std::make_shared<CaptureIMU>(ts, sen_imu, data);

    while( (dt-t.get()) < (std::static_pointer_cast<ProcessorIMU>(processor_ptr_)->getMaxTimeSpan()*number_of_KF) ){
        
        // Time and data variables
        dt += 0.001;
        ts.set(dt);
        imu_ptr->setTimeStamp(ts);
        imu_ptr->setData(data);

        // process data in capture
        imu_ptr->getTimeStamp();
        sen_imu->process(imu_ptr);
    }

    // PROCESS ODOM 3D DATA
    Eigen::Vector6s data_odom3D;
    data_odom3D << 0,0,0, 0,0,0;
    
    wolf::CaptureMotionPtr mot_ptr = std::make_shared<CaptureMotion>(ts, sen_odom3D, data_odom3D);
    sen_odom3D->process(mot_ptr);

    //===================================================== END{PROCESS DATA}

    //===================================================== SOLVER PART

    FrameIMUPtr origin_KF = std::static_pointer_cast<FrameIMU>(wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front());
    FrameIMUPtr last_KF = std::static_pointer_cast<FrameIMU>(wolf_problem_ptr_->getTrajectoryPtr()->closestKeyFrameToTimeStamp(ts));

    //Check and print wolf tree
    /*if(wolf_problem_ptr_->check(1)){
        wolf_problem_ptr_->print(4,1,1,1);
    }*/

    wolf_problem_ptr_->print(4,1,1,1);
     
    std::cout << "\t\t\t ______solving______" << std::endl;
    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
    std::cout << summary.FullReport() << std::endl;
    std::cout << "\t\t\t ______solved______" << std::endl;

    wolf_problem_ptr_->print(4,1,1,1);

    ASSERT_TRUE( (last_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS ));
    ASSERT_TRUE( (last_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS ));
    ASSERT_TRUE( (last_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS ));
    ASSERT_TRUE( (last_KF->getAccBiasPtr()->getVector() - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )); //because we simulate a perfect IMU
    ASSERT_TRUE( (last_KF->getGyroBiasPtr()->getVector() - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )); //because we simulate a perfect IMU

    // COMPUTE COVARIANCES
    std::cout << "\t\t\t ______computing covariances______" << std::endl;
    ceres_manager_wolf_diff->computeCovariances(ALL);//ALL_MARGINALS, ALL
    std::cout << "\t\t\t ______computed!______" << std::endl;

    //===================================================== END{SOLVER PART}
}

TEST_F(ProcessorIMU_Odom_tests, static_Optim_IMUOdom_nKFs_biasFixed)
{
    /* In this scenario, we simulate the integration of a perfect IMU that is not moving and we add an odometry measurement.
     * Initial State is [0,0,0, 0,0,0,1, 0,0,0] so we expect the Final State to be exactly the same
     * Origin KeyFrame is fixed
     * 
     * Finally, we can represent the graph as :
     *
     *  KF0 ---- constraintIMU ---- KF1 ---- constraintIMU ---- KF2 ---- (. . . . . .) ---- KFn
     *     \____________________________________constraintOdom3D___________________________/
     *
     * IMU BIAS UNFIXED
     *
     * With IMU data only, biases are not observable ! So covariance cannot be computed due to jacobian rank deficiency.
     * We must add an odometry to make covariances computable
     */

    //===================================================== END OF SETTINGS

    // set origin of processorMotions
    Eigen::VectorXs x_origin((Eigen::Matrix<wolf::Scalar,16,1>()<<0,0,0, 0,0,0,1, 0,0,0, 0,0,0, 0,0,0).finished());
    t.set(0);

    FrameBasePtr setOrigin_KF = processor_ptr_imu->setOrigin(x_origin, t);
    processor_ptr_odom3D->setOrigin(setOrigin_KF);

    wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->fix();

    //There should be 2 captures at origin_frame : CaptureOdom, captureIMU
    EXPECT_EQ((wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front())->getCaptureList().size(),2);
    ASSERT_TRUE(wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->isKey()) << "origin_frame is not a KeyFrame..." << std::endl;

    //===================================================== END{END OF SETTINGS}

    //===================================================== PROCESS DATA

    // PROCESS IMU DATA

    Eigen::Vector6s data;
    data << 0.00, 0.000, -wolf::gravity()(2), 0.0, 0.0, 0.0;
    Scalar dt = t.get();
    TimeStamp ts(0.001);
    wolf::CaptureIMUPtr imu_ptr = std::make_shared<CaptureIMU>(ts, sen_imu, data);

    while( (dt-t.get()) < (std::static_pointer_cast<ProcessorIMU>(processor_ptr_)->getMaxTimeSpan()*number_of_KF) ){
        
        // Time and data variables
        dt += 0.001;
        ts.set(dt);
        imu_ptr->setTimeStamp(ts);
        imu_ptr->setData(data);

        // process data in capture
        imu_ptr->getTimeStamp();
        sen_imu->process(imu_ptr);
    }

    // PROCESS ODOM 3D DATA
    Eigen::Vector6s data_odom3D;
    data_odom3D << 0,0,0, 0,0,0;
    
    wolf::CaptureMotionPtr mot_ptr = std::make_shared<CaptureMotion>(ts, sen_odom3D, data_odom3D);
    sen_odom3D->process(mot_ptr);

        //Fix all biases StateBlocks
    for(FrameBasePtr it : wolf_problem_ptr_->getTrajectoryPtr()->getFrameList()){
        ( std::static_pointer_cast<FrameIMU>(it) )->getAccBiasPtr()->fix();
        ( std::static_pointer_cast<FrameIMU>(it) )->getGyroBiasPtr()->fix();
    }

    //===================================================== END{PROCESS DATA}

    //===================================================== SOLVER PART

    FrameIMUPtr origin_KF = std::static_pointer_cast<FrameIMU>(wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front());
    FrameIMUPtr last_KF = std::static_pointer_cast<FrameIMU>(wolf_problem_ptr_->getTrajectoryPtr()->closestKeyFrameToTimeStamp(ts));

    //Check and print wolf tree
    /*if(wolf_problem_ptr_->check(1)){
        wolf_problem_ptr_->print(4,1,1,1);
    }*/

    wolf_problem_ptr_->print(4,1,1,1);
     
    std::cout << "\t\t\t ______solving______" << std::endl;
    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
    std::cout << summary.FullReport() << std::endl;
    std::cout << "\t\t\t ______solved______" << std::endl;

    wolf_problem_ptr_->print(4,1,1,1);

    ASSERT_TRUE( (last_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS ));
    ASSERT_TRUE( (last_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS ));
    ASSERT_TRUE( (last_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS ));
    ASSERT_TRUE( (last_KF->getAccBiasPtr()->getVector() - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )); //because we simulate a perfect IMU
    ASSERT_TRUE( (last_KF->getGyroBiasPtr()->getVector() - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )); //because we simulate a perfect IMU

    // COMPUTE COVARIANCES
    std::cout << "\t\t\t ______computing covariances______" << std::endl;
    ceres_manager_wolf_diff->computeCovariances(ALL);//ALL_MARGINALS, ALL
    std::cout << "\t\t\t ______computed!______" << std::endl;

    //===================================================== END{SOLVER PART}
}



TEST_F(ProcessorIMU_Odom_tests, static_Optim_IMUOdom_SeveralKFs)
{
    /* In this scenario, we simulate the integration of a perfect IMU that is not moving and we add an odometry measurement.
     * Initial State is [0,0,0, 0,0,0,1, 0,0,0] so we expect the Final State to be exactly the same
     * Origin KeyFrame is fixed
     * 
     * Finally, we can represent the graph as :
     *
     *  KF0 ---- constraintIMU ---- KF1 ---- constraintIMU ---- KF2 ---- (. . . . . .) ---- KFn
     *     \____constraintOdom3D___/   \____constraintOdom3D___/   \____constraintOdom3D___/
     *
     * data integration is done for 10s (10 * max_time_span)
     * IMU data are processed at 1 Khz (every ms)
     * Odom3D data are processed at 100 Hz (every 10 ms)
     *
     * With IMU data only, biases are not observable ! So covariance cannot be computed due to jacobian rank deficiency.
     * We must add an odometry to make covariances computable
     */

    using std::shared_ptr;
    using std::make_shared;
    using std::static_pointer_cast;

    //===================================================== END OF SETTINGS

    // set origin of processorMotions
    Eigen::VectorXs x_origin((Eigen::Matrix<wolf::Scalar,16,1>()<<0,0,0, 0,0,0,1, 0,0,0, 0,0,0, 0,0,0).finished());
    t.set(0);

    FrameBasePtr setOrigin_KF = processor_ptr_imu->setOrigin(x_origin, t);
    processor_ptr_odom3D->setOrigin(setOrigin_KF);

    wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->fix();

    //There should be 2 captures at origin_frame : CaptureOdom, captureIMU
    EXPECT_EQ((wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front())->getCaptureList().size(),2);
    ASSERT_TRUE(wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->isKey()) << "origin_frame is not a KeyFrame..." << std::endl;

    //===================================================== END{END OF SETTINGS}

    //===================================================== PROCESS DATA
    // PROCESS DATA

    Eigen::Vector6s data, data_odom3D;
    data << 0.00, 0.000, -wolf::gravity()(2), 0.0, 0.0, 0.0;
    data_odom3D << 0,0,0, 0,0,0;
    Scalar dt = t.get();
    TimeStamp ts(0.000);
    wolf::CaptureIMUPtr imu_ptr = std::make_shared<CaptureIMU>(ts, sen_imu, data);
    wolf::CaptureMotionPtr mot_ptr = std::make_shared<CaptureMotion>(ts, sen_odom3D, data_odom3D);
    wolf_problem_ptr_->setProcessorMotion(processor_ptr_imu);
    unsigned int iter = 0;
    const unsigned int odom_freq = 10; //processing odometry data every 10 ms

    while( (dt-t.get()) < (std::static_pointer_cast<ProcessorIMU>(processor_ptr_)->getMaxTimeSpan()*number_of_KF) ){
        
        // PROCESS IMU DATA
        // Time and data variables
        dt += 0.001;
        iter++;
        ts.set(dt);
        imu_ptr->setTimeStamp(ts);
        imu_ptr->setData(data);

        // process data in capture
        imu_ptr->getTimeStamp();
        sen_imu->process(imu_ptr);

        if(iter == odom_freq) //every 100 ms
        {
            // PROCESS ODOM 3D DATA
            mot_ptr->setTimeStamp(ts);
            mot_ptr->setData(data_odom3D);
            sen_odom3D->process(mot_ptr);

            iter = 0;
        }
    }

    //===================================================== END{PROCESS DATA}

    //===================================================== SOLVER PART

    FrameIMUPtr origin_KF = std::static_pointer_cast<FrameIMU>(wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front());
    FrameIMUPtr last_KF = std::static_pointer_cast<FrameIMU>(wolf_problem_ptr_->getTrajectoryPtr()->closestKeyFrameToTimeStamp(ts));

    //Check and print wolf tree
    //wolf_problem_ptr_->print(4,1,1,1);
    /*if(wolf_problem_ptr_->check(1)){
        wolf_problem_ptr_->print(4,1,1,1);
    }*/

    wolf_problem_ptr_->print(4,1,1,1);
     
    std::cout << "\t\t\t ______solving______" << std::endl;
    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
    std::cout << summary.FullReport() << std::endl;
    std::cout << "\t\t\t ______solved______" << std::endl;
    
    wolf_problem_ptr_->print(4,1,1,1);

    ASSERT_TRUE( (last_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS ));
    ASSERT_TRUE( (last_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS ));
    ASSERT_TRUE( (last_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS ));
    ASSERT_TRUE( (last_KF->getAccBiasPtr()->getVector() - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )); //because we simulate a perfect IMU
    ASSERT_TRUE( (last_KF->getGyroBiasPtr()->getVector() - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )); //because we simulate a perfect IMU

    // COMPUTE COVARIANCES
    std::cout << "\t\t\t ______computing covariances______" << std::endl;
    ceres_manager_wolf_diff->computeCovariances(ALL);//ALL_MARGINALS, ALL
    std::cout << "\t\t\t ______computed!______" << std::endl;

    //===================================================== END{SOLVER PART}
}


TEST_F(ProcessorIMU_Odom_tests,Motion_IMU_and_Odom)
{
    /* In this test we will process both IMU and Odom3D data at the same time (in a same loop).
     * we use data simulating a perfect IMU doing some motion. This motion could be a pure translation or a pure rotation or anything
     *
     * HOWEVER : The test will use 2 data files which shall meet the following requirements :
     *
     * - a file containing only imu data in IMU frame will be provided. IMU data shall be written in the form [ax, ay, az, wx, wy, wz].
     *      The IMU measurements must include the measurement of the gravity.
     *      First line of this file will contain the initial condition in position, orientation and velocity only. (PQV formulation)
     *      Each line of the file correspond to a new set of IMU data.
     *      Each data will be separated from the previous one with a tabulation (\t).
     *      Finally, the following should give a clear idea of how the file is and summarizes the previous points :
     *          (line1) px_initial\t   py_initial\t   pz_initial\t   qx_initial\t   qy_initial\t   qz_initial\t   qw_initial\t   vx_initial\t   vy_initial\t   vz_initial\t 
     *          (line2) TimesTamp1\t   ax1\t   ay1\t   az1\t   wx1\t   wy1\t   wz1\t   
     *          (line3) TimesTamp2\t   ax2\t   ay2\t   az2\t   wx2\t   wy2\t   wz2\t   
     *          (.           .           .       .       .       .       .       .  )
     *          (lineN) TimesTampN\t   axN\t   ayN\t   azN\t   wxN\t   wyN\t   wzN\t
     *
     * - a file containing only odometry measurements will be provided. This file only contains odometry in the form PO (using orientation vector here !)
     *      Here is how the file should look like :
     *          (line1) TimeStamp1\t    px1\t   py1\t   pz1\t   ox1\t   oy1\t   oz1
     *          (line2) TimeStamp2\t    px2\t   py2\t   pz2\t   ox2\t   oy2\t   oz2
     *          (   .         .           .       .       .       .       .       . )
     *          (lineN) TimeStampN\t    pxN\t   pyN\t   pzN\t   oxN\t   oyN\t   ozN
     *
     *      TIMESTAMPS FROM IMU DATA FILE AND ODOM FILE MUST MATCH PERFECTLY !!!!
     *      This does not mean that for each IMU data you must provide an odometry data !
     *
     *      Integration ends once the end of imu or odom data file is reached.
     *
     */

    using std::shared_ptr;
    using std::make_shared;
    using std::static_pointer_cast;

    std::ifstream imu_data_input;
    std::ifstream odom_data_input;

    imu_data_input.open(filename_motion_imu_data);
    odom_data_input.open(filename_motion_odom);
    WOLF_INFO("pure translation imu file: ", filename_motion_imu_data)
    WOLF_INFO( "pure translation odom: ", filename_motion_odom)

    if(!imu_data_input.is_open() || !odom_data_input.is_open()){
        std::cerr << "Failed to open data files... Exiting" << std::endl;
        ADD_FAILURE();
    }

    //prepare creation of file if DEBUG_RESULTS activated
    #ifdef DEBUG_RESULTS
        std::ofstream debug_results;
        debug_results.open("debug_results.dat");
        if(debug_results)
            debug_results << "%%TimeStamp\t"
                        << "dp_x\t" << "dp_y\t" << "dp_z\t" << "dq_x\t" << "dq_y\t" << "dq_z\t" << "dq_w\t" << "dv_x\t" << "dv_y\t" << "dv_z\t"
                        << "Dp_x\t" << "Dp_y\t" << "Dp_z\t" << "Dq_x\t" << "Dq_y\t" << "Dq_z\t" << "Dq_w\t" << "Dv_x\t" << "Dv_y\t" << "Dv_z\t"
                        << "X_x\t" << "X_y\t" << "X_z\t" << "Xq_x\t" << "Xq_y\t" << "Xq_z\t" << "Xq_w\t" << "Xv_x\t" << "Xv_y\t" << "Xv_z\t" << std::endl;
    #endif


    //===================================================== SETTING PROBLEM
    std::string wolf_root = _WOLF_ROOT_DIR;

    // reset origin of problem
    Eigen::VectorXs x_origin((Eigen::Matrix<wolf::Scalar,16,1>()<<0,0,0, 0,0,0,1, 0,0,0, 0,0,0, 0,0,0).finished());

    // initial conditions defined from data file
    // remember that matlab's quaternion is W,X,Y,Z and the one in Eigen has X,Y,Z,W form
    imu_data_input >> x_origin[0] >> x_origin[1] >> x_origin[2] >> x_origin[6] >> x_origin[3] >> x_origin[4] >> x_origin[5] >> x_origin[7] >> x_origin[8] >> x_origin[9];

    t.set(0);
    FrameBasePtr origin_KF = processor_ptr_imu->setOrigin(x_origin, t);
    processor_ptr_odom3D->setOrigin(origin_KF);
    //wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->fix();
    origin_KF->getPPtr()->fix();
    origin_KF->getOPtr()->fix();
    origin_KF->getVPtr()->fix();

    //===================================================== END{SETTING PROBLEM}

    //===================================================== PROCESS DATA
    // PROCESS DATA

    Eigen::Vector6s data_imu, data_odom3D;
    data_imu << 0,0,-wolf::gravity()(2), 0,0,0;
    data_odom3D << 0,0,0, 0,0,0;

    Scalar input_clock;
    TimeStamp ts(0);
    TimeStamp t_odom(0);
    wolf::CaptureIMUPtr imu_ptr = std::make_shared<CaptureIMU>(ts, sen_imu, data_imu);
    wolf::CaptureMotionPtr mot_ptr = std::make_shared<CaptureMotion>(t, sen_odom3D, data_odom3D);
    
    //read first odom data from file
    odom_data_input >> input_clock >> data_odom3D[0] >> data_odom3D[1] >> data_odom3D[2] >> data_odom3D[3] >> data_odom3D[4] >> data_odom3D[5];
    t_odom.set(input_clock);
    //when we find a IMU timestamp corresponding with this odometry timestamp then we process odometry measurement

    while( !imu_data_input.eof() && !odom_data_input.eof() )
    {
        // PROCESS IMU DATA
        // Time and data variables
        imu_data_input >> input_clock >> data_imu[0] >> data_imu[1] >> data_imu[2] >> data_imu[3] >> data_imu[4] >> data_imu[5]; //Ax, Ay, Az, Gx, Gy, Gz

        ts.set(input_clock);
        imu_ptr->setTimeStamp(ts);
        imu_ptr->setData(data_imu);

        // process data in capture
        imu_ptr->getTimeStamp();
        sen_imu->process(imu_ptr);

        if(ts.get() == t_odom.get()) //every 100 ms
        {
            // PROCESS ODOM 3D DATA
            mot_ptr->setTimeStamp(t_odom);
            mot_ptr->setData(data_odom3D);
            sen_odom3D->process(mot_ptr);

            //prepare next odometry measurement if there is any
            odom_data_input >> input_clock >> data_odom3D[0] >> data_odom3D[1] >> data_odom3D[2] >> data_odom3D[3] >> data_odom3D[4] >> data_odom3D[5];
            t_odom.set(input_clock);
        }
    }

    //closing file
    imu_data_input.close();
    odom_data_input.close();

    //===================================================== END{PROCESS DATA}

    //===================================================== SOLVER PART

    //Check and print wolf tree
    if(wolf_problem_ptr_->check()){
        wolf_problem_ptr_->print(4,1,1,1);
    }
     
    std::cout << "\t\t\t ______solving______" << std::endl;
    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
    std::cout << summary.FullReport() << std::endl;
    std::cout << "\t\t\t ______solved______" << std::endl;
    
    wolf_problem_ptr_->print(4,1,1,1);

    // COMPUTE COVARIANCES
    std::cout << "\t\t\t ______computing covariances______" << std::endl;
    ceres_manager_wolf_diff->computeCovariances(ALL);//ALL_MARGINALS, ALL
    std::cout << "\t\t\t ______computed!______" << std::endl;

    //===================================================== END{SOLVER PART}
}

int main(int argc, char **argv)
{
    /*  LIST OF TESTS :
     *      - ProcessorOdom3D:
     *          - static_ceresOptimisation_Odom_PO
     *          - static_ceresOptimisation_convergenceOdom_PO
     *          - static_ceresOptimisation_convergenceOdom_POV
     *      - ProcessorIMU:
     *          - static_ceresOptimisation_fixBias
     *      - ProcessorIMU_Odom_tests:
     *          - static_Optim_IMUOdom_2KF
     *          - static_Optim_IMUOdom_nKFs_biasUnfixed
     *          - static_Optim_IMUOdom_nKFs_biasFixed
     *          - static_Optim_IMUOdom_SeveralKFs
     *          - Motion_IMU_and_Odom
     */

     /* Running simply ./gtest_processorMotion_optimization_testCase will run all tests that do not need any input file
      * If you want to run specific test, just modify booleans in 'gtest_processorMotion_optimization_testCase.yaml.yaml' to specify the tests you want to run
      * In this file you can also modify some values that will be used in the tests (example : number of keyframes)
      * to make it simple, we modify global variables given values in this file
      */

     //We use parser to determine from yaml which tests will be executed
     //initialize all variables
     //Default is to only run tests which do not need any file input
     bool static_ceresOptimisation_Odom_PO = true;
     bool static_ceresOptimisation_convergenceOdom_PO = true;
     bool static_ceresOptimisation_convergenceOdom_POV = true;
     bool static_ceresOptimisation_fixBias = true;
     bool static_Optim_IMUOdom_2KF = true;
     bool static_Optim_IMUOdom_nKFs_biasUnfixed = true;
     bool static_Optim_IMUOdom_nKFs_biasFixed = true;
     bool static_Optim_IMUOdom_SeveralKFs = true;
     bool Motion_IMU_and_Odom = false;

     std::string wolf_root = _WOLF_ROOT_DIR;
     std::string gtest_ceres_yaml_path;
     gtest_ceres_yaml_path = wolf_root + "/src/examples/gtest_processorMotion_optimization_testCase.yaml";
     std::string tests_to_run = "";

     //if first argument is --use_yaml then we parse yaml file to configure the execution of tests
     if (argc == 2)
     {
        const char * option = argv[1];
        std:: string option_string(option);
         
        if(option_string == "--use_yaml") // use yaml option detected
        {
            YAML::Node gtest_config = YAML::LoadFile(gtest_ceres_yaml_path);

            static_ceresOptimisation_Odom_PO               = gtest_config["ProcessorOdom3D"]["static_ceresOptimisation_Odom_PO"]               .as<bool>();
            static_ceresOptimisation_convergenceOdom_PO    = gtest_config["ProcessorOdom3D"]["static_ceresOptimisation_convergenceOdom_PO"]    .as<bool>();
            static_ceresOptimisation_convergenceOdom_POV   = gtest_config["ProcessorOdom3D"]["static_ceresOptimisation_convergenceOdom_POV"]   .as<bool>();

            static_ceresOptimisation_fixBias               = gtest_config["ProcessorIMU"]["static_ceresOptimisation_fixBias"]                  .as<bool>();
            static_Optim_IMUOdom_2KF                       = gtest_config["ProcessorIMU_Odom_tests"]["static_Optim_IMUOdom_2KF"]               .as<bool>();
            static_Optim_IMUOdom_nKFs_biasUnfixed          = gtest_config["ProcessorIMU_Odom_tests"]["static_Optim_IMUOdom_nKFs_biasUnfixed"]  .as<bool>();
            static_Optim_IMUOdom_nKFs_biasFixed            = gtest_config["ProcessorIMU_Odom_tests"]["static_Optim_IMUOdom_nKFs_biasFixed"]    .as<bool>();
            static_Optim_IMUOdom_SeveralKFs                = gtest_config["ProcessorIMU_Odom_tests"]["static_Optim_IMUOdom_SeveralKFs"]        .as<bool>();
            Motion_IMU_and_Odom                            = gtest_config["ProcessorIMU_Odom_tests"]["Motion_IMU_and_Odom"]                    .as<bool>();
            number_of_KF                                   = gtest_config["ProcessorIMU_Odom_tests"]["n_KF"]                                   .as<unsigned int>();

            std::string motion_imu_data_string    = gtest_config["ProcessorIMU_Odom_tests"]["motion_IMU_filepath"]                             .as<std::string>();
            std::string motion_odom_string        = gtest_config["ProcessorIMU_Odom_tests"]["motion_Odom_filepath"]                            .as<std::string>();

             //store filename in global variables
            filename_motion_imu_data   = new char[motion_imu_data_string.length() + 1];
            filename_motion_odom       = new char[motion_odom_string.length()     + 1];

            std::strcpy(filename_motion_imu_data, motion_imu_data_string.c_str());
            std::strcpy(filename_motion_odom, motion_odom_string.c_str());
        }
     }

     // use booleans to Filter all tests that will be executed
    if(static_ceresOptimisation_Odom_PO)
        tests_to_run +=":ProcessorOdom3D.static_ceresOptimisation_Odom_PO";
    if(static_ceresOptimisation_convergenceOdom_PO)
        tests_to_run +=":ProcessorOdom3D.static_ceresOptimisation_convergenceOdom_PO";
    if(static_ceresOptimisation_convergenceOdom_POV)
        tests_to_run +=":ProcessorOdom3D.static_ceresOptimisation_convergenceOdom_POV";
    
    if(static_ceresOptimisation_fixBias)
        tests_to_run +=":ProcessorIMU.static_ceresOptimisation_fixBias";
    if(static_Optim_IMUOdom_2KF)
        tests_to_run +=":ProcessorIMU_Odom_tests.static_Optim_IMUOdom_2KF";
    if(static_Optim_IMUOdom_nKFs_biasUnfixed)
        tests_to_run +=":ProcessorIMU_Odom_tests.static_Optim_IMUOdom_nKFs_biasUnfixed";
    if(static_Optim_IMUOdom_nKFs_biasFixed)
        tests_to_run +=":ProcessorIMU_Odom_tests.static_Optim_IMUOdom_nKFs_biasFixed";
    if(static_Optim_IMUOdom_SeveralKFs)
        tests_to_run +=":ProcessorIMU_Odom_tests.static_Optim_IMUOdom_SeveralKFs";
    if(Motion_IMU_and_Odom)
        tests_to_run +=":ProcessorIMU_Odom_tests.Motion_IMU_and_Odom";


  ::testing::InitGoogleTest(&argc, argv);
  ::testing::GTEST_FLAG(filter) = tests_to_run;
  //::testing::GTEST_FLAG(filter) = "ProcessorIMU_Odom_tests_details.static_Optim_IMUOdom_2KF_perturbate_GyroBiasOrigin_FixedLast_extensive_**";
  //::testing::GTEST_FLAG(filter) = "ProcessorIMU_Odom_tests.IMU_Biased_perturbData";
  ::testing::GTEST_FLAG(filter) = "ProcessorIMU_Odom_tests_plateform_simulation.No_Perturbation";
  //google::InitGoogleLogging(argv[0]);
  return RUN_ALL_TESTS();
}
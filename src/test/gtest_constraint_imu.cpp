//Wolf
#include "wolf.h"
#include "problem.h"
#include "sensor_imu.h"
#include "capture_imu.h"
#include "state_block.h"
#include "state_quaternion.h"
#include "processor_imu.h"
#include "processor_odom_3D.h"
#include "ceres_wrapper/ceres_manager.h"

#include "utils_gtest.h"
#include "../src/logging.h"

#include <iostream>
#include <fstream>

//#define DEBUG_RESULTS

using namespace Eigen;
using namespace std;
using namespace wolf;

class ConstraintIMU_PrcImuOdom : public testing::Test
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


    virtual void SetUp()
    {
        using std::shared_ptr;
        using std::make_shared;
        using std::static_pointer_cast;

        //===================================================== SETTING PROBLEM
        std::string wolf_root = _WOLF_ROOT_DIR;

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
        prc_odom3D_params->max_time_span = 0.999;
        prc_odom3D_params->max_buff_length = 1000000000; //make it very high so that this condition will not pass
        prc_odom3D_params->dist_traveled = 1000000000;
        prc_odom3D_params->angle_turned = 1000000000;

        ProcessorBasePtr processor_ptr_odom = wolf_problem_ptr_->installProcessor("ODOM 3D", "odom", sen1_ptr, prc_odom3D_params);
        sen_odom3D = std::static_pointer_cast<SensorOdom3D>(sen1_ptr);
        processor_ptr_odom3D = std::static_pointer_cast<ProcessorOdom3D>(processor_ptr_odom);

        FrameBasePtr setOrigin_KF = processor_ptr_imu->setOrigin(x0, t);
        processor_ptr_odom3D->setOrigin(setOrigin_KF);
        origin_KF = std::static_pointer_cast<FrameIMU>(setOrigin_KF);

    //===================================================== END{SETTING PROBLEM}
    }

    virtual void TearDown(){}
};

class ConstraintIMU_accBiasObservation : public testing::Test
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
        Eigen::Vector6s origin_bias;


    virtual void SetUp()
    {
        using std::shared_ptr;
        using std::make_shared;
        using std::static_pointer_cast;

        //===================================================== SETTING PROBLEM
        std::string wolf_root = _WOLF_ROOT_DIR;

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
        SensorBasePtr sen1_ptr = wolf_problem_ptr_->installSensor("ODOM 3D", "odom", (Vector7s()<<0,0,0,0,0,0,1).finished(), wolf_root + "/src/examples/sensor_odom_3D_HQ.yaml");
        ProcessorOdom3DParamsPtr prc_odom3D_params = std::make_shared<ProcessorOdom3DParams>();
        prc_odom3D_params->max_time_span = 0.999;
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
        std::string imu_filepath_string(wolf_root + "/src/test/data/IMU/Static_and_odom/data_bias_check2.txt");
        std::string odom_filepath_string(wolf_root + "/src/test/data/IMU/Static_and_odom/odom_bias_check2.txt");

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
        imu_data_input >> origin_bias[0] >> origin_bias[1] >> origin_bias[2] >> origin_bias[3] >> origin_bias[4] >> origin_bias[5];

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

class ConstraintIMU_testBase : public testing::Test
{
    public:
        wolf::ProblemPtr wolf_problem_ptr_;
        wolf::TimeStamp ts;
        wolf::CaptureIMUPtr imu_ptr;
        Eigen::VectorXs state_vec;
        Eigen::VectorXs delta_preint;
        Eigen::Matrix<wolf::Scalar,9,9> delta_preint_cov;
        std::shared_ptr<wolf::FeatureIMU> feat_imu;
        wolf::FrameIMUPtr last_frame;
        wolf::FrameIMUPtr origin_frame;
        Eigen::Matrix<wolf::Scalar,9,6> dD_db_jacobians;
    
    virtual void SetUp()
    {
        using namespace wolf;
        using std::shared_ptr;
        using std::make_shared;
        using std::static_pointer_cast;
        
        // Wolf problem
        wolf_problem_ptr_ = Problem::create(FRM_PQVBB_3D);
        Eigen::VectorXs IMU_extrinsics(7);
        IMU_extrinsics << 0,0,0, 0,0,0,1; // IMU pose in the robot
        SensorBasePtr sensor_ptr = wolf_problem_ptr_->installSensor("IMU", "Main IMU", IMU_extrinsics, shared_ptr<IntrinsicsBase>());
        wolf_problem_ptr_->installProcessor("IMU", "IMU pre-integrator", "Main IMU", "");

    // Time and data variables
        TimeStamp t;
        Eigen::Vector6s data_;
        state_vec.resize(16);
        t.set(0.01);

    // Set the origin
        Eigen::VectorXs x0(16);
        x0 << 0,0,0,  0,0,0,1,  0,0,0,  0,0,0,  0,0,0; // Try some non-zero biases
        wolf_problem_ptr_->getProcessorMotionPtr()->setOrigin(x0, t);

    //create a keyframe at origin
        ts = wolf_problem_ptr_->getProcessorMotionPtr()->getBuffer().get().back().ts_;
        Eigen::VectorXs origin_state = x0;
        origin_frame = std::make_shared<FrameIMU>(KEY_FRAME, ts, origin_state);
        wolf_problem_ptr_->getTrajectoryPtr()->addFrame(origin_frame);
    
    // Create one capture to store the IMU data arriving from (sensor / callback / file / etc.)
        imu_ptr = std::make_shared<CaptureIMU>(t, sensor_ptr, data_);
        imu_ptr->setFramePtr(origin_frame);

    //process data
        data_ << 2, 0, 9.8, 0, 0, 0;
        t.set(0.1);
        // Expected state after one integration
        //x << 0.01,0,0, 0,0,0,1, 0.2,0,0, 0,0,0, 0,0,0; // advanced at a=2m/s2 during 0.1s ==> dx = 0.5*2*0.1^2 = 0.01; dvx = 2*0.1 = 0.2
    // assign data to capture
        imu_ptr->setData(data_);
        imu_ptr->setTimeStamp(t);
    // process data in capture
        sensor_ptr->process(imu_ptr);
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

TEST_F(ConstraintIMU_testBase, constructorIMU)
{   
    using namespace wolf;

    //create FrameIMU
    ts = 0.1;
    state_vec << 0.01,0,0, 0,0,0,1, 0.2,0,0, 0,0,0, 0,0,0;
   	last_frame = std::make_shared<FrameIMU>(KEY_FRAME, ts, state_vec);
    //create a feature
    delta_preint_cov = Eigen::MatrixXs::Identity(9,9);
    delta_preint.resize(10);
    delta_preint << 0.01,0,0.049, 0,0,0,1, 0.2,0,0.98;
    dD_db_jacobians = Eigen::Matrix<wolf::Scalar,9,6>::Random();
    feat_imu = std::make_shared<FeatureIMU>(delta_preint, delta_preint_cov, imu_ptr, dD_db_jacobians);

    //create the constraint
    ConstraintIMU constraint_imu(feat_imu,last_frame);
}

TEST_F(ConstraintIMU_PrcImuOdom, gyro_biased_static)
{
    //===================================================== PROCESS DATA
    // PROCESS IMU DATA

    Eigen::Vector6s data;
    Eigen::Vector6s bias((Eigen::Vector6s()<< 0, 0, 0, 0.03, 0.2, 0.007).finished());
    data << 0.00, 0.000, -wolf::gravity()(2), 0.0, 0.0, 0.0;
    data += bias;
    Scalar dt = t.get();
    TimeStamp ts(0.001);
    wolf::CaptureIMUPtr imu_ptr = std::make_shared<CaptureIMU>(ts, sen_imu, data);

    while( (dt-t.get()) <= (std::static_pointer_cast<ProcessorIMU>(processor_ptr_)->getMaxTimeSpan()) ){
        
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

    FrameIMUPtr last_KF = std::static_pointer_cast<FrameIMU>(wolf_problem_ptr_->getTrajectoryPtr()->closestKeyFrameToTimeStamp(ts));

    origin_KF->unfix();
    origin_KF->getPPtr()->fix();
    origin_KF->getOPtr()->fix();
    origin_KF->getVPtr()->fix();

    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
    std::cout << summary.BriefReport() << std::endl;

    wolf_problem_ptr_->print(4,1,1,1);

    ceres_manager_wolf_diff->computeCovariances(ALL);//ALL_MARGINALS, ALL
    //===================================================== END{PROCESS DATA}

    ASSERT_TRUE((last_KF->getPPtr()->getVector() - (Eigen::Vector3s()<< 0, 0, 0).finished()).isMuchSmallerThan(1, wolf::Constants::EPS ));
    EXPECT_TRUE((last_KF->getAccBiasPtr()->getVector() - bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS*10 )) << "last_KF Acc bias : " << last_KF->getAccBiasPtr()->getVector().transpose() << 
    "\n introduced Acc bias : " << bias.head(3).transpose() << std::endl;
    ASSERT_TRUE((last_KF->getGyroBiasPtr()->getVector() - bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS )) << "last_KF Gyro bias : " << last_KF->getGyroBiasPtr()->getVector().transpose() << 
    "\n introduced Gyro bias : " << bias.tail(3).transpose() << std::endl;

    ConstraintBaseList origin_KF_constraints = origin_KF->getConstrainedByList();
    Eigen::Matrix<wolf::Scalar,15,1> residuals;

    Eigen::Vector3s p1(origin_KF->getPPtr()->getVector());
    Eigen::Vector4s q1_vec(origin_KF->getOPtr()->getVector());
    Eigen::Map<Quaternions> q1(q1_vec.data());
    Eigen::Vector3s v1(origin_KF->getVPtr()->getVector());
    Eigen::Vector3s ab1(origin_KF->getAccBiasPtr()->getVector());
    Eigen::Vector3s wb1(origin_KF->getGyroBiasPtr()->getVector());
    Eigen::Vector3s p2(last_KF->getPPtr()->getVector());
    Eigen::Vector4s q2_vec(last_KF->getOPtr()->getVector());
    Eigen::Map<Quaternions> q2(q2_vec.data());
    Eigen::Vector3s v2(last_KF->getVPtr()->getVector());
    Eigen::Vector3s ab2(last_KF->getAccBiasPtr()->getVector());
    Eigen::Vector3s wb2(last_KF->getGyroBiasPtr()->getVector());

    for(auto ctr_ptr : origin_KF_constraints)
    {
        if (ctr_ptr->getTypeId() == wolf::CTR_IMU)
            {
                std::static_pointer_cast<ConstraintIMU>(ctr_ptr)->getResiduals(p1, q1, v1, ab1, wb1, p2, q2, v2, ab2, wb2, residuals);
                WOLF_DEBUG("ConstraintIMU residuals : \n", residuals.transpose())
            }
    }
}

TEST_F(ConstraintIMU_PrcImuOdom, gyroZ_biased_static)
{
    //===================================================== PROCESS DATA
    // PROCESS IMU DATA

    Eigen::Vector6s data;
    Eigen::Vector6s bias((Eigen::Vector6s()<< 0, 0, 0, 0, 0, 0.027).finished());
    data << 0.00, 0.000, -wolf::gravity()(2), 0.0, 0.0, 0.0;
    data += bias;
    Scalar dt = t.get();
    TimeStamp ts(0.001);
    wolf::CaptureIMUPtr imu_ptr = std::make_shared<CaptureIMU>(ts, sen_imu, data);

    while( (dt-t.get()) <= (std::static_pointer_cast<ProcessorIMU>(processor_ptr_)->getMaxTimeSpan()) ){
        
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

    FrameIMUPtr last_KF = std::static_pointer_cast<FrameIMU>(wolf_problem_ptr_->getTrajectoryPtr()->closestKeyFrameToTimeStamp(ts));

    origin_KF->unfix();
    origin_KF->getPPtr()->fix();
    origin_KF->getOPtr()->fix();

    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
    std::cout << summary.BriefReport() << std::endl;

    last_KF->getAccBiasPtr()->fix();
    last_KF->getGyroBiasPtr()->fix();
    origin_KF->getVPtr()->fix();

    wolf_problem_ptr_->print(4,1,1,1);

    ceres_manager_wolf_diff->computeCovariances(ALL);//ALL_MARGINALS, ALL
    //===================================================== END{PROCESS DATA}

    ASSERT_TRUE((last_KF->getPPtr()->getVector() - (Eigen::Vector3s()<< 0, 0, 0).finished()).isMuchSmallerThan(1, wolf::Constants::EPS ));
    EXPECT_TRUE((last_KF->getAccBiasPtr()->getVector() - bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS )) << "last_KF Acc bias : " << last_KF->getAccBiasPtr()->getVector().transpose() << 
    "\n introduced Acc bias : " << bias.head(3).transpose() << std::endl;
    ASSERT_TRUE((last_KF->getGyroBiasPtr()->getVector() - bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS )) << "last_KF Gyro bias : " << last_KF->getGyroBiasPtr()->getVector().transpose() << 
    "\n introduced Gyro bias : " << bias.tail(3).transpose() << std::endl;

    ConstraintBaseList origin_KF_constraints = origin_KF->getConstrainedByList();
    Eigen::Matrix<wolf::Scalar,15,1> residuals;

    Eigen::Vector3s p1(origin_KF->getPPtr()->getVector());
    Eigen::Vector4s q1_vec(origin_KF->getOPtr()->getVector());
    Eigen::Map<Quaternions> q1(q1_vec.data());
    Eigen::Vector3s v1(origin_KF->getVPtr()->getVector());
    Eigen::Vector3s ab1(origin_KF->getAccBiasPtr()->getVector());
    Eigen::Vector3s wb1(origin_KF->getGyroBiasPtr()->getVector());
    Eigen::Vector3s p2(last_KF->getPPtr()->getVector());
    Eigen::Vector4s q2_vec(last_KF->getOPtr()->getVector());
    Eigen::Map<Quaternions> q2(q2_vec.data());
    Eigen::Vector3s v2(last_KF->getVPtr()->getVector());
    Eigen::Vector3s ab2(last_KF->getAccBiasPtr()->getVector());
    Eigen::Vector3s wb2(last_KF->getGyroBiasPtr()->getVector());

    for(auto ctr_ptr : origin_KF_constraints)
    {
        if (ctr_ptr->getTypeId() == wolf::CTR_IMU)
            {
                std::static_pointer_cast<ConstraintIMU>(ctr_ptr)->getResiduals(p1, q1, v1, ab1, wb1, p2, q2, v2, ab2, wb2, residuals);
                WOLF_DEBUG("ConstraintIMU residuals : \n", residuals.transpose())
            }
    }
}

TEST_F(ConstraintIMU_PrcImuOdom, gyroX_biased_static)
{
    //===================================================== PROCESS DATA
    // PROCESS IMU DATA

    Eigen::Vector6s data;
    Eigen::Vector6s bias((Eigen::Vector6s()<< 0, 0, 0, 0.027, 0, 0).finished());
    data << 0.00, 0.000, -wolf::gravity()(2), 0.0, 0.0, 0.0;
    data += bias;
    Scalar dt = t.get();
    TimeStamp ts(0.001);
    wolf::CaptureIMUPtr imu_ptr = std::make_shared<CaptureIMU>(ts, sen_imu, data);

    while( (dt-t.get()) <= (std::static_pointer_cast<ProcessorIMU>(processor_ptr_)->getMaxTimeSpan()) ){
        
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

    FrameIMUPtr last_KF = std::static_pointer_cast<FrameIMU>(wolf_problem_ptr_->getTrajectoryPtr()->closestKeyFrameToTimeStamp(ts));

    origin_KF->unfix();
    origin_KF->getPPtr()->fix();
    origin_KF->getOPtr()->fix();

    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
    std::cout << summary.BriefReport() << std::endl;

    last_KF->getAccBiasPtr()->fix();
    last_KF->getGyroBiasPtr()->fix();
    origin_KF->getVPtr()->fix();

    wolf_problem_ptr_->print(4,1,1,1);

    ceres_manager_wolf_diff->computeCovariances(ALL);//ALL_MARGINALS, ALL
    //===================================================== END{PROCESS DATA}

    ASSERT_TRUE((last_KF->getPPtr()->getVector() - (Eigen::Vector3s()<< 0, 0, 0).finished()).isMuchSmallerThan(1, wolf::Constants::EPS ));
    EXPECT_TRUE((last_KF->getAccBiasPtr()->getVector() - bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS )) << "last_KF Acc bias : " << last_KF->getAccBiasPtr()->getVector().transpose() << 
    "\n introduced Acc bias : " << bias.head(3).transpose() << std::endl;
    ASSERT_TRUE((last_KF->getGyroBiasPtr()->getVector() - bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS )) << "last_KF Gyro bias : " << last_KF->getGyroBiasPtr()->getVector().transpose() << 
    "\n introduced Gyro bias : " << bias.tail(3).transpose() << std::endl;

    ConstraintBaseList origin_KF_constraints = origin_KF->getConstrainedByList();
    Eigen::Matrix<wolf::Scalar,15,1> residuals;

    Eigen::Vector3s p1(origin_KF->getPPtr()->getVector());
    Eigen::Vector4s q1_vec(origin_KF->getOPtr()->getVector());
    Eigen::Map<Quaternions> q1(q1_vec.data());
    Eigen::Vector3s v1(origin_KF->getVPtr()->getVector());
    Eigen::Vector3s ab1(origin_KF->getAccBiasPtr()->getVector());
    Eigen::Vector3s wb1(origin_KF->getGyroBiasPtr()->getVector());
    Eigen::Vector3s p2(last_KF->getPPtr()->getVector());
    Eigen::Vector4s q2_vec(last_KF->getOPtr()->getVector());
    Eigen::Map<Quaternions> q2(q2_vec.data());
    Eigen::Vector3s v2(last_KF->getVPtr()->getVector());
    Eigen::Vector3s ab2(last_KF->getAccBiasPtr()->getVector());
    Eigen::Vector3s wb2(last_KF->getGyroBiasPtr()->getVector());

    for(auto ctr_ptr : origin_KF_constraints)
    {
        if (ctr_ptr->getTypeId() == wolf::CTR_IMU)
            {
                std::static_pointer_cast<ConstraintIMU>(ctr_ptr)->getResiduals(p1, q1, v1, ab1, wb1, p2, q2, v2, ab2, wb2, residuals);
                WOLF_DEBUG("ConstraintIMU residuals : \n", residuals.transpose())
            }
    }
}

TEST_F(ConstraintIMU_PrcImuOdom, acc_biased_static)
{
    //===================================================== PROCESS DATA
    // PROCESS IMU DATA

    Eigen::Vector6s data;
    Eigen::Vector6s bias((Eigen::Vector6s()<< 0.03, 0.2, 0.007, 0, 0, 0).finished());
    data << 0.00, 0.000, -wolf::gravity()(2), 0.0, 0.0, 0.0;
    data += bias;
    Scalar dt = t.get();
    TimeStamp ts(0.001);
    wolf::CaptureIMUPtr imu_ptr = std::make_shared<CaptureIMU>(ts, sen_imu, data);

    while( (dt-t.get()) <= (std::static_pointer_cast<ProcessorIMU>(processor_ptr_)->getMaxTimeSpan()) ){
        
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

    FrameIMUPtr last_KF = std::static_pointer_cast<FrameIMU>(wolf_problem_ptr_->getTrajectoryPtr()->closestKeyFrameToTimeStamp(ts));

    origin_KF->unfix();
    origin_KF->getPPtr()->fix();
    origin_KF->getOPtr()->fix();
    origin_KF->getVPtr()->fix();

    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
    std::cout << summary.BriefReport() << std::endl;

    last_KF->getAccBiasPtr()->fix();
    last_KF->getGyroBiasPtr()->fix();

    wolf_problem_ptr_->print(4,1,1,1);

    ceres_manager_wolf_diff->computeCovariances(ALL);//ALL_MARGINALS, ALL
    //===================================================== END{PROCESS DATA}

    ASSERT_TRUE((last_KF->getPPtr()->getVector() - (Eigen::Vector3s()<< 0, 0, 0).finished()).isMuchSmallerThan(1, wolf::Constants::EPS ));
    EXPECT_TRUE((last_KF->getAccBiasPtr()->getVector() - bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS )) << "last_KF Acc bias : " << last_KF->getAccBiasPtr()->getVector().transpose() << 
    "\n introduced Acc bias : " << bias.head(3).transpose() << std::endl;
    ASSERT_TRUE((last_KF->getGyroBiasPtr()->getVector() - bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS )) << "last_KF Gyro bias : " << last_KF->getGyroBiasPtr()->getVector().transpose() << 
    "\n introduced Gyro bias : " << bias.tail(3).transpose() << std::endl;

    ConstraintBaseList origin_KF_constraints = origin_KF->getConstrainedByList();
    Eigen::Matrix<wolf::Scalar,15,1> residuals;

    Eigen::Vector3s p1(origin_KF->getPPtr()->getVector());
    Eigen::Vector4s q1_vec(origin_KF->getOPtr()->getVector());
    Eigen::Map<Quaternions> q1(q1_vec.data());
    Eigen::Vector3s v1(origin_KF->getVPtr()->getVector());
    Eigen::Vector3s ab1(origin_KF->getAccBiasPtr()->getVector());
    Eigen::Vector3s wb1(origin_KF->getGyroBiasPtr()->getVector());
    Eigen::Vector3s p2(last_KF->getPPtr()->getVector());
    Eigen::Vector4s q2_vec(last_KF->getOPtr()->getVector());
    Eigen::Map<Quaternions> q2(q2_vec.data());
    Eigen::Vector3s v2(last_KF->getVPtr()->getVector());
    Eigen::Vector3s ab2(last_KF->getAccBiasPtr()->getVector());
    Eigen::Vector3s wb2(last_KF->getGyroBiasPtr()->getVector());

    for(auto ctr_ptr : origin_KF_constraints)
    {
        if (ctr_ptr->getTypeId() == wolf::CTR_IMU)
            {
                std::static_pointer_cast<ConstraintIMU>(ctr_ptr)->getResiduals(p1, q1, v1, ab1, wb1, p2, q2, v2, ab2, wb2, residuals);
                WOLF_DEBUG("ConstraintIMU residuals : \n", residuals.transpose())
            }
    }
}

TEST_F(ConstraintIMU_PrcImuOdom, acc_gyro_biased_static)
{
    //===================================================== PROCESS DATA
    // PROCESS IMU DATA

    Eigen::Vector6s data;
    Eigen::Vector6s bias((Eigen::Vector6s()<< 0.03, 0.2, 0.007, 0.27, 0.52, 0.08).finished());
    data << 0.00, 0.000, -wolf::gravity()(2), 0.0, 0.0, 0.0;
    data += bias;
    Scalar dt = t.get();
    TimeStamp ts(0.001);
    wolf::CaptureIMUPtr imu_ptr = std::make_shared<CaptureIMU>(ts, sen_imu, data);

    while( (dt-t.get()) <= (std::static_pointer_cast<ProcessorIMU>(processor_ptr_)->getMaxTimeSpan()) ){
        
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

    FrameIMUPtr last_KF = std::static_pointer_cast<FrameIMU>(wolf_problem_ptr_->getTrajectoryPtr()->closestKeyFrameToTimeStamp(ts));

    origin_KF->unfix();
    origin_KF->getPPtr()->fix();
    origin_KF->getOPtr()->fix();
    origin_KF->getVPtr()->fix();

    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
    std::cout << summary.BriefReport() << std::endl;

    last_KF->getAccBiasPtr()->fix();
    last_KF->getGyroBiasPtr()->fix();

    wolf_problem_ptr_->print(4,1,1,1);

    ceres_manager_wolf_diff->computeCovariances(ALL);//ALL_MARGINALS, ALL
    //===================================================== END{PROCESS DATA}

    ASSERT_TRUE((last_KF->getPPtr()->getVector() - (Eigen::Vector3s()<< 0, 0, 0).finished()).isMuchSmallerThan(1, wolf::Constants::EPS ));
    EXPECT_TRUE((last_KF->getAccBiasPtr()->getVector() - bias.head(3)).isMuchSmallerThan(1, wolf::Constants::EPS )) << "last_KF Acc bias : " << last_KF->getAccBiasPtr()->getVector().transpose() << 
    "\n introduced Acc bias : " << bias.head(3).transpose() << std::endl;
    ASSERT_TRUE((last_KF->getGyroBiasPtr()->getVector() - bias.tail(3)).isMuchSmallerThan(1, wolf::Constants::EPS )) << "last_KF Gyro bias : " << last_KF->getGyroBiasPtr()->getVector().transpose() << 
    "\n introduced Gyro bias : " << bias.tail(3).transpose() << std::endl;
    
    ConstraintBaseList origin_KF_constraints = origin_KF->getConstrainedByList();
    Eigen::Matrix<wolf::Scalar,15,1> residuals;

    Eigen::Vector3s p1(origin_KF->getPPtr()->getVector());
    Eigen::Vector4s q1_vec(origin_KF->getOPtr()->getVector());
    Eigen::Map<Quaternions> q1(q1_vec.data());
    Eigen::Vector3s v1(origin_KF->getVPtr()->getVector());
    Eigen::Vector3s ab1(origin_KF->getAccBiasPtr()->getVector());
    Eigen::Vector3s wb1(origin_KF->getGyroBiasPtr()->getVector());
    Eigen::Vector3s p2(last_KF->getPPtr()->getVector());
    Eigen::Vector4s q2_vec(last_KF->getOPtr()->getVector());
    Eigen::Map<Quaternions> q2(q2_vec.data());
    Eigen::Vector3s v2(last_KF->getVPtr()->getVector());
    Eigen::Vector3s ab2(last_KF->getAccBiasPtr()->getVector());
    Eigen::Vector3s wb2(last_KF->getGyroBiasPtr()->getVector());

    for(auto ctr_ptr : origin_KF_constraints)
    {
        if (ctr_ptr->getTypeId() == wolf::CTR_IMU)
            {
                std::static_pointer_cast<ConstraintIMU>(ctr_ptr)->getResiduals(p1, q1, v1, ab1, wb1, p2, q2, v2, ab2, wb2, residuals);
                WOLF_DEBUG("ConstraintIMU residuals : \n", residuals.transpose()) 
            }
    }
}

TEST_F(ConstraintIMU_accBiasObservation, acc_gyro_bias_observation)
{
    origin_KF->unfix();
    origin_KF->getPPtr()->fix();
    origin_KF->getOPtr()->fix();
    //origin_KF->getVPtr()->fix();

    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
    std::cout << summary.BriefReport() << std::endl;

    last_KF->getAccBiasPtr()->fix();
    last_KF->getGyroBiasPtr()->fix();

    wolf_problem_ptr_->print(4,1,1,1);

    ceres_manager_wolf_diff->computeCovariances(ALL);//ALL_MARGINALS, ALL
    //===================================================== END{PROCESS DATA}

    EXPECT_TRUE( (expected_final_state.head(3) - last_KF->getPPtr()->getVector()).isMuchSmallerThan(1,wolf::Constants::EPS*10 ) ) <<
    "expected_final_state position : " << expected_final_state.head(3).transpose() << "\n last_KF position : " << last_KF->getPPtr()->getVector().transpose() << std::endl;
    EXPECT_TRUE( (expected_final_state.segment(3,4) - last_KF->getOPtr()->getVector()).isMuchSmallerThan(1,wolf::Constants::EPS ) ) <<
    "expected_final_state quaternion : " << expected_final_state.segment(3,4).transpose() << "\n last_KF quaternion : " << last_KF->getOPtr()->getVector().transpose() << std::endl;
    EXPECT_TRUE( (expected_final_state.tail(3) - last_KF->getVPtr()->getVector()).isMuchSmallerThan(1,wolf::Constants::EPS*10) ) <<
    "expected_final_state velocity : " << expected_final_state.tail(3).transpose() << "\n last_KF velocity : " << last_KF->getVPtr()->getVector().transpose() << std::endl;
    EXPECT_TRUE( (origin_bias.head(3) - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1,wolf::Constants::EPS ) ) <<
    "origin_bias Acc bias : " << origin_bias.head(3).transpose() << "\n origin_KF Acc Bias : " << origin_KF->getAccBiasPtr()->getVector().transpose() << std::endl;
    EXPECT_TRUE( (origin_bias.tail(3) - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1,wolf::Constants::EPS ) ) <<
    "origin_bias gyro bias : " << origin_bias.tail(3).transpose() << "\n origin_KF Gyro Bias : " << origin_KF->getGyroBiasPtr()->getVector().transpose() << std::endl;
    
    ConstraintBaseList origin_KF_constraints = origin_KF->getConstrainedByList();
    Eigen::Matrix<wolf::Scalar,15,1> residuals;

    Eigen::Vector3s p1(origin_KF->getPPtr()->getVector());
    Eigen::Vector4s q1_vec(origin_KF->getOPtr()->getVector());
    Eigen::Map<Quaternions> q1(q1_vec.data());
    Eigen::Vector3s v1(origin_KF->getVPtr()->getVector());
    Eigen::Vector3s ab1(origin_KF->getAccBiasPtr()->getVector());
    Eigen::Vector3s wb1(origin_KF->getGyroBiasPtr()->getVector());
    Eigen::Vector3s p2(last_KF->getPPtr()->getVector());
    Eigen::Vector4s q2_vec(last_KF->getOPtr()->getVector());
    Eigen::Map<Quaternions> q2(q2_vec.data());
    Eigen::Vector3s v2(last_KF->getVPtr()->getVector());
    Eigen::Vector3s ab2(last_KF->getAccBiasPtr()->getVector());
    Eigen::Vector3s wb2(last_KF->getGyroBiasPtr()->getVector());

    for(auto ctr_ptr : origin_KF_constraints)
    {
        if (ctr_ptr->getTypeId() == wolf::CTR_IMU)
            {
                std::static_pointer_cast<ConstraintIMU>(ctr_ptr)->getResiduals(p1, q1, v1, ab1, wb1, p2, q2, v2, ab2, wb2, residuals);
                WOLF_DEBUG("ConstraintIMU residuals : \n", residuals.transpose()) 
            }
    }

    Eigen::MatrixXs cov3(Eigen::Matrix3s::Zero());
    wolf_problem_ptr_->getCovarianceBlock(last_KF->getVPtr(), last_KF->getVPtr(), cov3);
    std::cout << "\n last_KF velocity covariance : \n" << cov3 << std::endl;
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  //::testing::GTEST_FLAG(filter) = "ConstraintIMU_PrcImuOdom.gyro_biased_static";
  ::testing::GTEST_FLAG(filter) = "ConstraintIMU_accBiasObservation*";
  return RUN_ALL_TESTS();
}

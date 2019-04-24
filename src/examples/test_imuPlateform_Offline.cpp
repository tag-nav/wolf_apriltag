//Wolf
#include "base/capture/capture_IMU.h"
#include "base/processor/processor_IMU.h"
#include "base/sensor/sensor_IMU.h"
#include "base/common/wolf.h"
#include "base/problem/problem.h"
#include "base/sensor/sensor_odom_3D.h"
#include "base/factor/factor_odom_3D.h"
#include "base/state_block/state_block.h"
#include "base/state_block/state_quaternion.h"
#include "base/processor/processor_odom_3D.h"
#include "base/ceres_wrapper/ceres_manager.h"

//std
#include <iostream>
#include <fstream>
#include <iomanip>
#include <ctime>
#include <cmath>

#define DEBUG_RESULTS

int _kbhit();

int main(int argc, char** argv)
{
    using namespace wolf;
    using std::shared_ptr;
    using std::make_shared;
    using std::static_pointer_cast;

    // LOADING DATA FILE (IMU)
    // FOR IMU, file content is : Timestampt\t Ax\t Ay\t Az\t Wx\t Wy\t Wz
    
    std::ifstream imu_data_input;
    const char * filename_imu;
    if (argc < 02)
    {
        
        WOLF_ERROR("Missing input argument! : needs 1 argument (path to imu data file).")
        return 1;
    }
    else
    {
        filename_imu = argv[1];

        imu_data_input.open(filename_imu);
        WOLF_INFO("imu file : ", filename_imu)

        //std::string dummy; //this is needed only if first line is headers or useless data
        //getline(imu_data_input, dummy);
    }

    if(!imu_data_input.is_open()){
        std::cerr << "Failed to open data files... Exiting" << std::endl;
        return 1;
    }

    #ifdef DEBUG_RESULTS
    std::ofstream debug_results;
    debug_results.open("debug_results_imu_constrained0.dat");
    if(debug_results)
        debug_results   << "%%TimeStamp\t"
                        << "X_x\t" << "X_y\t" << "X_z\t" << "Xq_x\t" << "Xq_y\t" << "Xq_z\t" << "Xq_w\t" << "Xv_x\t" << "Xv_y\t" << "Xv_z\t"
                        << "Cov_X\t" << "Cov_Y\t" << "Cov_Z\t" << "Cov_Qx\t" << "Cov_Qy\t" << "Cov_Qz\t" << "Cov_Qw" << "Cov_Vx\t" << "Cov_Vy\t" << "Cov_Vz\t" << std::endl;
    #endif

    //===================================================== SETTING PROBLEM
    std::string wolf_root = _WOLF_ROOT_DIR;
        
    // WOLF PROBLEM
    ProblemPtr wolf_problem_ptr_ = Problem::create("PQVBB 3D");
    Eigen::VectorXs x_origin(16);
    x_origin << 0,0,0,  0,0,0,1,  0,0,0,  0,0,0,  0,0,0; //INITIAL CONDITIONS
    TimeStamp t(0);

    // CERES WRAPPER
    ceres::Solver::Options ceres_options;
    ceres_options.minimizer_type = ceres::TRUST_REGION; //ceres::TRUST_REGION;ceres::LINE_SEARCH
    ceres_options.max_line_search_step_contraction = 1e-3;
    ceres_options.max_num_iterations = 1e4;
    CeresManager* ceres_manager_wolf_diff = new CeresManager(wolf_problem_ptr_, ceres_options);

    // SENSOR + PROCESSOR IMU
    SensorBasePtr sen0_ptr = wolf_problem_ptr_->installSensor("IMU", "Main IMU", (Vector7s()<<0,0,0,0,0,0,1).finished(), wolf_root + "/src/examples/sensor_imu.yaml");
    ProcessorIMUParamsPtr prc_imu_params = std::make_shared<ProcessorParamsIMU>();
    prc_imu_params->max_time_span = 10;
    prc_imu_params->max_buff_length = 1000000000; //make it very high so that this condition will not pass
    prc_imu_params->dist_traveled = 1000000000;
    prc_imu_params->angle_turned = 1000000000;
    
    ProcessorBasePtr processor_ptr_ = wolf_problem_ptr_->installProcessor("IMU", "IMU pre-integrator", sen0_ptr, prc_imu_params);
    SensorIMUPtr sen_imu = std::static_pointer_cast<SensorIMU>(sen0_ptr);
    ProcessorIMUPtr processor_ptr_imu = std::static_pointer_cast<ProcessorIMU>(processor_ptr_);

    // SENSOR + PROCESSOR ODOM 3D
    SensorBasePtr sen1_ptr = wolf_problem_ptr_->installSensor("ODOM 3D", "odom", (Vector7s()<<0,0,0,0,0,0,1).finished(), wolf_root + "/src/examples/sensor_odom_3D.yaml");
    ProcessorParamsOdom3DPtr prc_odom3D_params = std::make_shared<ProcessorParamsOdom3D>();
    prc_odom3D_params->max_time_span = 1.9999;
    prc_odom3D_params->max_buff_length = 1000000000; //make it very high so that this condition will not pass
    prc_odom3D_params->dist_traveled = 1000000000;
    prc_odom3D_params->angle_turned = 1000000000;

    ProcessorBasePtr processor_ptr_odom = wolf_problem_ptr_->installProcessor("ODOM 3D", "odom", sen1_ptr, prc_odom3D_params);
    SensorOdom3DPtr sen_odom3D = std::static_pointer_cast<SensorOdom3D>(sen1_ptr);
    ProcessorOdom3DPtr processor_ptr_odom3D = std::static_pointer_cast<ProcessorOdom3D>(processor_ptr_odom);

    // reset origin of problem
    t.set(0);
    
    FrameIMUPtr origin_KF = std::static_pointer_cast<FrameIMU>(processor_ptr_imu->setOrigin(x_origin, t));
    processor_ptr_odom3D->setOrigin(origin_KF);

    //fix parts of the problem if needed
    origin_KF->getP()->fix();
    origin_KF->getO()->fix();
    //===================================================== PROCESS DATA
    // PROCESS DATA

    Eigen::Vector6s data_imu, data_odom3D;
    data_imu << 0,0,-wolf::gravity()(2), 0,0,0;
    data_odom3D << 0,-0.06,0, 0,0,0;

    Scalar input_clock;
    TimeStamp ts(0);
    wolf::CaptureIMUPtr imu_ptr = std::make_shared<CaptureIMU>(ts, sen_imu, data_imu, Matrix6s::Identity(), Vector6s::Zero());
    wolf::CaptureMotionPtr mot_ptr = std::make_shared<CaptureMotion>(t, sen_odom3D, data_odom3D, 6, 6, nullptr);

    //when we find a IMU timestamp corresponding with this odometry timestamp then we process odometry measurement
    t = ts;

    clock_t begin = clock();
    
    while( !imu_data_input.eof())
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
    
    TimeStamp t0, tf;
    t0 = wolf_problem_ptr_->getProcessorMotion()->getBuffer().get().front().ts_;
    tf = wolf_problem_ptr_->getProcessorMotion()->getBuffer().get().back().ts_;
    int N = wolf_problem_ptr_->getProcessorMotion()->getBuffer().get().size();

    //Finally, process the only one odom input
    mot_ptr->setTimeStamp(ts);
    mot_ptr->setData(data_odom3D);
    sen_odom3D->process(mot_ptr);

    clock_t end = clock();
    FrameIMUPtr last_KF = std::static_pointer_cast<FrameIMU>(wolf_problem_ptr_->getTrajectory()->closestKeyFrameToTimeStamp(ts));

    //closing file
    imu_data_input.close();
    //===================================================== END{PROCESS DATA}

    double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;

    // Final state
    std::cout << "\nIntegration results ----------------------------------------------------------------------------------------------" << std::endl;
    std::cout << "Initial    state: " << std::fixed << std::setprecision(3) << std::setw(8)
    << x_origin.head(16).transpose() << std::endl;
    std::cout << "Integrated delta: " << std::fixed << std::setprecision(3) << std::setw(8)
    << wolf_problem_ptr_->getProcessorMotion()->getMotion().delta_integr_.transpose() << std::endl;
    std::cout << "Integrated state: " << std::fixed << std::setprecision(3) << std::setw(8)
    << wolf_problem_ptr_->getProcessorMotion()->getCurrentState().head(16).transpose() << std::endl;
    std::cout << "Integrated std  : " << std::fixed << std::setprecision(3) << std::setw(8)
    << (wolf_problem_ptr_->getProcessorMotion()->getMotion().delta_integr_cov_.diagonal().transpose()).array().sqrt() << std::endl;

    // Print statistics
    std::cout << "\nStatistics -----------------------------------------------------------------------------------" << std::endl;
    std::cout << "If you want meaningful CPU metrics, remove all couts in the loop / remove DEBUG_RESULTS definition variable, and compile in RELEASE mode!" << std::endl;

    /*TimeStamp t0, tf;
    t0 = wolf_problem_ptr_->getProcessorMotion()->getBuffer().get().front().ts_;
    tf = wolf_problem_ptr_->getProcessorMotion()->getBuffer().get().back().ts_;
    int N = wolf_problem_ptr_->getProcessorMotion()->getBuffer().get().size();*/
    std::cout << "t0        : " << t0.get() << " s" << std::endl;
    std::cout << "tf        : " << tf.get() << " s" << std::endl;
    std::cout << "duration  : " << tf-t0 << " s" << std::endl;
    std::cout << "N samples : " << N << std::endl;
    std::cout << "frequency : " << (N-1)/(tf-t0) << " Hz" << std::endl;
    std::cout << "CPU time  : " << elapsed_secs << " s" << std::endl;
    std::cout << "s/integr  : " << elapsed_secs/(N-1)*1e6 << " us" << std::endl;
    std::cout << "integr/s  : " << (N-1)/elapsed_secs << " ips" << std::endl;

    //fix parts of the problem if needed
    origin_KF->getP()->fix();
    origin_KF->getO()->fix();
    origin_KF->getV()->fix();
    
    std::cout << "\t\t\t ______solving______" << std::endl;
    std::string report = ceres_manager_wolf_diff->solve(SolverManager::ReportVerbosity::FULL);// 0: nothing, 1: BriefReport, 2: FullReport
    std::cout << report << std::endl;
    ceres_manager_wolf_diff->computeCovariances(SolverManager::CovarianceBlocksToBeComputed::ALL);
    std::cout << "\t\t\t ______solved______" << std::endl;

    wolf_problem_ptr_->print(4,1,1,1);

    #ifdef DEBUG_RESULTS
    Eigen::VectorXs frm_state(16);
    Eigen::Matrix<wolf::Scalar, 16, 1> cov_stdev;
    Eigen::MatrixXs covX(16,16);
    Eigen::MatrixXs cov3(Eigen::Matrix3s::Zero());

    wolf::FrameBasePtrList frame_list = wolf_problem_ptr_->getTrajectory()->getFrameList();
    for(FrameBasePtr frm_ptr : frame_list)
    {
        if(frm_ptr->isKey())
        {   
            //prepare needed variables
            FrameIMUPtr frmIMU_ptr = std::static_pointer_cast<FrameIMU>(frm_ptr);
            frm_state = frmIMU_ptr->getState();
            ts = frmIMU_ptr->getTimeStamp();

            //get data from covariance blocks
            wolf_problem_ptr_->getFrameCovariance(frmIMU_ptr, covX);
            wolf_problem_ptr_->getCovarianceBlock(frmIMU_ptr->getV(), frmIMU_ptr->getV(), cov3);
            covX.block(7,7,3,3) = cov3;
            wolf_problem_ptr_->getCovarianceBlock(frmIMU_ptr->getAccBias(), frmIMU_ptr->getAccBias(), cov3);
            covX.block(10,10,3,3) = cov3;
            wolf_problem_ptr_->getCovarianceBlock(frmIMU_ptr->getGyroBias(), frmIMU_ptr->getGyroBias(), cov3);
            covX.block(13,13,3,3) = cov3;
            for(int i = 0; i<16; i++)
                cov_stdev(i) = ( covX(i,i)? 2*sqrt(covX(i,i)):0); //if diagonal value is 0 then store 0 else store 2*sqrt(diag_value)

            debug_results << std::setprecision(16) << ts.get() << "\t" << frm_state(0) << "\t" << frm_state(1) << "\t" << frm_state(2)
            << "\t" << frm_state(3) << "\t" << frm_state(4) << "\t" << frm_state(5) << "\t" << frm_state(6)
            << "\t" << frm_state(7) << "\t" << frm_state(8) << "\t" << frm_state(9)
            << "\t" << frm_state(10) << "\t" << frm_state(11) << "\t" << frm_state(12) << "\t" << frm_state(13) << "\t" << frm_state(14) << "\t" << frm_state(15)
            << "\t" << cov_stdev(0) << "\t" << cov_stdev(1) << "\t" << cov_stdev(2)
            << "\t" << cov_stdev(3) << "\t" << cov_stdev(4) << "\t" << cov_stdev(5) << "\t" << cov_stdev(6)
            << "\t" << cov_stdev(7) << "\t" << cov_stdev(8) << "\t" << cov_stdev(9)
            << "\t" << cov_stdev(10) << "\t" << cov_stdev(11) << "\t" << cov_stdev(12) << "\t" << cov_stdev(13) << "\t" << cov_stdev(14) << "\t" << cov_stdev(15) << std::endl;
        }
    }

    debug_results.close();
    WOLF_WARN("WARNING : DEBUG_RESULTS ACTIVATED - slows the process (writing results to result_debugs.dat file)")

    #endif

    return 0;

}

int _kbhit()
{
    struct timeval tv;
    fd_set fds;
    tv.tv_sec = 0;
    tv.tv_usec = 0;
    FD_ZERO(&fds);
    FD_SET(STDIN_FILENO, &fds); //STDIN_FILENO is 0
    select(STDIN_FILENO+1, &fds, NULL, NULL, &tv);
    return FD_ISSET(STDIN_FILENO, &fds);
}

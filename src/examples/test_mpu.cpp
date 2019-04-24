/**
 * \file test_mpu.cpp
 *
 *  Created on: Oct 4, 2016
 *      \author: AtDinesh
 */

 //Wolf
#include "base/capture/capture_IMU.h"
#include "base/common/wolf.h"
#include "base/problem/problem.h"
#include "base/state_block/state_block.h"
#include "base/state_block/state_quaternion.h"
#include <iostream>
#include <fstream>
#include <iomanip>
#include <ctime>
#include <cmath>
#include <termios.h>
#include <fcntl.h>
#include "base/processor/processor_IMU.h"
#include "base/sensor/sensor_IMU.h"

#define DEBUG_RESULTS
#define FROM_FILE

int _kbhit();

int main(int argc, char** argv)
{
    using namespace wolf;

    #ifdef FROM_FILE
        std::ifstream data_file;
        const char * filename;

        if (argc < 2)
        {
            std::cout << "Missing input argument! : needs 1 argument (path to data file)." << std::endl;
            return 1;
        }
        else
        {
            filename = argv[1];
            data_file.open(filename);
            std::cout << "file: " << filename << std::endl;

            std::string dummy;
            getline(data_file, dummy);

        if(!data_file.is_open()){
            std::cerr << "Failed to open data files... Exiting" << std::endl;
            return 1;
        }
    }
    #else
    int fd,n;
    ///prepare MPU here
    if (argc < 2)
    {
        std::cout << "Missing input argument! : needs 1 argument : way to MPU device. (usually /dev/ttyACM#)\n Please make sure that you have rights to access the device and that your user belongs to the dialout group." << std::endl;
        return 1;
    }
    unsigned char buf[64] = {0};
	wolf::Scalar gravity = 9.81;
	wolf::Scalar sec_to_rad = 3.14159265359/180.0;
	wolf::Scalar accel_LSB = 1.0/8192.0; // = 4.0/32768.0
	wolf::Scalar gyro_LSB = 1.0/131.0; // = 250.0/32768.0
    wolf::Scalar accel_LSB_g = accel_LSB * gravity;
	wolf::Scalar gyro_LSB_rad = gyro_LSB * sec_to_rad;
    //wolf::Scalar Ax, Ay, Az, Gx, Gy, Gz;

    struct termios toptions;
    //open serial port
    std::cout << "open port...\n" << std::endl;
    fd = open(argv[1], O_RDWR | O_NOCTTY);
    if (fd != -1)
        std::cout << "MPU openned successfully! \n" << std::endl;
    else
        std::cout << "MPU could not be openned... \n" << std::endl;

    //configuring termios
    tcgetattr(fd, &toptions);
    cfsetispeed(&toptions, B1000000);
    cfsetospeed(&toptions, B1000000);
    toptions.c_cflag     |= (CLOCAL | CREAD);
    toptions.c_lflag     &= ~(ICANON | ECHO | ECHOE | ISIG);
    toptions.c_oflag     &= ~OPOST;
    toptions.c_cc[VMIN]  = 0;
    toptions.c_cc[VTIME] = 10;
    tcsetattr(fd, TCSANOW, &toptions);
    #endif

    #ifdef DEBUG_RESULTS
    std::ofstream debug_results;
    debug_results.open("debug_results.dat");
    if(debug_results)
        debug_results << "%%TimeStamp\t"
                      << "dp_x\t" << "dp_y\t" << "dp_z\t" << "dv_x\t" << "dv_y\t" << "dv_z\t" << "dq_x\t" << "dq_y\t" << "dq_z\t" << "dq_w\t"
                      << "Dp_x\t" << "Dp_y\t" << "Dp_z\t" << "Dv_x\t" << "Dv_y\t" << "Dv_z\t" << "Dq_x\t" << "Dq_y\t" << "Dq_z\t" << "Dq_w\t"
                      << "X_x\t" << "X_y\t" << "X_z\t" << "Xv_x\t" << "Xv_y\t" << "Xv_z\t" << "Xq_x\t" << "Xq_y\t" << "Xq_z\t" << "Xq_w\t" << std::endl;
    #endif

    // Wolf problem
    ProblemPtr wolf_problem_ptr_ = Problem::create("PQVBB 3D");
    Eigen::VectorXs IMU_extrinsics(7);
    IMU_extrinsics << 0,0,0, 0,0,0,1; // IMU pose in the robot
    SensorBasePtr sensor_ptr = wolf_problem_ptr_->installSensor("IMU", "Main IMU", IMU_extrinsics, IntrinsicsBase());
    wolf_problem_ptr_->installProcessor("IMU", "IMU pre-integrator", "Main IMU", "");

    // Time and data variables
    TimeStamp t;
    Eigen::Vector6s data_;
    Scalar mpu_clock = 0;

    t.set(mpu_clock * 0.0001); // clock in 0,1 ms ticks

    // Set the origin
    Eigen::VectorXs x0(16);
    x0 << 0,0,0,  0,0,0,  1,0,0,0,  0,0,.001,  0,0,.002; // Try some non-zero biases
    wolf_problem_ptr_->getProcessorMotion()->setOrigin(x0, t);

    // Create one capture to store the IMU data arriving from (sensor / callback / file / etc.)
    CaptureIMUPtr imu_ptr( std::make_shared<CaptureIMU>(t, sensor_ptr, data_) );

    // main loop
    using namespace std;
    clock_t begin = clock();
    std::cout << "\n\t\t\t\tENTERING MAIN LOOP - Please press ENTER to exit loop\n" << std::endl;

    #ifdef FROM_FILE
    while(!data_file.eof()){
        // read new data
        data_file >> mpu_clock >> data_[0] >> data_[1] >> data_[2] >> data_[3] >> data_[4] >> data_[5];
        t.set(mpu_clock); //
    #else
    while(!_kbhit()){
        // read new data
        do n = read(fd, buf, 1);//READ IT
        while (buf[0]!=0x47); //control character has been found
        n = read(fd, buf, 12);//read the data
        if (n>3){ //construct data_ from IMU input
			data_(0)   = (wolf::Scalar)((int16_t)((buf[1]<<8)|buf[0]))*accel_LSB_g;
			data_(1)   = (wolf::Scalar)((int16_t)((buf[3]<<8)|buf[2]))*accel_LSB_g;
			data_(2)   = (wolf::Scalar)((int16_t)((buf[5]<<8)|buf[4]))*accel_LSB_g;
			data_(3)   = (wolf::Scalar)((int16_t)((buf[7]<<8)|buf[6]))*gyro_LSB_rad;
			data_(4)   = (wolf::Scalar)((int16_t)((buf[9]<<8)|buf[8]))*gyro_LSB_rad;
			data_(5)   = (wolf::Scalar)((int16_t)((buf[11]<<8)|buf[10]))*gyro_LSB_rad;
            mpu_clock += 0.001;
            t.set(mpu_clock);
        }
        #endif

        // assign data to capture
        imu_ptr->setData(data_);
        imu_ptr->setTimeStamp(t);

        // process data in capture
        sensor_ptr->process(imu_ptr);

        #ifdef DEBUG_RESULTS

        Eigen::VectorXs delta_debug;
        Eigen::VectorXs delta_integr_debug;
        Eigen::VectorXs x_debug;
        TimeStamp ts;

        delta_debug = wolf_problem_ptr_->getProcessorMotion()->getMotion().delta_;
        delta_integr_debug = wolf_problem_ptr_->getProcessorMotion()->getMotion().delta_integr_;
        x_debug = wolf_problem_ptr_->getProcessorMotion()->getCurrentState();
        ts = wolf_problem_ptr_->getProcessorMotion()->getBuffer().get().back().ts_;

        if(debug_results)
            debug_results << ts.get() << "\t" << delta_debug(0) << "\t" << delta_debug(1) << "\t" << delta_debug(2) << "\t" << delta_debug(3) << "\t" << delta_debug(4) << "\t"
            << delta_debug(5) << "\t" << delta_debug(6) << "\t" << delta_debug(7) << "\t" << delta_debug(8) << "\t" << delta_debug(9) << "\t"
            << delta_integr_debug(0) << "\t" << delta_integr_debug(1) << "\t" << delta_integr_debug(2) << "\t" << delta_integr_debug(3) << "\t" << delta_integr_debug(4) << "\t"
            << delta_integr_debug(5) << "\t" << delta_integr_debug(6) << "\t" << delta_integr_debug(7) << "\t" << delta_integr_debug(8) << "\t" << delta_integr_debug(9) << "\t"
            << x_debug(0) << "\t" << x_debug(1) << "\t" << x_debug(2) << "\t" << x_debug(3) << "\t" << x_debug(4) << "\t"
            << x_debug(5) << "\t" << x_debug(6) << "\t" << x_debug(7) << "\t" << x_debug(8) << "\t" << x_debug(9) << "\n";
        #endif
    }

    clock_t end = clock();
    double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;

    // Final state
    std::cout << "\nIntegration results ----------------------------------------------------------------------------------------------" << std::endl;
    std::cout << "Initial    state: " << std::fixed << std::setprecision(3) << std::setw(8)
    << x0.head(16).transpose() << std::endl;
    std::cout << "Integrated delta: " << std::fixed << std::setprecision(3) << std::setw(8)
    << wolf_problem_ptr_->getProcessorMotion()->getMotion().delta_integr_.transpose() << std::endl;
    std::cout << "Integrated state: " << std::fixed << std::setprecision(3) << std::setw(8)
    << wolf_problem_ptr_->getProcessorMotion()->getCurrentState().head(16).transpose() << std::endl;
    std::cout << "Integrated std  : " << std::fixed << std::setprecision(3) << std::setw(8)
    << (wolf_problem_ptr_->getProcessorMotion()->getMotion().delta_integr_cov_.diagonal().transpose()).array().sqrt() << std::endl;

    // Print statistics
    std::cout << "\nStatistics -----------------------------------------------------------------------------------" << std::endl;
    std::cout << "If you want meaningful CPU metrics, remove all couts in the loop / remove DEBUG_RESULTS definition variable, and compile in RELEASE mode!" << std::endl;

#ifdef DEBUG_RESULTS
    std::cout << "\t\tWARNING : DEBUG_RESULTS ACTIVATED - slows the process (writing results to result_debugs.dat file)" << std::endl;
    debug_results.close();
#endif

    TimeStamp t0, tf;
    t0 = wolf_problem_ptr_->getProcessorMotion()->getBuffer().get().front().ts_;
    tf = wolf_problem_ptr_->getProcessorMotion()->getBuffer().get().back().ts_;
    int N = wolf_problem_ptr_->getProcessorMotion()->getBuffer().get().size();
    std::cout << "t0        : " << t0.get() << " s" << std::endl;
    std::cout << "tf        : " << tf.get() << " s" << std::endl;
    std::cout << "duration  : " << tf-t0 << " s" << std::endl;
    std::cout << "N samples : " << N << std::endl;
    std::cout << "frequency : " << (N-1)/(tf-t0) << " Hz" << std::endl;
    std::cout << "CPU time  : " << elapsed_secs << " s" << std::endl;
    std::cout << "s/integr  : " << elapsed_secs/(N-1)*1e6 << " us" << std::endl;
    std::cout << "integr/s  : " << (N-1)/elapsed_secs << " ips" << std::endl;

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

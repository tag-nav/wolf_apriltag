/**
 * \file test_diff_drive.cpp
 *
 *  Created on: Oct 26, 2017
 *  \author: Jeremie Deray
 */

//Wolf
#include "core/common/wolf.h"
#include "core/problem/problem.h"
#include "core/sensor/sensor_diff_drive.h"
#include "core/capture/capture_wheel_joint_position.h"
#include "core/processor/processor_diff_drive.h"

//std
#include <iostream>
#include <fstream>
#include <iomanip>
#include <ctime>
#include <cmath>

//#define DEBUG_RESULTS

void getOdom2DData(std::ifstream& _stream, wolf::Scalar& _stamp, Eigen::Vector2s& _data)
{
  /*
   * Data are logged as follows :
   *
   *  header:
   *    seq: xxx
   *    stamp:
   *      secs: xxx
   *      nsecs: xxx
   *    frame_id: ''
   *  twist:
   *    linear:
   *      x: 0.0
   *      y: 0.0
   *      z: 0.0
   *    angular:
   *      x: 0.0
   *      y: 0.0
   *      z: 0.0
   * ---
   */

  std::string dummy;

  getline(_stream, dummy); // header:
  getline(_stream, dummy); // seq: xxx
  getline(_stream, dummy); // stamp:
  getline(_stream, dummy); // secs: xxx

  // Find secs
  std::string sub("secs: ");
  std::string::size_type i = dummy.find(sub);
  dummy.erase(i, sub.length());

  _stamp = std::stod(dummy);

  // Find nsecs
  getline(_stream, dummy); // nsecs: xxx
  sub = "nsecs: ";
  i = dummy.find(sub);
  dummy.erase(i, sub.length());

  _stamp += std::stod(dummy) * wolf::Scalar(1e-9);

  getline(_stream, dummy); // frame_id: ''
  getline(_stream, dummy); // twist:
  getline(_stream, dummy); // linear:
  getline(_stream, dummy); // x: 0.0

  sub = "x: ";
  i = dummy.find(sub);
  dummy.erase(i, sub.length());

  _data(0) = std::stod(dummy);

  getline(_stream, dummy); // y: 0.0
  getline(_stream, dummy); // z: 0.0
  getline(_stream, dummy); // angular:
  getline(_stream, dummy); // x: 0.0
  getline(_stream, dummy); // y: 0.0
  getline(_stream, dummy); // z: 0.0

  sub = "z: ";
  i = dummy.find(sub);
  dummy.erase(i, sub.length());

  _data(1) = std::stod(dummy);

  getline(_stream, dummy); // ---
}

void readWheelData(std::ifstream &_stream, Eigen::Vector2s &_data)
{
  /*
   * left_wheel_joint_actual_position: [x]
   * right_wheel_joint_actual_position: [x]
   * ---
   */

  std::string dummy;
  std::string l_brac("[");
  std::string r_brac("]");

  getline(_stream, dummy);

  unsigned first = dummy.find(l_brac);
  unsigned last  = dummy.find(r_brac);

  //std::cout << "READING : " << dummy.substr(first+1, last-first-1) << std::endl;

  _data(0) = std::stod(dummy.substr(first+1, last-first-1));

  getline(_stream, dummy);

  first = dummy.find(l_brac);
  last  = dummy.find(r_brac);

  //std::cout << "READING : " << dummy.substr(first+1, last-first-1) << std::endl;

  _data(1) = std::stod(dummy.substr(first+1 , last-first-1));

  getline(_stream, dummy);
}

bool WHEEL_DATA = true;
bool VERBOSE = false;

int main(int argc, char** argv)
{
  using namespace wolf;

  WOLF_INFO("==================== diff drive test ======================");

  //load files containing data
  std::ifstream data_file;
  const char * filename;

  if (argc < 2)
  {
   WOLF_ERROR("Missing input argument! :"
              " needs 2 arguments (path to data file & data type "
              "- velocities or wheel positions).");
    return EXIT_FAILURE;
  }
  else
  {
    filename   = argv[1];
    if (argc >= 3) WHEEL_DATA = std::stoi(argv[2]);

    data_file.open(filename);

    WOLF_INFO("Data file: ", filename);

    if (!data_file.is_open())
    {
      WOLF_ERROR("Failed to open data files... Exiting");
      return EXIT_FAILURE;
    }
  }

  // Wolf problem
  ProblemPtr wolf_problem_ptr_ = Problem::create("PO", 2);

  const std::string sensor_name("Main Odometer");
  Eigen::VectorXs extrinsics(3);
  extrinsics << 0, 0, 0;

  IntrinsicsBasePtr intrinsics = std::make_shared<IntrinsicsDiffDrive>();

  IntrinsicsDiffDrivePtr intrinsics_diff_drive =
      std::static_pointer_cast<IntrinsicsDiffDrive>(intrinsics);

  intrinsics_diff_drive->left_radius_  = 0.1;
  intrinsics_diff_drive->right_radius_ = 0.1;
  intrinsics_diff_drive->separation_   = 0.3517;

  intrinsics_diff_drive->model_        = wolf::DiffDriveModel::Three_Factor_Model;
  intrinsics_diff_drive->factors_      = Eigen::Vector3s(1,1,1);

  intrinsics_diff_drive->left_resolution_  = 0.0001653; // [rad]
  intrinsics_diff_drive->right_resolution_ = 0.0001653; // [rad]

  intrinsics_diff_drive->left_gain_        = 0.01;
  intrinsics_diff_drive->right_gain_       = 0.01;

  // Time and data variables
  TimeStamp t;
  Scalar stamp_secs(0);
//  Scalar period_secs(0.010); //100Hz
  Scalar period_secs(0.020); //50Hz
  Eigen::Vector2s data_; data_ << 0,0;

  const auto scalar_max = std::numeric_limits<Scalar>::max();

  ProcessorParamsDiffDrivePtr processor_params = std::make_shared<ProcessorParamsDiffDrive>();
  processor_params->time_tolerance  = period_secs/2;
  processor_params->angle_turned    = scalar_max;
  processor_params->dist_traveled   = scalar_max;
  processor_params->max_time_span   = scalar_max;
  processor_params->max_buff_length = 999;
  processor_params->unmeasured_perturbation_std = 0.0001;

  SensorBasePtr sensor_ptr =
      wolf_problem_ptr_->installSensor("DIFF DRIVE", sensor_name, extrinsics, intrinsics);

  WOLF_INFO("Sensor 'DIFF DRIVE' installed.");

  auto diff_drive_sensor_ptr = std::static_pointer_cast<SensorDiffDrive>(sensor_ptr);

  wolf_problem_ptr_->installProcessor("DIFF DRIVE", "Diffential Drive processor", sensor_ptr, processor_params);

  WOLF_INFO("Processor 'DIFF DRIVE' installed.");

  // Get initial wheel data
  if (WHEEL_DATA)
    readWheelData(data_file, data_);
  else
    getOdom2DData(data_file, stamp_secs, data_);

  t.set(stamp_secs);
  auto processor_diff_drive_ptr =
      std::static_pointer_cast<ProcessorDiffDrive>(wolf_problem_ptr_->getProcessorMotion());
  processor_diff_drive_ptr->setTimeTolerance(period_secs/2); // overwrite time tolerance based on new evidence

  // Set the origin
  // Create one capture to store the Odometry data.
  std::shared_ptr<CaptureWheelJointPosition> data_ptr =
      std::make_shared<CaptureWheelJointPosition>(t, sensor_ptr, data_, nullptr);

  WOLF_INFO("Process first capture.");

  diff_drive_sensor_ptr->process(data_ptr);

  // main loop
  clock_t begin = clock();

  while (!data_file.eof())
  {
    // read new data
    if (WHEEL_DATA)
    {
      readWheelData(data_file, data_);
      stamp_secs += period_secs;
    }
    else
      getOdom2DData(data_file, stamp_secs, data_);

    t.set(stamp_secs);

    data_ptr = std::make_shared<CaptureWheelJointPosition>(t, sensor_ptr, data_, nullptr);

    // process data in capture
    diff_drive_sensor_ptr->process(data_ptr);

    WOLF_INFO_COND(VERBOSE, "At stamp ", stamp_secs,
                   " state ", processor_diff_drive_ptr->getCurrentState().transpose());
  }

  data_file.close();

  double elapsed_secs = double(clock() - begin) / CLOCKS_PER_SEC;

  // Final state
  WOLF_INFO("----------------------------------------- "
            "Integration results "
            "-----------------------------------------");

  WOLF_INFO("Integrated delta: " , /* std::fixed , std::setprecision(3),*/
            wolf_problem_ptr_->getProcessorMotion()->getMotion().delta_integr_.transpose());
  WOLF_INFO("Integrated state: " , /*std::fixed , std::setprecision(3),*/
            wolf_problem_ptr_->getProcessorMotion()->getCurrentState().transpose());
  WOLF_INFO("Integrated std  : " , /*std::fixed , std::setprecision(3),*/
            (wolf_problem_ptr_->getProcessorMotion()->getMotion().
                delta_integr_cov_.diagonal().transpose()).array().sqrt());

  // Print statistics
  TimeStamp t0, tf;
  t0 = wolf_problem_ptr_->getProcessorMotion()->getBuffer().get().front().ts_;
  tf = wolf_problem_ptr_->getProcessorMotion()->getBuffer().get().back().ts_;

  double N = (double)wolf_problem_ptr_->getProcessorMotion()->getBuffer().get().size();

  WOLF_INFO("t0        : " , t0.get()               , " secs");
  WOLF_INFO("tf        : " , tf.get()               , " secs");
  WOLF_INFO("duration  : " , tf-t0                  , " secs");
  WOLF_INFO("N samples : " , N                               );
  WOLF_INFO("frequency : " , (N-1)/(tf-t0)          , " Hz"  );
  WOLF_INFO("CPU time  : " , elapsed_secs           , " s"   );
  WOLF_INFO("s/integr  : " , elapsed_secs/(N-1)*1e6 , " us"  );
  WOLF_INFO("integr/s  : " , (N-1)/elapsed_secs     , " ips" );

  return 0;
}

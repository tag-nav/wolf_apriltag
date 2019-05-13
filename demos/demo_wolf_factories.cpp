/**
 * \file test_wolf_factories.cpp
 *
 *  Created on: Apr 25, 2016
 *      \author: jsola
 */

#include "core/processor/processor_IMU.h"
#include "core/sensor/sensor_GPS_fix.h"
#include "core/hardware/hardware_base.h"
#include "core/sensor/sensor_camera.h"
#include "core/sensor/sensor_odom_2D.h"
#include "../sensor_imu.h"
//#include "../sensor_gps.h"

#include "core/processor/processor_odom_2D.h"
#include "core/processor/processor_odom_3D.h"
#include "../processor_image_feature.h"

#include "core/problem/problem.h"

#include "core/common/factory.h"

#include <iostream>
#include <iomanip>
#include <cstdlib>

int main(void)
{
    using namespace wolf;
    using namespace std;
    using std::shared_ptr;
    using std::make_shared;
    using std::static_pointer_cast;

    //==============================================================================================
    std::string wolf_root       = _WOLF_ROOT_DIR;
    std::string wolf_config     = wolf_root + "/src/examples";
    std::cout << "\nwolf directory for configuration files: " << wolf_config << std::endl;
    //==============================================================================================

    cout << "\n====== Registering creators in the Wolf Factories =======" << endl;

    cout << "If you look above, you see the registered creators.\n"
            "There is only one attempt per class, and it is successful!\n"
            "We do this by registering in the class\'s .cpp file.\n"
            "\n"
            "- See \'" << wolf_root << "/src/examples/test_wolf_factories.cpp\'\n"
            "  for the way to install sensors and processors to wolf::Problem." << endl;

    // Start creating the problem

    ProblemPtr problem = Problem::create(FRM_PO_3D);

    // define some useful parameters
    Eigen::VectorXs pq_3d(7), po_2d(3), p_3d(3);
    shared_ptr<IntrinsicsOdom2D> intr_odom2d_ptr = nullptr;

    cout << "\n================== Intrinsics Factory ===================" << endl;

    // Use params factory for camera intrinsics
    IntrinsicsBasePtr intr_cam_ptr = IntrinsicsFactory::get().create("CAMERA", wolf_config + "/camera_params_ueye_sim.yaml");
    ProcessorParamsBasePtr params_ptr = ProcessorParamsFactory::get().create("IMAGE FEATURE", wolf_config + "/processor_image_feature.yaml");

    cout << "CAMERA with intrinsics      : " << (static_pointer_cast<IntrinsicsCamera>(intr_cam_ptr))->pinhole_model_raw.transpose() << endl;
//    cout << "Processor IMAGE image width : " << (static_pointer_cast<ProcessorParamsImage>(params_ptr))->image.width << endl;

    cout << "\n==================== Install Sensors ====================" << endl;

    // Install sensors
    problem->installSensor("CAMERA",     "front left camera",    pq_3d,  intr_cam_ptr);
    problem->installSensor("CAMERA",     "front right camera",   pq_3d,  wolf_config + "/camera_params_ueye_sim.yaml");
    problem->installSensor("ODOM 2D",    "main odometer",        po_2d,  intr_odom2d_ptr);
    problem->installSensor("GPS FIX",    "GPS fix",              p_3d);
    problem->installSensor("IMU",        "inertial",             pq_3d);
//    problem->installSensor("GPS",        "GPS raw",              p_3d);
    problem->installSensor("ODOM 2D",    "aux odometer",         po_2d,  intr_odom2d_ptr);
    problem->installSensor("CAMERA", "rear camera", pq_3d, wolf_root + "/src/examples/camera_params_ueye_sim.yaml");

    // print available sensors
    for (auto sen : problem->getHardware()->getSensorList())
    {
        cout << "Sensor " << setw(2) << left << sen->id()
                << " | type " << setw(8) << sen->getType()
                << " | name: " << sen->getName() << endl;
    }

    cout << "\n=================== Install Processors ===================" << endl;

    // Install processors and bind them to sensors -- by sensor name!
    problem->installProcessor("ODOM 2D", "main odometry",    "main odometer");
    problem->installProcessor("ODOM 2D", "sec. odometry",    "aux odometer");
    problem->installProcessor("IMU",     "pre-integrated",   "inertial");
    problem->installProcessor("IMAGE FEATURE",   "ORB",              "front left camera", wolf_config + "/processor_image_feature.yaml");
//    problem->createProcessor("GPS",     "GPS pseudoranges", "GPS raw");

    // print installed processors
    for (auto sen : problem->getHardware()->getSensorList())
        for (auto prc : sen->getProcessorList())
            cout << "Processor " << setw(2) << left  << prc->id()
            << " | type : " << setw(8) << prc->getType()
            << " | name: " << setw(17) << prc->getName()
            << " | bound to sensor " << setw(2) << prc->getSensor()->id() << ": " << prc->getSensor()->getName() << endl;

    return 0;
}


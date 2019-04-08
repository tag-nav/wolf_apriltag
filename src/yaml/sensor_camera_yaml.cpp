/**
 * \file sensor_camera_yaml.cpp
 *
 *  Created on: May 13, 2016
 *      \author: jsola
 */

// wolf yaml
#include "base/yaml/yaml_conversion.h"

// wolf
#include "base/sensor/sensor_camera.h"
#include "base/factory.h"

// yaml-cpp library
#include <yaml-cpp/yaml.h>

namespace wolf
{

namespace
{
static IntrinsicsBasePtr createIntrinsicsCamera(const std::string & _filename_dot_yaml)
{
    YAML::Node camera_config = YAML::LoadFile(_filename_dot_yaml);

    //    if (camera_config["sensor type"].as<std::string>() == "CAMERA") // this does not work: camera YAML files are ROS-styled
    if (camera_config["camera_matrix"]) // check that at least this field exists to validate YAML file of the correct type
    {

        // YAML:: to Eigen::
        using namespace Eigen;
        unsigned int width      = camera_config["image_width"]                      .as<unsigned int>();
        unsigned int height     = camera_config["image_height"]                     .as<unsigned int>();
//<<<<<<< HEAD
//        VectorXd intrinsic      = camera_config["camera_matrix"]["data"]            .as<VectorXd>();
//        VectorXd projection     = camera_config["projection_matrix"]["data"]        .as<VectorXd>();
//=======
//>>>>>>> devel
        VectorXd distortion     = camera_config["distortion_coefficients"]["data"]  .as<VectorXd>();
        VectorXd intrinsic      = camera_config["camera_matrix"]["data"]            .as<VectorXd>();
        VectorXd projection     = camera_config["projection_matrix"]["data"]        .as<VectorXd>();

        // Eigen:: to wolf::
        std::shared_ptr<IntrinsicsCamera> intrinsics_cam = std::make_shared<IntrinsicsCamera>();
//<<<<<<< HEAD
//        intrinsics_cam->type = sensor_type;
//        intrinsics_cam->name = sensor_name;
////        intrinsics_cam->pinhole_model[0] = intrinsic[2];
////        intrinsics_cam->pinhole_model[1] = intrinsic[5];
////        intrinsics_cam->pinhole_model[2] = intrinsic[0];
////        intrinsics_cam->pinhole_model[3] = intrinsic[4];
//        intrinsics_cam->pinhole_model[0] = projection[2]; // u0
//        intrinsics_cam->pinhole_model[1] = projection[6]; // v0
//        intrinsics_cam->pinhole_model[2] = projection[0]; // au
//        intrinsics_cam->pinhole_model[3] = projection[5]; // av
//=======

        intrinsics_cam->width   = width;
        intrinsics_cam->height  = height;

        intrinsics_cam->pinhole_model_raw[0] = intrinsic[2];
        intrinsics_cam->pinhole_model_raw[1] = intrinsic[5];
        intrinsics_cam->pinhole_model_raw[2] = intrinsic[0];
        intrinsics_cam->pinhole_model_raw[3] = intrinsic[4];

        intrinsics_cam->pinhole_model_rectified[0] = projection[2];
        intrinsics_cam->pinhole_model_rectified[1] = projection[6];
        intrinsics_cam->pinhole_model_rectified[2] = projection[0];
        intrinsics_cam->pinhole_model_rectified[3] = projection[5];

//>>>>>>> devel
        assert (distortion.size() == 5 && "Distortion size must be size 5!");

        WOLF_WARN_COND( distortion(2) != 0 || distortion(3) != 0 , "Wolf does not handle tangential distortion. Please consider re-calibrating without tangential distortion!");

        if (distortion(4) == 0)
            if (distortion(1) == 0)
                if (distortion(0) == 0)
                    intrinsics_cam->distortion.resize(0);
                else
                {
                    intrinsics_cam->distortion.resize(1);
                    intrinsics_cam->distortion = distortion.head<1>();
                }
            else
            {
                intrinsics_cam->distortion.resize(2);
                intrinsics_cam->distortion = distortion.head<2>();
            }
        else
        {
            intrinsics_cam->distortion.resize(3);
            intrinsics_cam->distortion.head<2>() = distortion.head<2>();
            intrinsics_cam->distortion.tail<1>() = distortion.tail<1>();
        }

        //=========================================
        // ===== this part for debugging only =====
        //=========================================
//        std::cout << "\n--- Parameters Parsed from YAML file ---" << std::endl;
//        std::cout << "sensor type: " << sensor_type << std::endl;
//        std::cout << "sensor name: " << sensor_name << std::endl;

//        // extrinsics discarded in this creator
//        Vector3d pos            = camera_config["extrinsic"]["position"].as<Vector3d>();
//        Vector3d ori            = camera_config["extrinsic"]["orientation"].as<Vector3d>() * M_PI / 180; // roll, pitch, yaw [rad]
//        Quaternions quat; v2q(ori, quat);
//        std::cout << "sensor extrinsics: " << std::endl;
//        std::cout << "\tposition    : " << pos.transpose() << std::endl;
//        std::cout << "\torientation : " << ori.transpose() << std::endl;

//        std::cout << "sensor intrinsics: " << std::endl;
//        std::cout << "\timage size  : " << width << "x" << height << std::endl;
//        std::cout << "\tintrinsic   : " << intrinsic.transpose() << std::endl;
//        std::cout << "\tdistoriton  : " << distortion.transpose() << std::endl;
        //=========================================

        return intrinsics_cam;
    }

    std::cout << "Bad configuration file. No or bad sensor type found." << std::endl;
    return nullptr;
}

// Register in the SensorFactory
const bool WOLF_UNUSED registered_camera_intr = IntrinsicsFactory::get().registerCreator("CAMERA", createIntrinsicsCamera);

} // namespace [unnamed]

} // namespace wolf


/**
 * \file yaml-test.cpp
 *
 *  Created on: May 1, 2016
 *      \author: jsola
 */

#include "base/math/pinhole_tools.h"
#include "yaml/yaml_conversion.h"
#include "processor_image_feature.h"
#include "base/common/factory.h"

#include <yaml-cpp/yaml.h>

#include <eigen3/Eigen/Dense>

#include <iostream>
#include <fstream>

int main()
{

    //=============================================================================================
    std::string wolf_root       = _WOLF_ROOT_DIR;
    std::cout << "\nwolf root directory: " << wolf_root << std::endl;
    //=============================================================================================

    using namespace Eigen;
    using namespace wolf;
    using std::string;
    using YAML::Node;

    // Camera parameters

    YAML::Node camera_config = YAML::LoadFile(wolf_root + "/src/examples/camera.yaml");

    if (camera_config["sensor type"])
    {
        std::string sensor_type = camera_config["sensor type"].as<std::string>();

        std::string sensor_name = camera_config["sensor name"].as<std::string>();

        YAML::Node params   = camera_config["intrinsic"];

        // convert yaml to Eigen
        Vector3s pos        = camera_config["extrinsic"]["position"].as<Vector3s>();
        Vector3s ori        = camera_config["extrinsic"]["orientation"].as<Vector3s>() * M_PI / 180;
        Vector2s size       = params["image size"].as<Vector2s>();
        Vector4s intrinsic  = params["pinhole model"].as<Vector4s>();
        VectorXs distortion = params["distortion"].as<VectorXs>();

        // compute correction model
        VectorXs correction(distortion.size());
        pinhole::computeCorrectionModel(intrinsic, distortion, correction);

        // output
        std::cout << "sensor type       : " << sensor_type << std::endl;
        std::cout << "sensor name       : " << sensor_name << std::endl;
        std::cout << "sensor extrinsics : " << std::endl;
        std::cout << "\tposition        : " << pos.transpose() << std::endl;
        std::cout << "\torientation     : " << ori.transpose() << std::endl;
        std::cout << "sensor parameters : " << std::endl;
        std::cout << "\timage size      : " << size.transpose() << std::endl;
        std::cout << "\tpinhole model   : " << intrinsic.transpose() << std::endl;
        std::cout << "\tdistoriton      : " << distortion.transpose() << std::endl;
        std::cout << "\tcorrection      : " << correction.transpose() << std::endl;
    }
    else
        std::cout << "Bad configuration file. No sensor type found." << std::endl;

//    // Processor Image parameters
//
//    ProcessorParamsImage p;
//
//    Node params = YAML::LoadFile(wolf_root + "/src/examples/processor_image_feature.yaml");
//
//    if (params["processor type"])
//    {
//        Node as = params["active search"];
//        p.active_search.grid_width      = as["grid width"].as<unsigned int>();
//        p.active_search.grid_height     = as["grid height"].as<unsigned int>();
//        p.active_search.separation      = as["separation"].as<unsigned int>();
//
//        Node img = params["image"];
//        p.image.width                   = img["width"].as<unsigned int>();
//        p.image.height                  = img["height"].as<unsigned int>();
//
//        Node alg = params["algorithm"];
//        p.max_new_features            = alg["maximum new features"].as<unsigned int>();
//        p.min_features_for_keyframe   = alg["minimum features for new keyframe"].as<unsigned int>();
//    }

    return 0;
}

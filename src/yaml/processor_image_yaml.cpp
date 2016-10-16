/**
 * \file processor_image_yaml.cpp
 *
 *  Created on: May 21, 2016
 *      \author: jsola
 */

// wolf yaml
#include "yaml_conversion.h"

// wolf
#include "../processor_image.h"
#include "../factory.h"

// yaml-cpp library
#include <yaml-cpp/yaml.h>

namespace wolf
{
namespace
{
static ProcessorParamsBasePtr createProcessorParamsImage(const std::string & _filename_dot_yaml)
{
    using std::string;
    using YAML::Node;

    std::shared_ptr<ProcessorParamsImage> p = std::make_shared<ProcessorParamsImage>();

    Node params = YAML::LoadFile(_filename_dot_yaml);

    if (!params.IsNull())
    {
        Node dd_yaml = params["detector-descriptor"];
        if(dd_yaml["type"].as<string>() == "ORB")
        {
            DetectorDescriptorParamsOrb* dd = new DetectorDescriptorParamsOrb;
            dd->type                        = DD_ORB;
            dd->nfeatures                   = dd_yaml["nfeatures"].as<unsigned int>();
            dd->scaleFactor                 = dd_yaml["scale factor"].as<float>();
            dd->nlevels                     = dd_yaml["nlevels"].as<unsigned int>();
            dd->edgeThreshold               = dd_yaml["edge threshold"].as<unsigned int>();
            std::cout << "edgeThreshold: " << dd->edgeThreshold << std::endl;
            dd->firstLevel                  = dd_yaml["first level"].as<unsigned int>();
            dd->WTA_K                       = dd_yaml["WTA_K"].as<unsigned int>();
            dd->scoreType                   = dd_yaml["score type"].as<int>(); // enum { kBytes = 32, HARRIS_SCORE=0, FAST_SCORE=1 };
            dd->patchSize                   = dd_yaml["patch size"].as<unsigned int>();
            p->detector_descriptor_params_ptr = dd;
        }else
        {
            std::cout << "Unknown detector-descriptor type " << dd_yaml["type"].as<string>() << std::endl;
            // TODO: add BRISK params struct
        }

        Node m = params["matcher"];
        p->matcher.min_normalized_score = m["minimum normalized score"].as<Scalar>();
        p->matcher.similarity_norm      = m["similarity norm"].as<int>(); // enum { NORM_INF=1, NORM_L1=2, NORM_L2=4, NORM_L2SQR=5, NORM_HAMMING=6, NORM_HAMMING2=7, NORM_TYPE_MASK=7, NORM_RELATIVE=8, NORM_MINMAX=32 };
        p->matcher.roi_width            = m["roi"]["width"].as<unsigned int>();
        p->matcher.roi_height           = m["roi"]["height"].as<unsigned int>();

        Node as = params["active search"];
        p->active_search.grid_width     = as["grid width"].as<unsigned int>();
        p->active_search.grid_height    = as["grid height"].as<unsigned int>();
        p->active_search.separation     = as["separation"].as<unsigned int>();

        Node img = params["image"];
        p->image.width  = img["width"].as<unsigned int>();
        p->image.height = img["height"].as<unsigned int>();

        Node alg = params["algorithm"];
        p->algorithm.max_new_features = alg["maximum new features"].as<unsigned int>();
        p->algorithm.min_features_for_keyframe = alg["minimum features for new keyframe"].as<unsigned int>();

    }

    return p;
}

// Register in the SensorFactory
const bool registered_prc_image_par = ProcessorParamsFactory::get().registerCreator("IMAGE", createProcessorParamsImage);
const bool registered_prc_image_landmark_par = ProcessorParamsFactory::get().registerCreator("IMAGE LANDMARK", createProcessorParamsImage);


}
}


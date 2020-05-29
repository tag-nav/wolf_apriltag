/**
 * \file processor_tracker_landmark_apriltag_yaml.cpp
 *
 *  Created on: Dec 6, 2018
 *      \author: jsola
 */


// wolf
#include "apriltag/processor/processor_tracker_landmark_apriltag.h"
#include "core/yaml/yaml_conversion.h"
#include "core/common/factory.h"

// yaml-cpp library
#include <yaml-cpp/yaml.h>

namespace wolf
{

namespace
{
static ParamsProcessorBasePtr createParamsProcessorLandmarkApriltag(const std::string & _filename_dot_yaml)
{
    YAML::Node config = YAML::LoadFile(_filename_dot_yaml);

    if (config.IsNull())
    {
        WOLF_ERROR("Invalid YAML file!");
        return nullptr;
    }
    else if (config["type"].as<std::string>() == "ProcessorTrackerLandmarkApriltag")
    {
        ParamsProcessorTrackerLandmarkApriltagPtr params = std::make_shared<ParamsProcessorTrackerLandmarkApriltag>();

        YAML::Node detector_parameters      = config["detector parameters"];
        params->quad_decimate_              = detector_parameters["quad_decimate"]            .as<double>();
        params->quad_sigma_                 = detector_parameters["quad_sigma"]               .as<double>();
        params->nthreads_                   = detector_parameters["nthreads"]                 .as<int>();
        params->debug_                      = detector_parameters["debug"]                    .as<bool>();
        params->refine_edges_               = detector_parameters["refine_edges"]             .as<bool>();
        params->ippe_min_ratio_             = detector_parameters["ippe_min_ratio"]           .as<double>();
        params->ippe_max_rep_error_         = detector_parameters["ippe_max_rep_error"]       .as<double>();

        YAML::Node tag_parameters           = config["tag parameters"];
        params->tag_family_                 = tag_parameters["tag_family"]          .as<std::string>();
        // params->tag_black_border_           = tag_parameters["tag_black_border"]    .as<int>();
        params->tag_width_default_          = tag_parameters["tag_width_default"]   .as<double>();

        // read list of tag widths
        YAML::Node tag_widths               = config["tag widths"];
        for (auto pair_id_width : tag_widths)
        {
            int tag_id                      = pair_id_width.first                   .as<int>();
            double tag_width                = pair_id_width.second                  .as<double>();
            params->tag_widths_.emplace(tag_id, tag_width);
        }

        YAML::Node noise                    = config["noise"];
        params->std_xy_                     = noise["std_xy"]                       .as<double>();
        params->std_z_                      = noise["std_z"]                        .as<double>();
        params->std_rpy_          = M_TORAD * noise["std_rpy_degrees"]              .as<double>();
        params->std_pix_                    = noise["std_pix"]                      .as<double>();

        YAML::Node vote                     = config["vote"];
        params->voting_active               = vote["voting active"]                  .as<bool>();
        params->min_time_vote_              = vote["min_time_vote"]                  .as<double>();
        params->max_time_vote_              = vote["max_time_vote"]                  .as<double>();
        params->min_features_for_keyframe   = vote["min_features_for_keyframe"]      .as<unsigned int>();
        params->max_features_diff_          = vote["max_features_diff"]              .as<int>();
        params->nb_vote_for_every_first_    = vote["nb_vote_for_every_first"]        .as<int>();
        params->enough_info_necessary_      = vote["enough_info_necessary"]          .as<bool>();
        
        params->reestimate_last_frame_      = config["reestimate_last_frame"]        .as<bool>();
        params->add_3d_cstr_                = config["add_3D_cstr"]                  .as<bool>();

        params->max_new_features            = config["max_new_features"]             .as<int>();
        params->apply_loss_function         = config["apply_loss_function"]          .as<bool>();

        return params;
    }
    else
    {
        WOLF_ERROR("Wrong processor type! Should be \"ProcessorTrackerLandmarkApriltag\"");
        return nullptr;
    }
    return nullptr;
}

// Register in the FactorySensor
const bool WOLF_UNUSED registered_prc_apriltag = FactoryParamsProcessor::get().registerCreator("ProcessorTrackerLandmarkApriltag", createParamsProcessorLandmarkApriltag);
const bool WOLF_UNUSED registered_prc_apriltag_wrapper = FactoryParamsProcessor::get().registerCreator("ProcessorTrackerLandmarkApriltag_Wrapper", createParamsProcessorLandmarkApriltag);

} // namespace [unnamed]

} // namespace wolf

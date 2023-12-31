//--------LICENSE_START--------
//
// Copyright (C) 2020,2021,2022,2023 Institut de Robòtica i Informàtica Industrial, CSIC-UPC.
// Authors: Joan Solà Ortega (jsola@iri.upc.edu)
// All rights reserved.
//
// This file is part of WOLF
// WOLF is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
//--------LICENSE_END--------

// Wolf apriltag
#include "apriltag/landmark/landmark_apriltag.h"

// Wolf core
#include <core/common/factory.h>
#include <core/math/rotations.h>
#include <core/yaml/yaml_conversion.h>
#include <core/state_block/state_block_derived.h>
#include <core/state_block/state_quaternion.h>

namespace wolf {

LandmarkApriltag::LandmarkApriltag(Eigen::Vector7d& pose, const int& _tagid, const double& _tag_width) :
	LandmarkBase("LandmarkApriltag", std::make_shared<StatePoint3d>(pose.head(3)), std::make_shared<StateQuaternion>(pose.tail(4))),
	tag_id_(_tagid),
	tag_width_(_tag_width)
{
  	setDescriptor(Eigen::VectorXd::Constant(1,_tagid)); //change tagid to int ? do not use descriptor vector ?
}

LandmarkApriltag::~LandmarkApriltag()
{
    //
}


double LandmarkApriltag::getTagWidth() const
{
    return tag_width_;
}

int LandmarkApriltag::getTagId() const
{
    return tag_id_;
}

// LANDMARK SAVE AND LOAD FROM YAML

// static
LandmarkBasePtr LandmarkApriltag::create(const YAML::Node& _lmk_node)
{
    // Parse YAML node with lmk info and data
    unsigned int    tag_id                  = _lmk_node["tag id"]               .as<unsigned int>();
    double          tag_width               = _lmk_node["tag width"]            .as<double>();
    Eigen::Vector3d pos                     = _lmk_node["position"]             .as<Eigen::Vector3d>();
    bool            pos_fixed               = _lmk_node["position fixed"]       .as<bool>();
    Eigen::Vector4d vquat;
    if (_lmk_node["orientation"].size() == 3)
    {
        // we have been given 3 Euler angles in degrees
        Eigen::Vector3d   euler = M_TORAD * ( _lmk_node["orientation"]          .as<Eigen::Vector3d>() );
        Eigen::Matrix3d       R = e2R(euler);
        Eigen::Quaterniond quat = R2q(R);
        vquat                   = quat.coeffs();
    }
    else if (_lmk_node["orientation"].size() == 4)
    {
        // we have been given a quaternion
        vquat                               = _lmk_node["orientation"]          .as<Eigen::Vector4d>();
    }
    bool            ori_fixed               = _lmk_node["orientation fixed"]    .as<bool>();

    bool            transformable           = _lmk_node["transformable"]        .as<bool>();

    Eigen::Vector7d pose; pose << pos, vquat;

    // Create a new landmark
    LandmarkApriltagPtr lmk_ptr = std::make_shared<LandmarkApriltag>(pose, tag_id, tag_width);
    lmk_ptr->getP()->setFixed(pos_fixed);
    lmk_ptr->getO()->setFixed(ori_fixed);
    lmk_ptr->getP()->setTransformable(transformable);
    lmk_ptr->getO()->setTransformable(transformable);

    return lmk_ptr;

}

YAML::Node LandmarkApriltag::saveToYaml() const
{
    // First base things
    YAML::Node node = LandmarkBase::saveToYaml();

    // Then Apriltag specific things
    node["tag id"] = getTagId();
    node["tag width"] = getTagWidth();

    return node;
}


// Register landmark creator
namespace
{
const bool WOLF_UNUSED registered_lmk_apriltag = FactoryLandmark::registerCreator("LandmarkApriltag", LandmarkApriltag::create);
}


} // namespace wolf

//--------LICENSE_START--------
//
// Copyright (C) 2020,2021,2022 Institut de Robòtica i Informàtica Industrial, CSIC-UPC.
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
#include "apriltag/feature/feature_apriltag_proj.h"

namespace wolf {

FeatureApriltagProj::FeatureApriltagProj(const Eigen::Vector8d & _measurement,
                                         const Eigen::Matrix8d & _meas_uncertainty,
                                         const int _tag_id,
                                         const double _tag_width,
                                         const Eigen::Vector7d& _pose_pnp,
                                         UncertaintyType _uncertainty_type) :
    FeatureBase("FeatureApriltagProj", _measurement, _meas_uncertainty, _uncertainty_type),
    tag_id_     (_tag_id),
    tag_width_  (_tag_width),
    tag_corners_(4),
    pose_pnp_(_pose_pnp)
{
    setTrackId(_tag_id);  // assuming there is a single landmark with this id in the scene

    for (int i = 0; i < 4; i++)
    {
        tag_corners_[i].x = _measurement[2*i];
        tag_corners_[i].y = _measurement[2*i+1];
    }
}

FeatureApriltagProj::~FeatureApriltagProj()
{
    //
}

int FeatureApriltagProj::getTagId() const
{
    return tag_id_;
}

double FeatureApriltagProj::getTagWidth() const
{
    return tag_width_;
}

const std::vector<cv::Point2d>& FeatureApriltagProj::getTagCorners() const
{
    return tag_corners_;
}

const Eigen::Vector7d& FeatureApriltagProj::getPosePnp() const
{
    return pose_pnp_;
}

} // namespace wolf

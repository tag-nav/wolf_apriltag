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
#include "apriltag/feature/feature_apriltag_pose.h"

namespace wolf {

FeatureApriltagPose::FeatureApriltagPose(const Eigen::Vector7d & _measurement,
                                 const Eigen::Matrix6d & _meas_uncertainty,
                                 const int _tag_id,
                                 const Vector8d & _corners_vec,
                                 double _rep_error1,
                                 double _rep_error2,
                                 bool _use_rotation,
                                 UncertaintyType _uncertainty_type) :
    FeatureBase("FeatureApriltagPose", _measurement, _meas_uncertainty, _uncertainty_type),
    tag_id_     (_tag_id),
    tag_corners_(4),
    rep_error1_(_rep_error1),
    rep_error2_(_rep_error2),
    use_rotation_(_use_rotation)
{
    setTrackId(_tag_id);  // assuming there is a single landmark with this id in the scene

    for (int i = 0; i < 4; i++)
    {
        tag_corners_[i].x = _corners_vec[2*i];
        tag_corners_[i].y = _corners_vec[2*i+1];
    }
}

FeatureApriltagPose::~FeatureApriltagPose()
{
    //
}

double FeatureApriltagPose::getTagId() const
{
    return tag_id_;
}

const std::vector<cv::Point2d>& FeatureApriltagPose::getTagCorners() const
{
    return tag_corners_;
}

double FeatureApriltagPose::getRepError1() const
{
    return rep_error1_;
}

double FeatureApriltagPose::getRepError2() const
{
    return rep_error2_;
}

bool FeatureApriltagPose::getUserotation() const
{
    return use_rotation_;
}

} // namespace wolf

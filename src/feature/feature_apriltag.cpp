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
#include "apriltag/feature/feature_apriltag.h"

namespace wolf {

FeatureApriltag::FeatureApriltag(const Eigen::Vector7d & _measurement,
                                 const Eigen::Matrix6d & _meas_uncertainty,
                                 const int _tag_id,
                                 const apriltag_detection_t & _det,
                                 double _rep_error1,
                                 double _rep_error2,
                                 bool _use_rotation,
                                 UncertaintyType _uncertainty_type) :
    FeatureBase("FeatureApriltag", _measurement, _meas_uncertainty, _uncertainty_type),
    tag_id_     (_tag_id),
    tag_corners_(4),
    detection_  (_det),
    rep_error1_(_rep_error1),
    rep_error2_(_rep_error2),
    use_rotation_(_use_rotation)
{
    assert(_det.id == _tag_id && "Tag ID and detection ID do not match!");
    setTrackId(_tag_id);
    for (int i = 0; i < 4; i++)
    {
        tag_corners_[i].x = detection_.p[i][0];
        tag_corners_[i].y = detection_.p[i][1];
    }
}

FeatureApriltag::~FeatureApriltag()
{
    //
}

double FeatureApriltag::getTagId() const
{
    return tag_id_;
}

const apriltag_detection_t& FeatureApriltag::getDetection() const
{
    return detection_;
}

const std::vector<cv::Point2d>& FeatureApriltag::getTagCorners() const
{
    return tag_corners_;
}

double FeatureApriltag::getRepError1() const
{
    return rep_error1_;
}

double FeatureApriltag::getRepError2() const
{
    return rep_error2_;
}

bool FeatureApriltag::getUserotation() const
{
    return use_rotation_;
}

} // namespace wolf

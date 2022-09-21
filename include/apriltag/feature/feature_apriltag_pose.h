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
#ifndef FEATURE_APRILTAG_POSE_H_
#define FEATURE_APRILTAG_POSE_H_

// Wolf apriltag
#include "apriltag/feature/feature_apriltag.h"

// Wolf core
#include <core/feature/feature_base.h>

// opencv
#include <opencv2/features2d.hpp>

namespace wolf {
    
WOLF_PTR_TYPEDEFS(FeatureApriltagPose);


//class FeatureApriltagPose
class FeatureApriltagPose : public FeatureApriltag
{
    public:

        FeatureApriltagPose(
                        const Eigen::Vector7d & _measurement,
                        const Eigen::Matrix6d & _meas_info,
                        int _tag_id,
                        double _tag_width,
                        const Vector8d & _corners_vec,
                        bool _use_rotation,
                        double _detection_margin,
                        double _reprojection_error_best, 
                        double _reprojection_error_second, 
                        UncertaintyType _uncertainty_type = UNCERTAINTY_IS_INFO);
        ~FeatureApriltagPose() override;

        Eigen::VectorXd getPosePnp() const override;
};

inline FeatureApriltagPose::FeatureApriltagPose(
                                 const Eigen::Vector7d & _measurement,
                                 const Eigen::Matrix6d & _meas_info,
                                 const int _tag_id,
                                 double _tag_width,
                                 const Vector8d & _corners_vec,
                                 bool _use_rotation,
                                 double _detection_margin,
                                 double _reprojection_error_best, 
                                 double _reprojection_error_second, 
                                 UncertaintyType _uncertainty_type) :
    FeatureApriltag("FeatureApriltagPose", _measurement, _meas_info, _tag_id, _tag_width, _corners_vec, _use_rotation, _detection_margin, _reprojection_error_best, _reprojection_error_second, _uncertainty_type)
{
}

inline FeatureApriltagPose::~FeatureApriltagPose()
{
    //
}

inline Eigen::VectorXd FeatureApriltagPose::getPosePnp() const
{
    return measurement_;
}

} // namespace wolf

#endif

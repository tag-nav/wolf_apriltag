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
                        const int _tag_id,
                        const Vector8d & _corners_vec,
                        double _rep_error1,
                        double _rep_error2,
                        bool _use_rotation,
                        UncertaintyType _uncertainty_type = UNCERTAINTY_IS_INFO);
        ~FeatureApriltagPose() override;
        
        double getRepError1() const;
        double getRepError2() const;
        bool getUserotation() const;

    private:
        double rep_error1_;
        double rep_error2_;
        bool use_rotation_;
};

inline FeatureApriltagPose::FeatureApriltagPose(
                                 const Eigen::Vector7d & _measurement,
                                 const Eigen::Matrix6d & _meas_info,
                                 const int _tag_id,
                                 const Vector8d & _corners_vec,
                                 double _rep_error1,
                                 double _rep_error2,
                                 bool _use_rotation,
                                 UncertaintyType _uncertainty_type) :
    FeatureApriltag("FeatureApriltagPose", _measurement, _meas_info, _tag_id, _corners_vec, _uncertainty_type),
    rep_error1_(_rep_error1),
    rep_error2_(_rep_error2),
    use_rotation_(_use_rotation)
{
}

inline FeatureApriltagPose::~FeatureApriltagPose()
{
    //
}

inline double FeatureApriltagPose::getRepError1() const
{
    return rep_error1_;
}
inline double FeatureApriltagPose::getRepError2() const
{
    return rep_error2_;
}
inline bool FeatureApriltagPose::getUserotation() const
{
    return use_rotation_;
}

} // namespace wolf

#endif

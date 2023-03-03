// WOLF - Copyright (C) 2020,2021,2022,2023
// Institut de Robòtica i Informàtica Industrial, CSIC-UPC.
// Authors: Joan Solà Ortega (jsola@iri.upc.edu) and
// Joan Vallvé Navarro (jvallve@iri.upc.edu)
// All rights reserved.
//
// This file is part of WOLF: http://www.iri.upc.edu/wolf
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

#pragma once

// Wolf includes
#include "apriltag/common/apriltag.h"
#include "core/feature/feature_base.h"

// std includes

// external library incudes
#include "apriltag/apriltag.h"
#include "apriltag/common/zarray.h"

// opencv
#include <opencv2/features2d.hpp>

namespace wolf
{

WOLF_PTR_TYPEDEFS(FeatureApriltag);

// class FeatureApriltag
class FeatureApriltag : public FeatureBase
{
  public:
    FeatureApriltag(const Eigen::Vector7d&      _measurement,
                    const Eigen::Matrix6d&      _meas_covariance,
                    const int                   _tag_id,
                    const apriltag_detection_t& _det,
                    double                      _rep_error1,
                    double                      _rep_error2,
                    bool                        _use_rotation,
                    UncertaintyType             _uncertainty_type = UNCERTAINTY_IS_INFO);
    ~FeatureApriltag() override;

    /** \brief Returns tag id
     *
     * Returns tag id
     *
     **/
    double getTagId() const;

    const apriltag_detection_t& getDetection() const;

    const std::vector<cv::Point2d>& getTagCorners() const;

    double getRepError1() const;
    double getRepError2() const;
    bool   getUserotation() const;

  private:
    int                      tag_id_;
    std::vector<cv::Point2d> tag_corners_;
    apriltag_detection_t     detection_;
    double                   rep_error1_;
    double                   rep_error2_;
    bool                     use_rotation_;
};

}  // namespace wolf
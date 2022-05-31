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
#ifndef FEATURE_APRILTAG_PROJ_H_
#define FEATURE_APRILTAG_PROJ_H_

//Wolf includes
#include "core/feature/feature_base.h"

//std includes

//external library incudes
#include "apriltag/apriltag.h"
#include "apriltag/common/zarray.h"

// opencv
#include <opencv2/features2d.hpp>

namespace wolf {
    
WOLF_PTR_TYPEDEFS(FeatureApriltagProj);

//class FeatureApriltagProj
class FeatureApriltagProj : public FeatureBase
{
    public:
        FeatureApriltagProj(const Eigen::Vector8d & _measurement,
                            const Eigen::Matrix8d & _meas_covariance,
                            const int _tag_id,
                            const double _tag_width,
                            UncertaintyType _uncertainty_type = UNCERTAINTY_IS_COVARIANCE);

        ~FeatureApriltagProj() override;
        
        /** \brief Returns tag id
         * 
         * Returns tag id
         * 
         **/
        int getTagId() const; 
        double getTagWidth() const; 

        const std::vector<cv::Point2d>& getTagCorners() const;

    private:
        int tag_id_;
        double tag_width_;
        std::vector<cv::Point2d> tag_corners_;
        
};

} // namespace wolf

#endif

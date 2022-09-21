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
#ifndef FEATURE_APRILTAG_H_
#define FEATURE_APRILTAG_H_

//Wolf includes
#include "core/feature/feature_base.h"

// opencv
#include <opencv2/features2d.hpp>

namespace wolf {
    
WOLF_PTR_TYPEDEFS(FeatureApriltag);


//class FeatureApriltag
class FeatureApriltag : public FeatureBase
{
    public:

        FeatureApriltag(
            const std::string _type,
            const Eigen::VectorXd & _measurement,
            const Eigen::MatrixXd & _meas_uncertainty,
            const int _tag_id,
            double _tag_width,
            const Vector8d & _corners_vec,
            bool _use_rotation,
            double _detection_margin,
            double _reprojection_error_best, 
            double _reprojection_error_second, 
            UncertaintyType _uncertainty_type = UNCERTAINTY_IS_INFO);
        ~FeatureApriltag() override;
        
        /** \brief Return tag id
         * 
         **/
        int getTagId() const; 

        /** \brief Return vector of tag corners
         * 
         **/
        const std::vector<cv::Point2d>& getTagCorners() const;

        /** \brief Return pose compute by PnP algorithm
         * 
         **/
        virtual Eigen::VectorXd getPosePnp() const = 0;

        /** \brief Return a boolean assessing if the rotation part of the transformation is reliable.
         * 
         **/
        bool getUseRotation() const;

        /** \brief Return the width of the detected tag (from user specification)
         * 
         **/
        double getTagWidth() const;


    private:
        int tag_id_;
        double tag_width_;
        std::vector<cv::Point2d> tag_corners_;
        bool use_rotation_;
        double detection_margin_, reprojection_error_best_, reprojection_error_second_;        
};


inline FeatureApriltag::FeatureApriltag(
    const std::string _type,
    const Eigen::VectorXd & _measurement,
    const Eigen::MatrixXd & _meas_uncertainty,
    const int _tag_id,
    double _tag_width,
    const Vector8d & _corners_vec,
    bool _use_rotation,
    double _detection_margin,
    double _reprojection_error_best, 
    double _reprojection_error_second, 
    UncertaintyType _uncertainty_type) :
        FeatureBase(_type, _measurement, _meas_uncertainty, _uncertainty_type),
        tag_id_(_tag_id),
        tag_width_(_tag_width),
        tag_corners_(4),
        use_rotation_(_use_rotation),
        detection_margin_(_detection_margin),
        reprojection_error_best_(_reprojection_error_best),
        reprojection_error_second_(_reprojection_error_second)
{
    setTrackId(_tag_id);  // assuming there is a single landmark with this id in the scene

    for (int i = 0; i < 4; i++)
    {
        tag_corners_[i].x = _corners_vec[2*i];
        tag_corners_[i].y = _corners_vec[2*i+1];
    }
}

inline FeatureApriltag::~FeatureApriltag()
{
    //
}

inline int FeatureApriltag::getTagId() const
{
    return tag_id_;
}

inline const std::vector<cv::Point2d>& FeatureApriltag::getTagCorners() const
{
    return tag_corners_;
}

inline double FeatureApriltag::getTagWidth() const
{
    return tag_width_;
}

inline bool FeatureApriltag::getUseRotation() const
{
    return use_rotation_;
}



} // namespace wolf

#endif

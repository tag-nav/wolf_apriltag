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

#ifndef LANDMARK_APRILTAG_H_
#define LANDMARK_APRILTAG_H_

//Wolf includes
#include "core/landmark/landmark_base.h"

#include "core/state_block/state_quaternion.h"

// Std includes


namespace wolf {

WOLF_PTR_TYPEDEFS(LandmarkApriltag);
    
//class LandmarkApriltag
class LandmarkApriltag : public LandmarkBase
{
    public:
        /** \brief Constructor with type, time stamp and the position state pointer
         *
         * Constructor with type, and state pointer
         * \param _p_ptr StateBlock shared pointer to the position
         * \param _o_ptr StateBlock shared pointer to the orientation
         * \param _tagid descriptor of the landmark: id of the tag
         * \param _tag_width : total width of the tag (in metres)
         *
         **/
		LandmarkApriltag(Eigen::Vector7d& pose, const int& _tagid, const double& _tag_width);

        ~LandmarkApriltag() override;
        
        /** \brief Returns tag id
         * 
         * Returns id of the tag
         * 
         **/
        int getTagId() const;
        
        /** \brief Returns tag width
         * 
         * Returns width of the tag
         * 
         **/
        double getTagWidth() const;

        /** Factory method to create new landmarks from YAML nodes
         */
        static LandmarkBasePtr create(const YAML::Node& _lmk_node);

        YAML::Node toYaml() const override;


    private:
        const int tag_id_;
        const double tag_width_;         
        
};

} // namespace wolf

#endif

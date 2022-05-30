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
/**
 * \file gtest_feature_apriltag_proj.cpp
 *
 *  Created on: May 30, 2022
 *      \author: mfourmy
 */


#include <core/utils/utils_gtest.h>

#include "apriltag/feature/feature_apriltag_proj.h"

using namespace wolf;

class FeatureApriltag_test : public testing::Test
{
    public:
        Eigen::Vector8d meas;
        Eigen::Matrix8d cov;
        int tag_id_;
        double tag_width_;
        apriltag_detection_t det_;

        void SetUp() override
        {
            meas << 1,2,3,4,5,6,7,8;
            cov.setIdentity() * 2.0;

            tag_id_ = 1;
            tag_width_ = 0.2;

            det_.id      = 1;
            det_.p[0][0] =  1.0;
            det_.p[0][1] = -1.0;
            det_.p[1][0] =  1.0;
            det_.p[1][1] =  1.0;
            det_.p[2][0] = -1.0;
            det_.p[2][1] =  1.0;
            det_.p[3][0] = -1.0;
            det_.p[3][1] = -1.0;
        }
};

TEST_F(FeatureApriltag_test, type)
{
    auto f = std::make_shared<FeatureApriltagProj>(meas, cov, tag_id_, tag_width_, det_);

    ASSERT_EQ(f->getType(), "FeatureApriltagProj");
}

TEST_F(FeatureApriltag_test, getters)
{
    auto f = std::make_shared<FeatureApriltagProj>(meas, cov, tag_id_, tag_width_, det_);

    ASSERT_EQ(f->getTagId(), tag_id_);
    ASSERT_EQ(f->getTagWidth(), tag_width_);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}


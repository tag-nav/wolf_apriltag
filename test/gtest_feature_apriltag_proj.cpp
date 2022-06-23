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

class FeatureApriltagProj_test : public testing::Test
{
    public:
        Eigen::Vector8d meas_;
        Eigen::Matrix8d cov_;
        int tag_id_;
        double tag_width_;
        apriltag_detection_t det_;
        Eigen::Vector7d pose_pnp_;

        void SetUp() override
        {

            tag_id_ = 1;
            tag_width_ = 0.2;

            meas_ << 1.0, -1.0,
                     1.0,  1.0,
                    -1.0,  1.0,
                    -1.0, -1.0;
            cov_.setIdentity();
            pose_pnp_ << 0,0,0, 0,0,0,1;
        }
};

TEST_F(FeatureApriltagProj_test, type)
{
    auto f = std::make_shared<FeatureApriltagProj>(meas_, cov_, tag_id_, tag_width_, pose_pnp_);

    ASSERT_EQ(f->getType(), "FeatureApriltagProj");
}

TEST_F(FeatureApriltagProj_test, getters)
{
    auto f = std::make_shared<FeatureApriltagProj>(meas_, cov_, tag_id_, tag_width_, pose_pnp_);

    ASSERT_EQ(f->getTagId(), tag_id_);
    ASSERT_EQ(f->getTagWidth(), tag_width_);
    ASSERT_MATRIX_APPROX(f->getPosePnp(), pose_pnp_, 1e-6);
}


TEST_F(FeatureApriltagProj_test, getCorners)
{
    auto f = std::make_shared<FeatureApriltagProj>(meas_, cov_, tag_id_, tag_width_, pose_pnp_);

    ASSERT_EQ(f->getTagCorners().size(), 4);

    ASSERT_EQ(f->getTagCorners()[0].x,  1.0);
    ASSERT_EQ(f->getTagCorners()[0].y, -1.0);
    ASSERT_EQ(f->getTagCorners()[1].x,  1.0);
    ASSERT_EQ(f->getTagCorners()[1].y,  1.0);
    ASSERT_EQ(f->getTagCorners()[2].x, -1.0);
    ASSERT_EQ(f->getTagCorners()[2].y,  1.0);
    ASSERT_EQ(f->getTagCorners()[3].x, -1.0);
    ASSERT_EQ(f->getTagCorners()[3].y, -1.0);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}


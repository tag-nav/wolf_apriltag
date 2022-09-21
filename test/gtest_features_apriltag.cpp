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

#include <core/utils/utils_gtest.h>

#include "apriltag/feature/feature_apriltag.h"
#include "apriltag/feature/feature_apriltag_pose.h"
#include "apriltag/feature/feature_apriltag_proj.h"

using namespace wolf;

class FeatureApriltag_fixture : public testing::Test
{
    public:
        Eigen::Vector7d pose;
        Eigen::Matrix6d info_pose;
        Eigen::Vector8d corners_vec;
        Eigen::Matrix8d cov_pix;
        int tag_id;
        double tag_width;
        bool use_rotation;
        double decision_margin, rep_err1, rep_err2;

        void SetUp() override
        {
            pose << 1,2,3,4,5,6,7;
            pose = pose + 42*Eigen::Vector7d::Ones();
            info_pose.setIdentity();

            corners_vec  << 1.0, -1.0,
                            1.0,  1.0,
                           -1.0,  1.0,
                           -1.0, -1.0;
            corners_vec = 42.0*corners_vec;
            cov_pix.setIdentity();

            tag_id = 1;
            tag_width = 0.1;

            use_rotation = true;

            decision_margin = 10;
            rep_err1 = 1;
            rep_err2 = 2;
        }
};

TEST_F(FeatureApriltag_fixture, type)
{
    FeatureApriltagPosePtr fa_pose = std::make_shared<FeatureApriltagPose>(pose, info_pose, tag_id, tag_width, corners_vec, use_rotation, decision_margin, rep_err1, rep_err2);
    ASSERT_EQ(fa_pose->getType(), "FeatureApriltagPose");

    FeatureApriltagProjPtr fa_proj = std::make_shared<FeatureApriltagProj>(corners_vec, cov_pix, tag_id, tag_width, pose, use_rotation, decision_margin, rep_err1, rep_err2);
    ASSERT_EQ(fa_proj->getType(), "FeatureApriltagProj");
}

TEST_F(FeatureApriltag_fixture, FeatureApriltag_getters)
{
    // FeatureApriltag has a pure virtual method, need to instanciate one of the derived classes
    FeatureApriltagPosePtr fa_pose = std::make_shared<FeatureApriltagPose>(pose, info_pose, tag_id, tag_width, corners_vec, use_rotation, decision_margin, rep_err1, rep_err2);

    ASSERT_EQ(fa_pose->getTagId(), 1);
    ASSERT_EQ(fa_pose->getTagWidth(), tag_width);
    ASSERT_EQ(fa_pose->getTagCorners().at(0).x, corners_vec(0));
    ASSERT_EQ(fa_pose->getTagCorners().at(0).y, corners_vec(1));
    ASSERT_EQ(fa_pose->getTagCorners().at(1).x, corners_vec(2));
    ASSERT_EQ(fa_pose->getTagCorners().at(1).y, corners_vec(3));
    ASSERT_EQ(fa_pose->getTagCorners().at(2).x, corners_vec(4));
    ASSERT_EQ(fa_pose->getTagCorners().at(2).y, corners_vec(5));
    ASSERT_EQ(fa_pose->getTagCorners().at(3).x, corners_vec(6));
    ASSERT_EQ(fa_pose->getTagCorners().at(3).y, corners_vec(7));
    ASSERT_EQ(fa_pose->getUseRotation(), use_rotation);
}

TEST_F(FeatureApriltag_fixture, FeatureApriltagProj_getters)
{
    FeatureApriltagProjPtr fa_proj = std::make_shared<FeatureApriltagProj>(corners_vec, cov_pix, tag_id, tag_width, pose, use_rotation, decision_margin, rep_err1, rep_err2);

    ASSERT_MATRIX_APPROX(fa_proj->getPosePnp(), pose, 1e-6);
}

TEST_F(FeatureApriltag_fixture, FeatureApriltagPose_getters)
{
    FeatureApriltagPosePtr fa_pose = std::make_shared<FeatureApriltagPose>(pose, info_pose, tag_id, tag_width, corners_vec, use_rotation, decision_margin, rep_err1, rep_err2);

    ASSERT_MATRIX_APPROX(fa_pose->getPosePnp(), pose, 1e-6);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}


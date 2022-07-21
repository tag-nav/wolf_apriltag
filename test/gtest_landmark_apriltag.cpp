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
 * \file gtest_landmark_apriltag.cpp
 *
 *  Created on: Dec 6, 2018
 *      \author: jsola
 */


#include <core/utils/utils_gtest.h>


#include "core/common/wolf.h"
#include "core/utils/logging.h"

#include "apriltag/landmark/landmark_apriltag.h"
#include "apriltag/internal/config.h"

using namespace Eigen;
using namespace wolf;
using std::static_pointer_cast;

class LandmarkApriltag_class : public testing::Test{
    public:
        void SetUp() override
        {
            wolf_root = _WOLF_APRILTAG_ROOT_DIR;
            problem = Problem::create("PO", 3);
        }
    public:
        std::string wolf_root;
        ProblemPtr   problem;
};

TEST(LandmarkApriltag, getTagId)
{
    Vector7d p; p << 1,2,3,0,0,0,1;
    LandmarkApriltagPtr l = std::make_shared<LandmarkApriltag>(p, 5, 0.2); // pose, tag_id, tag_width
    ASSERT_EQ(l->getTagId(), 5);
}

TEST(LandmarkApriltag, getTagWidth)
{
    Vector7d p; p << 1,2,3,0,0,0,1;
    LandmarkApriltagPtr l = std::make_shared<LandmarkApriltag>(p, 5, 0.2); // pose, tag_id, tag_width
    ASSERT_EQ(l->getTagWidth(), 0.2);
}

TEST(LandmarkApriltag, getPose)
{
    Vector7d p;
    p << 0,0,0, 0,0,0,1;
    LandmarkApriltagPtr l = std::make_shared<LandmarkApriltag>(p, 5, 0.2); // pose, tag_id, tag_width
    ASSERT_MATRIX_APPROX(l->getState().vector("PO"), p, 1e-6);
}

TEST_F(LandmarkApriltag_class, create)
{
    // load original hand-written map
    problem->loadMap(wolf_root + "/demos/map_apriltag_1.yaml"); // this will call create()
    ASSERT_EQ(problem->getMap()->getLandmarkList().size(), 4);
    ASSERT_EQ(problem->getMap()->getLandmarkList().front()->getType(), "LandmarkApriltag");
}

TEST_F(LandmarkApriltag_class, toYaml)
{
    // load original hand-written map
    problem->loadMap(wolf_root + "/demos/map_apriltag_1.yaml");
    ASSERT_EQ(problem->getMap()->getLandmarkList().size(), 4);

    // write map on new file
    problem->saveMap(wolf_root + "/demos/map_apriltag_save.yaml"); // this will call toYaml()

    // delete existing map
    while(!problem->getMap()->getLandmarkList().empty()) problem->getMap()->getLandmarkList().front()->remove();
    ASSERT_EQ(problem->getMap()->getLandmarkList().size(), 0);

    // reload the saved map
    problem->loadMap(wolf_root + "/demos/map_apriltag_save.yaml");
    ASSERT_EQ(problem->getMap()->getLandmarkList().size(), 4);
    ASSERT_EQ(problem->getMap()->getLandmarkList().front()->getType(), "LandmarkApriltag");
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}


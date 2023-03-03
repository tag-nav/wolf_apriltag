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

#include <core/utils/utils_gtest.h>

#include "apriltag/feature/feature_apriltag.h"

using namespace wolf;

class FeatureApriltag_test : public testing::Test
{
    public:
        Eigen::Vector7d pose;
        Eigen::Matrix6d cov;
        int tag_id;
        apriltag_detection_t det;
        double rep_error1;
        double rep_error2;
        bool use_rotation;

        void SetUp() override
        {
            pose << 1,2,3,4,5,6,7;
            cov.setIdentity() * 2.0;

            det.id      = 1;
            tag_id      = det.id;
            det.p[0][0] =  1.0;
            det.p[0][1] = -1.0;
            det.p[1][0] =  1.0;
            det.p[1][1] =  1.0;
            det.p[2][0] = -1.0;
            det.p[2][1] =  1.0;
            det.p[3][0] = -1.0;
            det.p[3][1] = -1.0;

            rep_error1 = 0.01;
            rep_error2 = 0.1;
            use_rotation = true;
        }
};

TEST_F(FeatureApriltag_test, type)
{
    FeatureApriltagPtr f = std::make_shared<FeatureApriltag>(pose, cov, tag_id, det, rep_error1, rep_error2, use_rotation);

    ASSERT_EQ(f->getType(), "FeatureApriltag");
}

TEST_F(FeatureApriltag_test, getTagId)
{
    FeatureApriltagPtr f = std::make_shared<FeatureApriltag>(pose, cov, tag_id, det, rep_error1, rep_error2, use_rotation);

    ASSERT_EQ(f->getTagId(), 1);
}

TEST_F(FeatureApriltag_test, getCorners)
{
    FeatureApriltagPtr f = std::make_shared<FeatureApriltag>(pose, cov, tag_id, det, rep_error1, rep_error2, use_rotation);

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

TEST_F(FeatureApriltag_test, getDetection)
{
    FeatureApriltagPtr f = std::make_shared<FeatureApriltag>(pose, cov, tag_id, det, rep_error1, rep_error2, use_rotation);

    ASSERT_EQ(f->getDetection().id, 1);
}

TEST_F(FeatureApriltag_test, tagid_detid_equality)
{
    FeatureApriltagPtr f = std::make_shared<FeatureApriltag>(pose, cov, tag_id, det, rep_error1, rep_error2, use_rotation);

    ASSERT_EQ(f->getDetection().id, f->getTagId());
}

TEST_F(FeatureApriltag_test, tagCorners_detection_equality)
{
    FeatureApriltagPtr f = std::make_shared<FeatureApriltag>(pose, cov, tag_id, det, rep_error1, rep_error2, use_rotation);

    for (int i = 0; i<f->getTagCorners().size(); i++)
    {
        ASSERT_EQ(f->getTagCorners()[i].x, f->getDetection().p[i][0]);
        ASSERT_EQ(f->getTagCorners()[i].y, f->getDetection().p[i][1]);
    }
}

TEST_F(FeatureApriltag_test, getRepErrors)
{
    FeatureApriltagPtr f = std::make_shared<FeatureApriltag>(pose, cov, tag_id, det, rep_error1, rep_error2, use_rotation);

    double err1 = f->getRepError1();
    double err2 = f->getRepError2();

    ASSERT_EQ(err1, rep_error1);
    ASSERT_EQ(err2, rep_error2);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}


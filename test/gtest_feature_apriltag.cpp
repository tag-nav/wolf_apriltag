/**
 * \file gtest_feature_apriltag.cpp
 *
 *  Created on: Dec 22, 2018
 *      \author: jsola
 */


#include "utils_gtest.h"

#include "base/feature/feature_apriltag.h"

using namespace wolf;

class FeatureApriltag_test : public testing::Test
{
    public:
        Eigen::Vector7s pose;
        Eigen::Matrix6s cov;
        int tag_id;
        apriltag_detection_t det;

        virtual void SetUp()
        {
            pose << 1,2,3,4,5,6,7;
            cov.setIdentity() * 2.0;

            det.id      = 1;
            det.p[0][0] =  1.0;
            det.p[0][1] = -1.0;
            det.p[1][0] =  1.0;
            det.p[1][1] =  1.0;
            det.p[2][0] = -1.0;
            det.p[2][1] =  1.0;
            det.p[3][0] = -1.0;
            det.p[3][1] = -1.0;

            tag_id      = det.id;
        }
};

TEST_F(FeatureApriltag_test, type)
{
    FeatureApriltag f(pose, cov, tag_id, det);

    ASSERT_EQ(f.getType(), "APRILTAG");
}

TEST_F(FeatureApriltag_test, getTagId)
{
    FeatureApriltag f(pose, cov, tag_id, det);

    ASSERT_EQ(f.getTagId(), 1);
}

TEST_F(FeatureApriltag_test, getCorners)
{
    FeatureApriltag f(pose, cov, tag_id, det);

    ASSERT_EQ(f.getTagCorners().size(), 4);

    ASSERT_EQ(f.getTagCorners()[0].x,  1.0);
    ASSERT_EQ(f.getTagCorners()[0].y, -1.0);
    ASSERT_EQ(f.getTagCorners()[1].x,  1.0);
    ASSERT_EQ(f.getTagCorners()[1].y,  1.0);
    ASSERT_EQ(f.getTagCorners()[2].x, -1.0);
    ASSERT_EQ(f.getTagCorners()[2].y,  1.0);
    ASSERT_EQ(f.getTagCorners()[3].x, -1.0);
    ASSERT_EQ(f.getTagCorners()[3].y, -1.0);
}

TEST_F(FeatureApriltag_test, getDetection)
{
    FeatureApriltag f(pose, cov, tag_id, det);

    ASSERT_EQ(f.getDetection().id, 1);
}

TEST_F(FeatureApriltag_test, tagid_detid_equality)
{
    FeatureApriltag f(pose, cov, tag_id, det);

    ASSERT_EQ(f.getDetection().id, f.getTagId());
}

TEST_F(FeatureApriltag_test, tagCorners_detection_equality)
{
    FeatureApriltag f(pose, cov, tag_id, det);

    for (int i = 0; i<f.getTagCorners().size(); i++)
    {
        ASSERT_EQ(f.getTagCorners()[i].x, f.getDetection().p[i][0]);
        ASSERT_EQ(f.getTagCorners()[i].y, f.getDetection().p[i][1]);
    }
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}


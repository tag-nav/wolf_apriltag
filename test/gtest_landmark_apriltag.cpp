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
        virtual void SetUp()
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
    Vector7s p;
    LandmarkApriltagPtr l = std::make_shared<LandmarkApriltag>(p, 5, 0.2); // pose, tag_id, tag_width
    ASSERT_EQ(l->getTagId(), 5);
}

TEST(LandmarkApriltag, getTagWidth)
{
    Vector7s p;
    LandmarkApriltagPtr l = std::make_shared<LandmarkApriltag>(p, 5, 0.2); // pose, tag_id, tag_width
    ASSERT_EQ(l->getTagWidth(), 0.2);
}

TEST(LandmarkApriltag, getPose)
{
    Vector7s p;
    p << 0,0,0, 0,0,0,1;
    LandmarkApriltagPtr l = std::make_shared<LandmarkApriltag>(p, 5, 0.2); // pose, tag_id, tag_width
    ASSERT_MATRIX_APPROX(l->getState(), p, 1e-6);
}

TEST_F(LandmarkApriltag_class, create)
{
    // load original hand-written map
    problem->loadMap(wolf_root + "/demos/map_apriltag_1.yaml"); // this will call create()
    ASSERT_EQ(problem->getMap()->getLandmarkList().size(), 4);
    ASSERT_EQ(problem->getMap()->getLandmarkList().front()->getType(), "APRILTAG");
}

TEST_F(LandmarkApriltag_class, saveToYaml)
{
    // load original hand-written map
    problem->loadMap(wolf_root + "/demos/map_apriltag_1.yaml");
    ASSERT_EQ(problem->getMap()->getLandmarkList().size(), 4);

    // write map on new file
    problem->saveMap(wolf_root + "/demos/map_apriltag_save.yaml"); // this will call saveToYaml()

    // delete existing map
    while(!problem->getMap()->getLandmarkList().empty()) problem->getMap()->getLandmarkList().front()->remove();
    ASSERT_EQ(problem->getMap()->getLandmarkList().size(), 0);

    // reload the saved map
    problem->loadMap(wolf_root + "/demos/map_apriltag_save.yaml");
    ASSERT_EQ(problem->getMap()->getLandmarkList().size(), 4);
    ASSERT_EQ(problem->getMap()->getLandmarkList().front()->getType(), "APRILTAG");
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}


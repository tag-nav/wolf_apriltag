/*
 * gtest_capture_base.cpp
 *
 *  Created on: Apr 11, 2017
 *      Author: jsola
 */

#include "utils_gtest.h"

#include "base/capture/capture_base.h"
#include "base/state_angle.h"

using namespace wolf;
using namespace Eigen;

TEST(CaptureBase, ConstructorNoSensor)
{
    CaptureBasePtr C(std::make_shared<CaptureBase>("DUMMY", 1.2)); // timestamp = 1.2

    ASSERT_EQ(C->getTimeStamp(), 1.2);
    ASSERT_FALSE(C->getFrame());
    ASSERT_FALSE(C->getProblem());
    ASSERT_FALSE(C->getSensor());
}

TEST(CaptureBase, ConstructorWithSensor)
{
    SensorBasePtr S(std::make_shared<SensorBase>("DUMMY", nullptr, nullptr, nullptr, 2));
    CaptureBasePtr C(std::make_shared<CaptureBase>("DUMMY", 1.5, S)); // timestamp = 1.5
    ASSERT_EQ(C->getTimeStamp(), 1.5);
    ASSERT_FALSE(C->getFrame());
    ASSERT_FALSE(C->getProblem());
    ASSERT_TRUE(C->getSensor());
    ASSERT_FALSE(C->getSensorP());
    ASSERT_FALSE(C->getSensorO());
}

TEST(CaptureBase, Static_sensor_params)
{
    StateBlockPtr p(std::make_shared<StateBlock>(2));
    StateBlockPtr o(std::make_shared<StateAngle>() );
    StateBlockPtr i(std::make_shared<StateBlock>(2));
    SensorBasePtr S(std::make_shared<SensorBase>("DUMMY", p, o, i, 2));
    CaptureBasePtr C(std::make_shared<CaptureBase>("DUMMY", 1.5, S)); // timestamp = 1.5

    // query sensor blocks
    ASSERT_EQ(S->getP(), p);
    ASSERT_EQ(S->getO(), o);
    ASSERT_EQ(S->getIntrinsic(), i);

    // query capture blocks
    ASSERT_EQ(C->getSensorP(), p);
    ASSERT_EQ(C->getSensorO(), o);
    ASSERT_EQ(C->getSensorIntrinsic(), i);
}

TEST(CaptureBase, Dynamic_sensor_params)
{
    StateBlockPtr p(std::make_shared<StateBlock>(2));
    StateBlockPtr o(std::make_shared<StateAngle>() );
    StateBlockPtr i(std::make_shared<StateBlock>(2));
    SensorBasePtr S(std::make_shared<SensorBase>("DUMMY", nullptr, nullptr, nullptr, 2, true, true)); // last 2 'true' mark dynamic
    CaptureBasePtr C(std::make_shared<CaptureBase>("DUMMY", 1.5, S, p, o, i)); // timestamp = 1.5

    // query sensor blocks -- need KFs to find some Capture with the params
    //    ASSERT_EQ(S->getP(), p);
    //    ASSERT_EQ(S->getO(), o);
    //    ASSERT_EQ(S->getIntrinsic(), i);

    // query capture blocks
    ASSERT_EQ(C->getSensorP(), p);
    ASSERT_EQ(C->getSensorO(), o);
    ASSERT_EQ(C->getSensorIntrinsic(), i);
}

TEST(CaptureBase, addFeature)
{
    CaptureBasePtr C(std::make_shared<CaptureBase>("DUMMY", 1.2)); // timestamp = 1.2
    FeatureBasePtr f = C->addFeature(std::make_shared<FeatureBase>("DUMMY", Vector2s::Zero(), Matrix2s::Identity()));
    ASSERT_FALSE(C->getFeatureList().empty());
    ASSERT_EQ(C->getFeatureList().front(), f);
}

TEST(CaptureBase, addFeatureList)
{
    CaptureBasePtr C(std::make_shared<CaptureBase>("DUMMY", 1.2)); // timestamp = 1.2
    FeatureBasePtr f_first = C->addFeature(std::make_shared<FeatureBase>("DUMMY", Vector2s::Zero(), Matrix2s::Identity()));
    ASSERT_EQ(C->getFeatureList().size(), (unsigned int) 1);

    // make a list to add
    std::list<FeatureBasePtr> fl;
    for (int i = 0; i<3; i++)
    {
        fl.push_back(std::make_shared<FeatureBase>("DUMMY", Vector2s::Zero(), Matrix2s::Identity()));
    }
    FeatureBasePtr f_last = fl.back();

    C->addFeatureList((fl));
    ASSERT_EQ(C->getFeatureList().size(), (unsigned int) 4);
//    ASSERT_EQ(fl.size(), (unsigned int) 0); // features have been moved, not copied // EDIT 02-2019: features have been copied
    ASSERT_EQ(C->getFeatureList().front(), f_first);
    ASSERT_EQ(C->getFeatureList().back(), f_last);
}

TEST(CaptureBase, process)
{
    SensorBasePtr S(std::make_shared<SensorBase>("DUMMY", nullptr, nullptr, nullptr, 2));
    CaptureBasePtr C(std::make_shared<CaptureBase>("DUMMY", 1.5, nullptr));
    ASSERT_DEATH({C->process();},""); // No sensor in the capture should fail
    C->setSensor(S);
    ASSERT_TRUE(C->process()); // This should not fail (although it does nothing)
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}


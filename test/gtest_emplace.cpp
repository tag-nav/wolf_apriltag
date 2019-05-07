/*
 * gtest_emplace.cpp
 *
 *  Created on: Mar 20, 2019
 *      Author: jcasals
 */

#include "utils_gtest.h"
#include "core/utils/logging.h"

#include "core/problem/problem.h"
#include "core/capture/capture_IMU.h"
#include "core/sensor/sensor_base.h"
#include "core/sensor/sensor_odom_3D.h"
#include "core/sensor/sensor_IMU.h"
#include "core/processor/processor_odom_3D.h"
#include "core/processor/processor_odom_2D.h"
#include "core/feature/feature_odom_2D.h"
#include "core/feature/feature_IMU.h"
#include "core/processor/processor_tracker_feature_dummy.h"

#include <iostream>

using namespace wolf;
using namespace Eigen;

TEST(Emplace, Landmark)
{
    ProblemPtr P = Problem::create("POV", 3);

    LandmarkBase::emplace<LandmarkBase>(P->getMap(),"Dummy", nullptr, nullptr);
    ASSERT_EQ(P, P->getMap()->getLandmarkList().front()->getMap()->getProblem());
}

TEST(Emplace, Sensor)
{
    ProblemPtr P = Problem::create("POV", 3);

    SensorBase::emplace<SensorBase>(P->getHardware(), "Dummy", nullptr, nullptr, nullptr, 2, false);
    ASSERT_EQ(P, P->getHardware()->getSensorList().front()->getHardware()->getProblem());
}
TEST(Emplace, Frame)
{
    ProblemPtr P = Problem::create("POV", 3);

    ASSERT_NE(P->getTrajectory(), nullptr);
    FrameBase::emplace<FrameBase>(P->getTrajectory(),  KEY, TimeStamp(0), std::make_shared<StateBlock>(2,true), std::make_shared<StateBlock>(2,true));
    ASSERT_EQ(P, P->getTrajectory()->getFrameList().front()->getTrajectory()->getProblem());
}

TEST(Emplace, Processor)
{
    ProblemPtr P = Problem::create("POV", 3);

    auto sen = SensorBase::emplace<SensorBase>(P->getHardware(), "Dummy", nullptr, nullptr, nullptr, 2, false);
    auto prc = ProcessorOdom2D::emplace<ProcessorOdom2D>(sen, std::make_shared<ProcessorParamsOdom2D>());
    ASSERT_EQ(P, P->getHardware()->getSensorList().front()->getProcessorList().front()->getSensor()->getProblem());
    ASSERT_EQ(sen, sen->getProcessorList().front()->getSensor());
    ASSERT_EQ(prc, sen->getProcessorList().front());
}

TEST(Emplace, Capture)
{
    ProblemPtr P = Problem::create("POV", 3);

    ASSERT_NE(P->getTrajectory(), nullptr);
    auto frm = FrameBase::emplace<FrameBase>(P->getTrajectory(),  KEY, TimeStamp(0), std::make_shared<StateBlock>(2,true), std::make_shared<StateBlock>(2,true));
    ASSERT_EQ(P, P->getTrajectory()->getFrameList().front()->getTrajectory()->getProblem());

    auto cpt = CaptureBase::emplace<CaptureBase>(frm, "Dummy", TimeStamp(0), nullptr, nullptr, nullptr, nullptr);
    ASSERT_EQ(P, P->getTrajectory()->getFrameList().front()->getCaptureList().front()->getFrame()->getTrajectory()->getProblem());
    ASSERT_EQ(P, P->getTrajectory()->getFrameList().front()->getCaptureList().front()->getProblem());
    ASSERT_EQ(frm, frm->getCaptureList().front()->getFrame());
}

TEST(Emplace, Feature)
{
    ProblemPtr P = Problem::create("POV", 3);

    ASSERT_NE(P->getTrajectory(), nullptr);
    auto frm = FrameBase::emplace<FrameBase>(P->getTrajectory(),  KEY, TimeStamp(0), std::make_shared<StateBlock>(2,true), std::make_shared<StateBlock>(2,true));
    ASSERT_EQ(P, P->getTrajectory()->getFrameList().front()->getTrajectory()->getProblem());

    auto cpt = CaptureBase::emplace<CaptureBase>(frm, "Dummy", TimeStamp(0), nullptr, nullptr, nullptr, nullptr);
    ASSERT_EQ(P, P->getTrajectory()->getFrameList().front()->getCaptureList().front()->getFrame()->getTrajectory()->getProblem());
    ASSERT_EQ(P, P->getTrajectory()->getFrameList().front()->getCaptureList().front()->getProblem());
    ASSERT_EQ(frm, frm->getCaptureList().front()->getFrame());
    auto cov = Eigen::MatrixXs(2,2);
    cov(0,0) = 1;
    cov(1,1) = 1;
    FeatureBase::emplace<FeatureBase>(cpt, "Dummy", Eigen::VectorXs(5), cov);
    ASSERT_EQ(P, P->getTrajectory()->getFrameList().front()->getCaptureList().front()->getFeatureList().front()->getCapture()->getFrame()->getTrajectory()->getProblem());
    ASSERT_EQ(P, P->getTrajectory()->getFrameList().front()->getCaptureList().front()->getFeatureList().front()->getProblem());
    ASSERT_EQ(cpt, cpt->getFeatureList().front()->getCapture());
}
TEST(Emplace, Factor)
{
    ProblemPtr P = Problem::create("POV", 3);

    ASSERT_NE(P->getTrajectory(), nullptr);
    auto frm = FrameBase::emplace<FrameBase>(P->getTrajectory(),  KEY, TimeStamp(0), std::make_shared<StateBlock>(2,true), std::make_shared<StateBlock>(2,true));
    ASSERT_EQ(P, P->getTrajectory()->getFrameList().front()->getTrajectory()->getProblem());

    auto cpt = CaptureBase::emplace<CaptureBase>(frm, "Dummy", TimeStamp(0), nullptr, nullptr, nullptr, nullptr);
    ASSERT_EQ(P, P->getTrajectory()->getFrameList().front()->getCaptureList().front()->getFrame()->getTrajectory()->getProblem());
    ASSERT_EQ(P, P->getTrajectory()->getFrameList().front()->getCaptureList().front()->getProblem());
    ASSERT_EQ(frm, frm->getCaptureList().front()->getFrame());
    auto cov = Eigen::MatrixXs(2,2);
    cov(0,0) = 1;
    cov(1,1) = 1;
    auto ftr = FeatureBase::emplace<FeatureOdom2D>(cpt, Eigen::VectorXs(5), cov);
    ASSERT_EQ(P, P->getTrajectory()->getFrameList().front()->getCaptureList().front()->getFeatureList().front()->getCapture()->getFrame()->getTrajectory()->getProblem());
    ASSERT_EQ(P, P->getTrajectory()->getFrameList().front()->getCaptureList().front()->getFeatureList().front()->getProblem());
    ASSERT_EQ(cpt, cpt->getFeatureList().front()->getCapture());
    auto cnt = FactorBase::emplace<FactorOdom2D>(ftr, ftr, frm);
    ASSERT_NE(nullptr, ftr->getFactorList().front().get());
}

TEST(Emplace, EmplaceDerived)
{
    ProblemPtr P = Problem::create("POV", 3);

    auto frm = FrameBase::emplace<FrameBase>(P->getTrajectory(),  KEY, TimeStamp(0), std::make_shared<StateBlock>(2,true), std::make_shared<StateBlock>(2,true));
    // LandmarkBase::emplace<LandmarkBase>(MapBaseWPtr(P->getMap()),"Dummy", nullptr, nullptr);
    auto sen = SensorBase::emplace<SensorIMU>(P->getHardware(), Eigen::VectorXs(7), IntrinsicsIMU());
    auto cov = Eigen::MatrixXs(2,2);
    cov(0,0) = 1;
    cov(1,1) = 1;
    auto cpt = CaptureBase::emplace<CaptureIMU>(frm, TimeStamp(0), sen, Eigen::Vector6s(), cov,
                                                Eigen::Vector6s(), frm);
    auto cpt2 = std::static_pointer_cast<CaptureIMU>(cpt);
    auto m = Eigen::Matrix<Scalar,9,6>();
    for(int i = 0; i < 9; i++)
        for(int j = 0; j < 6; j++)
            m(i,j) = 1;

    auto ftr = FeatureBase::emplace<FeatureIMU>(cpt, Eigen::VectorXs(5), cov,
                                                Eigen::Vector6s(), m, cpt2);
    ASSERT_EQ(sen, P->getHardware()->getSensorList().front());
    ASSERT_EQ(P, P->getTrajectory()->getFrameList().front()->getCaptureList().front()->getFeatureList().front()->getProblem());
}
TEST(Emplace, Nullpointer)
{
    
    // incomming_dummy = wolf::CaptureBase::emplace<wolf::CaptureBase>(nullptr, "DUMMY", 1.2, sensor_ptr);
}
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
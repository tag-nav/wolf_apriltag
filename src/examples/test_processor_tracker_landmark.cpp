/**
 * \file test_processor_tracker_landmark.cpp
 *
 *  Created on: Apr 12, 2016
 *      \author: jvallve
 */

//std
#include <iostream>

//Wolf
#include "core/common/wolf.h"
#include "core/problem/problem.h"
#include "core/sensor/sensor_base.h"
#include "core/state_block/state_block.h"
#include "core/processor/processor_tracker_landmark_dummy.h"
#include "core/capture/capture_void.h"

void print_problem_pointers(wolf::ProblemPtr wolf_problem_ptr_)
{
    using namespace wolf;
    std::cout << "\n-----------\nWolf tree begin" << std::endl;
    std::cout << "Hrd: " << wolf_problem_ptr_->getHardware()->getProblem() << std::endl;
    for (SensorBasePtr sen : wolf_problem_ptr_->getHardware()->getSensorList())
    {
        std::cout << "\tSen: " << sen->getProblem() << std::endl;
        for (ProcessorBasePtr prc : sen->getProcessorList())
        {
            std::cout << "\t\tPrc: " << prc->getProblem() << std::endl;
        }
    }
    std::cout << "Trj: " << wolf_problem_ptr_->getTrajectory()->getProblem() << std::endl;
    for (FrameBasePtr frm : wolf_problem_ptr_->getTrajectory()->getFrameList())
    {
        std::cout << "\tFrm: " << frm->getProblem() << std::endl;
        for (CaptureBasePtr cap : frm->getCaptureList())
        {
            std::cout << "\t\tCap: " << cap->getProblem() << std::endl;
            for (FeatureBasePtr ftr : cap->getFeatureList())
            {
                std::cout << "\t\t\tFtr: " << ftr->getProblem() << std::endl;
                for (FactorBasePtr ctr : ftr->getFactorList())
                {
                    std::cout << "\t\t\t\tCtr: " << ctr->getProblem() << std::endl;
                }
            }
        }
    }
    std::cout << "Map: " << wolf_problem_ptr_->getMap()->getProblem() << std::endl;
    for (LandmarkBasePtr lmk : wolf_problem_ptr_->getMap()->getLandmarkList())
    {
        std::cout << "\tLmk: " << lmk->getProblem() << std::endl;
    }
    std::cout << "Wolf tree end\n-----------\n" << std::endl;
}

int main()
{
    using namespace wolf;

    std::cout << std::endl << "==================== processor tracker landmark test ======================" << std::endl;

    // Wolf problem
    ProblemPtr wolf_problem_ptr_ = Problem::create("PO", 2);
    // SensorBasePtr sensor_ptr_ = std::make_shared< SensorBase>("ODOM 2D", std::make_shared<StateBlock>(Eigen::VectorXs::Zero(2)),
    //                                          std::make_shared<StateBlock>(Eigen::VectorXs::Zero(1)),
    //                                          std::make_shared<StateBlock>(Eigen::VectorXs::Zero(2)), 2);
    auto sensor_ptr_ = SensorBase::emplace<SensorBase>(wolf_problem_ptr_->getHardware(), "ODOM 2D", std::make_shared<StateBlock>(Eigen::VectorXs::Zero(2)),
                                                       std::make_shared<StateBlock>(Eigen::VectorXs::Zero(1)),
                                                       std::make_shared<StateBlock>(Eigen::VectorXs::Zero(2)), 2);
    ProcessorParamsTrackerLandmarkPtr params_trk = std::make_shared<ProcessorParamsTrackerLandmark>();
    params_trk->max_new_features = 5;
    params_trk->min_features_for_keyframe = 7;
    params_trk->time_tolerance = 0.25;
    // std::shared_ptr<ProcessorTrackerLandmarkDummy> processor_ptr_ = std::make_shared< ProcessorTrackerLandmarkDummy>(params_trk);
    std::shared_ptr<ProcessorTrackerLandmarkDummy> processor_ptr_ =
        std::static_pointer_cast<ProcessorTrackerLandmarkDummy>(ProcessorBase::emplace<ProcessorTrackerLandmarkDummy>(sensor_ptr_, params_trk));
    // wolf_problem_ptr_->addSensor(sensor_ptr_);
    // sensor_ptr_->addProcessor(processor_ptr_);

    std::cout << "sensor & processor created and added to wolf problem" << std::endl;

    TimeStamp t(0);
    Scalar dt = 0.5;
    for (auto i = 0; i < 10; i++)
    {
        processor_ptr_->process(std::make_shared<CaptureVoid>(t, sensor_ptr_));
        t += dt;
    }

    wolf_problem_ptr_->print(2);

    return 0;
}


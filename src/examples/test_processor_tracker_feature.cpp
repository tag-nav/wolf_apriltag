/**
 * \file test_processor_tracker_feature.cpp
 *
 *  Created on: Apr 11, 2016
 *      \author: jvallve
 */

//std
#include <iostream>

//Wolf
#include "core/common/wolf.h"
#include "core/problem/problem.h"
#include "core/sensor/sensor_base.h"
#include "core/state_block/state_block.h"
#include "core/processor/processor_tracker_feature_dummy.h"
#include "core/capture/capture_void.h"

int main()
{
    using namespace wolf;
    using std::shared_ptr;
    using std::make_shared;
    using std::static_pointer_cast;

    std::cout << std::endl << "==================== processor tracker feature test ======================" << std::endl;

    // Wolf problem
    ProblemPtr wolf_problem_ptr_ = Problem::create("PO 2D");
    SensorBasePtr sensor_ptr_ = make_shared<SensorBase>("ODOM 2D", std::make_shared<StateBlock>(Eigen::VectorXs::Zero(2)),
                                             std::make_shared<StateBlock>(Eigen::VectorXs::Zero(1)),
                                             std::make_shared<StateBlock>(Eigen::VectorXs::Zero(2)), 2);

    ProcessorParamsTrackerFeaturePtr params_trk = std::make_shared<ProcessorParamsTrackerFeature>();
    params_trk->max_new_features = 4;
    params_trk->min_features_for_keyframe = 7;
    params_trk->time_tolerance = 0.25;
    shared_ptr<ProcessorTrackerFeatureDummy> processor_ptr_ = make_shared<ProcessorTrackerFeatureDummy>(params_trk);

    wolf_problem_ptr_->addSensor(sensor_ptr_);
    sensor_ptr_->addProcessor(processor_ptr_);

    std::cout << "sensor & processor created and added to wolf problem" << std::endl;

    TimeStamp t(0);
    Scalar dt = 0.5;
    for (auto i = 0; i < 10; i++)
    {
        processor_ptr_->process(make_shared<CaptureVoid>(t, sensor_ptr_));
        t += dt;
    }

    wolf_problem_ptr_->print(2);

    return 0;
}


/**
 * \file test_processor_tracker_feature.cpp
 *
 *  Created on: Apr 11, 2016
 *      \author: jvallve
 */

//std
#include <iostream>

//Wolf
#include "wolf.h"
#include "problem.h"
#include "sensor_base.h"
#include "state_block.h"
#include "processor_tracker_feature_dummy.h"
#include "capture_void.h"

int main()
{
    using namespace wolf;
    using std::shared_ptr;
    using std::make_shared;
    using std::static_pointer_cast;

    std::cout << std::endl << "==================== processor tracker feature test ======================" << std::endl;

    // Wolf problem
    ProblemPtr wolf_problem_ptr_ = Problem::create(FRM_PO_2D);
    SensorBasePtr sensor_ptr_ = make_shared<SensorBase>(SEN_ODOM_2D, "ODOM 2D", std::make_shared<StateBlock>(Eigen::VectorXs::Zero(2)),
                                             std::make_shared<StateBlock>(Eigen::VectorXs::Zero(1)),
                                             std::make_shared<StateBlock>(Eigen::VectorXs::Zero(2)), 2);

    shared_ptr<ProcessorTrackerFeatureDummy> processor_ptr_ = make_shared<ProcessorTrackerFeatureDummy>();

    wolf_problem_ptr_->addSensor(sensor_ptr_);
    sensor_ptr_->addProcessor(processor_ptr_);

    std::cout << "sensor & processor created and added to wolf problem" << std::endl;

    for (auto i = 0; i < 10; i++)
        processor_ptr_->process(make_shared<CaptureVoid>(TimeStamp(0), sensor_ptr_));

    wolf_problem_ptr_->check();
    wolf_problem_ptr_->print();

    return 0;
}


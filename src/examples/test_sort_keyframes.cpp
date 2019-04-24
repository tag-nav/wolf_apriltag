/**
 * \file test_sort_keyframes.cpp
 *
 *  Created on: May 24, 2016
 *      \author: jvallve
 */

// Wolf includes
#include "base/common/wolf.h"
#include "base/problem/problem.h"
#include "base/frame/frame_base.h"
#include "base/trajectory/trajectory_base.h"

// STL includes
#include <list>
#include <memory>

// General includes
#include <iostream>

using namespace wolf;

void printFrames(ProblemPtr _problem_ptr)
{
    std::cout << "TRAJECTORY:" << std::endl;
    for (auto frm : _problem_ptr->getTrajectory()->getFrameList())
        std::cout << "\t " << (frm->isKey() ? "KEY FRAME: " : "FRAME: ") << frm->id() << " - TS: " << frm->getTimeStamp().getSeconds() << "." << frm->getTimeStamp().getNanoSeconds() << std::endl;
}

int main()
{
    ProblemPtr problem_ptr = Problem::create(FRM_PO_2D);

    problem_ptr->emplaceFrame(NON_KEY_FRAME, Eigen::VectorXs::Zero(3), TimeStamp(0.1));
    FrameBasePtr frm2 = problem_ptr->emplaceFrame(NON_KEY_FRAME, Eigen::VectorXs::Zero(3), TimeStamp(0.2));
    FrameBasePtr frm3 = problem_ptr->emplaceFrame(NON_KEY_FRAME, Eigen::VectorXs::Zero(3), TimeStamp(0.3));
    problem_ptr->emplaceFrame(NON_KEY_FRAME, Eigen::VectorXs::Zero(3), TimeStamp(0.4));
    FrameBasePtr frm5 = problem_ptr->emplaceFrame(NON_KEY_FRAME, Eigen::VectorXs::Zero(3), TimeStamp(0.5));
    FrameBasePtr frm6 = problem_ptr->emplaceFrame(NON_KEY_FRAME, Eigen::VectorXs::Zero(3), TimeStamp(0.6));

    printFrames(problem_ptr);

    std::cout << std::endl << "Frame " << frm5->id() << " set KEY" << std::endl;
    frm5->setKey();

    printFrames(problem_ptr);

    std::cout << std::endl << "Frame " << frm2->id() << " set KEY" << std::endl;
    frm2->setKey();

    printFrames(problem_ptr);

    std::cout << std::endl << "Frame " << frm3->id() << " new TimeStamp:" << 0.45 << std::endl;
    frm3->setTimeStamp(TimeStamp(0.45));

    printFrames(problem_ptr);

    std::cout << std::endl << "Frame " << frm3->id() << " set KEY" << std::endl;
    frm3->setKey();

    printFrames(problem_ptr);

    std::cout << std::endl << "Frame " << frm6->id() << " set KEY" << std::endl;
    frm6->setKey();

    printFrames(problem_ptr);

    FrameBasePtr frm7 = problem_ptr->emplaceFrame(KEY_FRAME, Eigen::VectorXs::Zero(3), TimeStamp(0.7));
    std::cout << std::endl << "created Key Frame " << frm7->id() << " TS: " << 0.7 << std::endl;

    printFrames(problem_ptr);

    FrameBasePtr frm8 = problem_ptr->emplaceFrame(KEY_FRAME, Eigen::VectorXs::Zero(3), TimeStamp(0.35));
    std::cout << std::endl << "created Key Frame " << frm8->id() << " TS: " << 0.35 << std::endl;

    printFrames(problem_ptr);

    return 0;
}

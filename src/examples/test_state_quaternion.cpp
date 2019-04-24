/*
 * \file test_state_quaternion.cpp
 *
 *  Created on: Mar 7, 2016
 *      \author: jsola
 */

#include "base/frame/frame_base.h"
#include "base/state_block/state_quaternion.h"
#include "base/common/time_stamp.h"

#include <iostream>

int main (void)
{
    using namespace wolf;

    // 3D
    StateBlockPtr pp = std::make_shared<StateBlock>(3);
    StateQuaternionPtr op = std::make_shared<StateQuaternion>();
    StateBlockPtr vp = std::make_shared<StateBlock>(3);

    TimeStamp t;

    FrameBase pqv(t,pp,op,vp);

    std::cout << "P local param: " << pqv.getP()->getLocalParametrization() << std::endl;
    std::cout << "Q local param: " << pqv.getO()->getLocalParametrization() << std::endl;
    std::cout << "V local param: " << pqv.getV()->getLocalParametrization() << std::endl;

    //    delete pp;
    //    delete op;
    //    delete vp;
    // Note: Deleting the StateBlock pointers will be done at the destruction of FrameBase.

    std::cout << "Done" << std::endl;

    return 1;
}

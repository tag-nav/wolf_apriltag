/*
 * \file StateQuaternion.h
 *
 *  Created on: Mar 7, 2016
 *      author: jsola
 */

#ifndef SRC_STATE_QUATERNION_H_
#define SRC_STATE_QUATERNION_H_

#include "state_block.h"

namespace wolf {

class StateQuaternion : public StateBlock
{
    public:
        StateQuaternion(bool _fixed = false);
        StateQuaternion(const Eigen::VectorXs _state, bool _fixed = false);
        StateQuaternion(const Eigen::Quaternions _quaternion, bool _fixed = false) :
            StateBlock(_quaternion.coeffs())
        {
        }
        virtual ~StateQuaternion();
};

} // namespace wolf

#endif /* SRC_STATE_QUATERNION_H_ */

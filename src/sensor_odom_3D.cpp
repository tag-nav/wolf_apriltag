/**
 * \file sensor_odom_3D.cpp
 *
 *  Created on: Oct 7, 2016
 *      \author: jsola
 */

#include "sensor_odom_3D.h"

#include "state_block.h"
#include "state_quaternion.h"

namespace wolf {

SensorOdom3D::SensorOdom3D(StateBlockPtr _p_ptr, StateQuaternionPtr _o_ptr, const Scalar& _k_disp_to_disp, const Scalar& _k_disp_to_rot, const Scalar&  _k_rot_to_rot) :
        SensorBase(SEN_ODOM_3D, "ODOM 3D", _p_ptr, _o_ptr, nullptr, 6), k_disp_to_disp_(_k_disp_to_disp), k_disp_to_rot_(_k_disp_to_rot), k_rot_to_rot_(_k_rot_to_rot)
{
    //
}

SensorOdom3D::~SensorOdom3D()
{
    //
}

Scalar SensorOdom3D::getDispVarToDispNoiseFactor() const
{
    return k_disp_to_disp_;
}

Scalar SensorOdom3D::getDispVarToRotNoiseFactor() const
{
    return k_disp_to_rot_;
}

Scalar SensorOdom3D::getRotVarToRotNoiseFactor() const
{
    return k_rot_to_rot_;
}

// Define the factory method
SensorBasePtr SensorOdom3D::create(const std::string& _unique_name, const Eigen::VectorXs& _extrinsics_po,
                                 const IntrinsicsBasePtr _intrinsics)
{
    // decode extrinsics vector
    assert(_extrinsics_po.size() == 7 && "Bad extrinsics vector length. Should be 7 for 3D.");
    StateBlockPtr pos_ptr = std::make_shared<StateBlock>(_extrinsics_po.head(3), true);
    StateQuaternionPtr ori_ptr = std::make_shared<StateQuaternion>(_extrinsics_po.tail(4), true);
    // cast intrinsics into derived type
//    IntrinsicsOdom3D* params = (IntrinsicsOdom3D*)(_intrinsics);
//    params->k_disp_to_disp = 1.0;
//    params->k_disp_to_rot = 0.0;
//    params->k_rot_to_rot = 1.0;
    SensorBasePtr odo = std::make_shared<SensorOdom3D>(pos_ptr, ori_ptr, 0.1, 0.1, 0.1);
    odo->setName(_unique_name);
    return odo;
}

} // namespace wolf


// Register in the SensorFactory
#include "sensor_factory.h"
namespace wolf {
WOLF_REGISTER_SENSOR("ODOM 3D", SensorOdom3D)
}

#include "sensor_imu.h"

#include "state_block.h"
#include "state_quaternion.h"

namespace wolf {

SensorIMU::SensorIMU(StateBlock* _p_ptr, StateBlock* _o_ptr, StateBlock* _a_w_biases_ptr) :
//                SensorBase(SEN_IMU, "IMU", _p_ptr, _o_ptr, (_a_w_biases_ptr == nullptr) ? new StateBlock(6, false) : _a_w_biases_ptr, 6)
SensorBase(SEN_IMU, "IMU", _p_ptr, _o_ptr, _a_w_biases_ptr, 6)
{
    //
}

SensorIMU::~SensorIMU()
{
    //
}

// Define the factory method
SensorBasePtr SensorIMU::create(const std::string& _unique_name, const Eigen::VectorXs& _extrinsics_pq,
                              const IntrinsicsBasePtr _intrinsics)
{
    // decode extrinsics vector
    assert(_extrinsics_pq.size() == 7 && "Bad extrinsics vector length. Should be 7 for 3D.");
    StateBlock* pos_ptr = new StateBlock(_extrinsics_pq.head(3), true);
    StateBlock* ori_ptr = new StateQuaternion(_extrinsics_pq.tail(4), true);

    // cast instrinsics to good type and extract intrinsic vector
    //    IntrinsicsIMU* intrinsics = (IntrinsicsIMU*)((_intrinsics));
    StateBlock* bias_ptr = new StateBlock(6, false); // We'll have the IMU biases here
    SensorBasePtr sen = new SensorIMU(pos_ptr, ori_ptr, bias_ptr);
    sen->setName(_unique_name);
    return sen;
}

} // namespace wolf


// Register in the SensorFactory
#include "sensor_factory.h"
//#include "factory.h"
namespace wolf {
namespace
{
const bool registered_imu = SensorFactory::get().registerCreator("IMU", SensorIMU::create);
}
} // namespace wolf

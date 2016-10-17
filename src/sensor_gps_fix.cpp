#include "sensor_gps_fix.h"
#include "state_block.h"
#include "state_quaternion.h"

namespace wolf {

SensorGPSFix::SensorGPSFix(StateBlock* _p_ptr, StateBlock* _o_ptr, const double& _noise) :
        SensorBase(SEN_GPS_FIX, "GPS FIX", _p_ptr, _o_ptr, nullptr, Eigen::VectorXs::Constant(1,_noise))
{
    //
}

SensorGPSFix::~SensorGPSFix()
{
    //
}

Scalar SensorGPSFix::getNoise() const
{
    return noise_std_(0);
}

// Define the factory method
SensorBasePtr SensorGPSFix::create(const std::string& _unique_name, const Eigen::VectorXs& _extrinsics,
                                 const IntrinsicsBasePtr _intrinsics)
{
    assert((_extrinsics.size() == 2 || _extrinsics.size() == 3)
            && "Bad extrinsic vector size. Should be 2 for 2D, 3 for 3D.");
    StateBlock* pos_ptr = new StateBlock(_extrinsics, true);
    //    SensorGPSFix* sen = new SensorGPSFix(pos_ptr, nullptr, 0); // TODO remove line
    std::shared_ptr<SensorGPSFix> sen = std::make_shared<SensorGPSFix>(pos_ptr, nullptr, 0);
    sen->setName(_unique_name);
    return sen;
}

} // namespace wolf


// Register in the SensorFactory
#include "sensor_factory.h"
namespace wolf {
WOLF_REGISTER_SENSOR("GPS FIX", SensorGPSFix)
} // namespace wolf

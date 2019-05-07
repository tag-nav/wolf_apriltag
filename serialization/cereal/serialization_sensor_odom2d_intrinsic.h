#ifndef _WOLF_SERIALIZATION_CEREAL_SENSOR_ODOM2D_INTRINSIC_H_
#define _WOLF_SERIALIZATION_CEREAL_SENSOR_ODOM2D_INTRINSIC_H_

// Wolf includes
#include "core/sensor/sensor_odom_2D.h"

#include "serialization_sensor_intrinsic_base.h"

namespace cereal {

template <class Archive>
void serialize(Archive& ar, wolf::IntrinsicsOdom2D& o,
               std::uint32_t const /*version*/)
{
  ar( cereal::make_nvp("IntrinsicsBase",
        cereal::base_class<wolf::IntrinsicsBase>(&o)) );

  ar( cereal::make_nvp("k_disp_to_disp", o.k_disp_to_disp) );
  ar( cereal::make_nvp("k_rot_to_rot",   o.k_rot_to_rot)   );
}

} // namespace cereal

CEREAL_REGISTER_TYPE_WITH_NAME(wolf::IntrinsicsOdom2D, "IntrinsicsOdom2D")

#endif /* _WOLF_SERIALIZATION_CEREAL_SENSOR_ODOM2D_INTRINSIC_H_ */

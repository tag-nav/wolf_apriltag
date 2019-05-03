#ifndef _WOLF_SERIALIZATION_CEREAL_SENSOR_INTRINSIC_BASE_H_
#define _WOLF_SERIALIZATION_CEREAL_SENSOR_INTRINSIC_BASE_H_

// Wolf includes
#include "core/sensor/sensor_base.h"

#include <cereal/cereal.hpp>
#include <cereal/types/polymorphic.hpp>

namespace cereal {

template <class Archive>
void serialize(Archive& ar, wolf::IntrinsicsBase& o,
               std::uint32_t const /*version*/)
{
  ar( cereal::make_nvp("type", o.type) );
  ar( cereal::make_nvp("name", o.name) );
}

} // namespace cereal

// No need to register base
//CEREAL_REGISTER_TYPE_WITH_NAME(wolf::IntrinsicsBase, "IntrinsicsBase");

#endif /* _WOLF_SERIALIZATION_CEREAL_SENSOR_INTRINSIC_BASE_H_ */

#ifndef _WOLF_SERIALIZATION_CEREAL_PROCESSOR_PARAM_BASE_H_
#define _WOLF_SERIALIZATION_CEREAL_PROCESSOR_PARAM_BASE_H_

// Wolf includes
#include "core/processor/processor_base.h"

#include <cereal/cereal.hpp>
#include <cereal/types/polymorphic.hpp>

namespace cereal {

template <class Archive>
void serialize(Archive& ar, wolf::ProcessorParamsBase& o,
               std::uint32_t const /*version*/)
{
  ar( cereal::make_nvp("type", o.type) );
  ar( cereal::make_nvp("name", o.name) );
}

} // namespace cereal

#endif /* _WOLF_SERIALIZATION_CEREAL_SENSOR_INTRINSIC_BASE_H_ */

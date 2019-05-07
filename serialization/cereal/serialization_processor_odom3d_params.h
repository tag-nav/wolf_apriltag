#ifndef _WOLF_SERIALIZATION_CEREAL_PROCESSOR_ODOM3D_PARAMS_H_
#define _WOLF_SERIALIZATION_CEREAL_PROCESSOR_ODOM3D_PARAMS_H_

// Wolf includes
#include "core/processor/processor_odom_3D.h"
#include "serialization_processor_params_base.h"

namespace cereal {

template <class Archive>
void serialize(Archive& ar, wolf::ProcessorParamsOdom3D& o,
               std::uint32_t const /*version*/)
{
  ar( cereal::make_nvp("ProcessorParamsBase",
        cereal::base_class<wolf::ProcessorParamsBase>(&o)) );

  ar( cereal::make_nvp("angle_turned",    o.angle_turned)    );
  ar( cereal::make_nvp("dist_traveled",   o.dist_traveled)   );
  ar( cereal::make_nvp("max_buff_length", o.max_buff_length) );
  ar( cereal::make_nvp("max_time_span",   o.max_time_span)   );
}

} // namespace cereal

CEREAL_REGISTER_TYPE_WITH_NAME(wolf::ProcessorParamsOdom3D, "ProcessorOdom3DParams")

#endif /* _WOLF_SERIALIZATION_CEREAL_PROCESSOR_ODOM3D_PARAMS_H_ */

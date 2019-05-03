#ifndef _WOLF_SERIALIZATION_CEREAL_PROCESSOR_ODOM2D_PARAMS_H_
#define _WOLF_SERIALIZATION_CEREAL_PROCESSOR_ODOM2D_PARAMS_H_

// Wolf includes
#include "core/processor/processor_odom_2D.h"
#include "serialization_processor_params_base.h"

namespace cereal {

template <class Archive>
void serialize(Archive& ar, wolf::ProcessorParamsOdom2D& o,
               std::uint32_t const /*version*/)
{
  ar( cereal::make_nvp("ProcessorParamsBase",
        cereal::base_class<wolf::ProcessorParamsBase>(&o)) );

  ar( cereal::make_nvp("cov_det_th_",        o.cov_det)        );
  ar( cereal::make_nvp("dist_traveled_th_",  o.dist_traveled_th_)  );
  ar( cereal::make_nvp("elapsed_time_th_",   o.elapsed_time_th_)   );
  ar( cereal::make_nvp("theta_traveled_th_", o.theta_traveled_th_) );
  ar( cereal::make_nvp("unmeasured_perturbation_std_",
                       o.unmeasured_perturbation_std)   );
}

} // namespace cereal

CEREAL_REGISTER_TYPE_WITH_NAME(wolf::ProcessorParamsOdom2D, "ProcessorParamsOdom2D")

#endif /* _WOLF_SERIALIZATION_CEREAL_PROCESSOR_ODOM3D_PARAMS_H_ */

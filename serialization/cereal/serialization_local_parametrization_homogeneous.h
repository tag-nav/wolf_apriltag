#ifndef _WOLF_IO_CEREAL_LOCAL_PARAMETRIZATION_BASE_HOMOGENEOUS_H_
#define _WOLF_IO_CEREAL_LOCAL_PARAMETRIZATION_BASE_HOMOGENEOUS_H_

#include "core/local_parametrization_homogeneous.h"

#include "serialization_local_parametrization_base.h"

#include <cereal/cereal.hpp>

CEREAL_REGISTER_TYPE_WITH_NAME(wolf::LocalParametrizationHomogeneous,
                               "LocalParametrizationHomogeneous");

namespace cereal {

template<class Archive>
inline void serialize(
    Archive& ar,
    wolf::LocalParametrizationHomogeneous& lp,
    std::uint32_t const /*file_version*/)
{
  ar( cereal::make_nvp("LocalParametrizationBase",
                       cereal::base_class<wolf::LocalParametrizationBase>(&lp)) );
}

} //namespace cereal

#endif /* _WOLF_IO_CEREAL_LOCAL_PARAMETRIZATION_BASE_HOMOGENEOUS_H_ */

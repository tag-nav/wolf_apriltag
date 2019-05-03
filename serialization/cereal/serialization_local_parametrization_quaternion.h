#ifndef WOLF_IO_SERIALIZATION_LOCAL_PARAMETRIZATION_QUATERNION_H_
#define WOLF_IO_SERIALIZATION_LOCAL_PARAMETRIZATION_QUATERNION_H_

#include "core/local_parametrization_quaternion.h"

#include "serialization_local_parametrization_base.h"

#include <cereal/cereal.hpp>

CEREAL_REGISTER_TYPE_WITH_NAME(wolf::LocalParametrizationQuaternion<wolf::DQ_LOCAL>,
                               "wolf_LocalParametrizationQuaternion_DQ_LOCAL")

CEREAL_REGISTER_TYPE_WITH_NAME(wolf::LocalParametrizationQuaternion<wolf::DQ_GLOBAL>,
                               "wolf_LocalParametrizationQuaternion_DQ_GLOBAL")

namespace cereal {

template<class Archive, unsigned int DeltaReference>
inline void serialize(
    Archive &ar,
    wolf::LocalParametrizationQuaternion<DeltaReference> &lp,
    const unsigned int /*file_version*/)
{
  ar( cereal::make_nvp("LocalParametrizationBase",
                       cereal::base_class<wolf::LocalParametrizationBase>(&lp)) );
}

} //namespace boost

#endif /* WOLF_IO_SERIALIZATION_LOCAL_PARAMETRIZATION_QUATERNION_H_ */

#ifndef _WOLF_IO_CEREAL_LOCAL_PARAMETRIZATION_BASE_H_
#define _WOLF_IO_CEREAL_LOCAL_PARAMETRIZATION_BASE_H_

#include "base/state_block/local_parametrization_base.h"

#include <cereal/cereal.hpp>
#include <cereal/types/polymorphic.hpp>

namespace cereal {

// Since classes deriving from LocalParametrizationBase
// have default constructor calling the non-default
// LocalParametrizationBase constructor with pre-defined
// arguments, there is nothing to save here.
template<class Archive>
inline void serialize(
    Archive& /*ar*/,
    wolf::LocalParametrizationBase& /*lpb*/,
    std::uint32_t const /*file_version*/)
{
  //
}

} //namespace cereal

#endif /* _WOLF_IO_CEREAL_LOCAL_PARAMETRIZATION_BASE_H_ */

#ifndef _WOLF_IO_CEREAL_TIME_STAMP_H_
#define _WOLF_IO_CEREAL_TIME_STAMP_H_

// Wolf includes
#include "core/time_stamp.h"

#include <cereal/cereal.hpp>

namespace cereal {

/// @note serialization versionning raise
/// a compile error here...
template <class Archive>
void save(Archive& ar, const wolf::TimeStamp& o/*, std::uint32_t const version*/)
{
  ar( cereal::make_nvp("value", o.get()) );
}

template <class Archive>
void load(Archive& ar, wolf::TimeStamp& o/*, std::uint32_t const version*/)
{
  auto val = o.get();

  ar( cereal::make_nvp("value", val) );

  o.set(val);
}

} // namespace cereal

#endif /* _WOLF_IO_CEREAL_TIME_STAMP_H_ */

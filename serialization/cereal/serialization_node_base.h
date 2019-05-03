#ifndef _WOLF_IO_CEREAL_NODE_BASE_H_
#define _WOLF_IO_CEREAL_NODE_BASE_H_

// Wolf includes
#include "core/node_base.h"

#include <cereal/cereal.hpp>
#include <cereal/types/polymorphic.hpp>

namespace wolf {

struct NodeBase::Serializer {

  template <class Archive>
  static void serialize(Archive& ar, NodeBase& o, std::uint32_t const /*version*/)
  {
    ar( cereal::make_nvp("node_class_", o.node_category_) );
    ar( cereal::make_nvp("node_type_",  o.node_type_)  );
    ar( cereal::make_nvp("node_name_",  o.node_name_)  );
    ar( cereal::make_nvp("node_id_",    o.node_id_)    );

//    ar( cereal::make_nvp("problem_ptr_", o.problem_ptr_) );

    // Not sure what to do with this guy ...
    //ar( cereal::make_nvp("node_id_count_",    o.node_id_count_)    );
  }

  template <class Archive>
  static void load_and_construct( Archive& ar, cereal::construct<wolf::NodeBase>& construct,
                                  std::uint32_t const /*version*/ )
  {
    decltype(std::declval<wolf::NodeBase>().getCategory()) nb_class;
    decltype(std::declval<wolf::NodeBase>().getType())  nb_type;
    decltype(std::declval<wolf::NodeBase>().getName())  nb_name;

    ar( cereal::make_nvp("node_class_", nb_class) );
    ar( cereal::make_nvp("node_type_",  nb_type) );
    ar( cereal::make_nvp("node_name_",  nb_name) );

    construct( nb_class, nb_type, nb_name );

    ar( cereal::make_nvp("node_id_", construct->node_id_) );

//    ar( cereal::make_nvp("problem_ptr_", construct->problem_ptr_) );

    // Not sure what to do with this guy ...
    //ar( cereal::make_nvp("node_id_count_", construct->node_id_count_)    );
  }
};

} // namespace wolf

namespace cereal {

/// @note No default constructor thus the need
/// for these specializations
template <>
struct LoadAndConstruct<wolf::NodeBase>
{
  template <class Archive>
  static void load_and_construct( Archive& ar,
                                  cereal::construct<wolf::NodeBase>& construct,
                                  std::uint32_t const version )
  {
    wolf::NodeBase::Serializer::load_and_construct(ar, construct, version);
  }
};

template <class Archive>
void serialize(Archive& ar, wolf::NodeBase& o, std::uint32_t const version)
{
  wolf::NodeBase::Serializer::serialize(ar, o, version);
}

} // namespace cereal

#endif /* _WOLF_IO_CEREAL_NODE_BASE_H_ */

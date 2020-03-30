#include "apriltag/landmark/landmark_apriltag.h"
#include "core/common/factory.h"
#include "core/math/rotations.h"
#include "core/yaml/yaml_conversion.h"

namespace wolf {

LandmarkApriltag::LandmarkApriltag(Eigen::Vector7d& pose, const int& _tagid, const double& _tag_width) :
	LandmarkBase("LandmarkApriltag", std::make_shared<StateBlock>(pose.head(3)), std::make_shared<StateQuaternion>(pose.tail(4))),
	tag_id_(_tagid),
	tag_width_(_tag_width)
{
  	setDescriptor(Eigen::VectorXd::Constant(1,_tagid)); //change tagid to int ? do not use descriptor vector ?
    setId(_tagid); // overwrite lmk ID to match tag_id.
}

LandmarkApriltag::~LandmarkApriltag()
{
    //
}


double LandmarkApriltag::getTagWidth() const
{
    return tag_width_;
}

int LandmarkApriltag::getTagId() const
{
    return tag_id_;
}

// LANDMARK SAVE AND LOAD FROM YAML

// static
LandmarkBasePtr LandmarkApriltag::create(const YAML::Node& _lmk_node)
{
    // Parse YAML node with lmk info and data
    unsigned int    id                      = _lmk_node["id"]                   .as<unsigned int>();
    unsigned int    tag_id                  = _lmk_node["tag id"]               .as<unsigned int>();
    double          tag_width               = _lmk_node["tag width"]            .as<double>();
    Eigen::Vector3d pos                     = _lmk_node["position"]             .as<Eigen::Vector3d>();
    bool            pos_fixed               = _lmk_node["position fixed"]       .as<bool>();
    Eigen::Vector4d vquat;
    if (_lmk_node["orientation"].size() == 3)
    {
        // we have been given 3 Euler angles in degrees
        Eigen::Vector3d   euler = M_TORAD * ( _lmk_node["orientation"]          .as<Eigen::Vector3d>() );
        Eigen::Matrix3d       R = e2R(euler);
        Eigen::Quaterniond quat = R2q(R);
        vquat                   = quat.coeffs();
    }
    else if (_lmk_node["orientation"].size() == 4)
    {
        // we have been given a quaternion
        vquat                               = _lmk_node["orientation"]          .as<Eigen::Vector4d>();
    }
    bool            ori_fixed               = _lmk_node["orientation fixed"]    .as<bool>();

    Eigen::Vector7d pose; pose << pos, vquat;

    // Create a new landmark
    LandmarkApriltagPtr lmk_ptr = std::make_shared<LandmarkApriltag>(pose, tag_id, tag_width);
    lmk_ptr->setId(id);
    lmk_ptr->getP()->setFixed(pos_fixed);
    lmk_ptr->getO()->setFixed(ori_fixed);

    return lmk_ptr;

}

YAML::Node LandmarkApriltag::saveToYaml() const
{
    // First base things
    YAML::Node node = LandmarkBase::saveToYaml();

    // Then Apriltag specific things
    node["tag id"] = getTagId();
    node["tag width"] = getTagWidth();

    return node;
}


// Register landmark creator
namespace
{
const bool WOLF_UNUSED registered_lmk_apriltag = LandmarkFactory::get().registerCreator("LandmarkApriltag", LandmarkApriltag::create);
}


} // namespace wolf

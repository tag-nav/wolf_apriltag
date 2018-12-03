
#include "feature_apriltag.h"

namespace wolf {

FeatureApriltag::FeatureApriltag(const Eigen::Vector7s & _measurement, const Eigen::Matrix6s & _meas_covariance, const apriltag_detection_t * _det) :
    FeatureBase("APRILTAG", _measurement, _meas_covariance), *det(_det)
{
    //_measurement : translation + quaternion
    // meas_cov : a definir
    //std::cout << "feature: "<< _measurement.transpose() << std::endl;
}

FeatureApriltag::~FeatureApriltag()
{
    //
}

Scalar FeatureApriltag::getTagId() const
{
    return det->id;
}

} // namespace wolf

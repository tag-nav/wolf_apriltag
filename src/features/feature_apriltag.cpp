
#include "feature_apriltag.h"

namespace wolf {

FeatureApriltag::FeatureApriltag(const Eigen::Vector7s & _measurement,
                                 const Eigen::Matrix6s & _meas_uncertainty,
                                 const int _tag_id,
                                 const apriltag_detection_t & _det,
                                 UncertaintyType _uncertainty_type) :
    FeatureBase("APRILTAG", _measurement, _meas_uncertainty, _uncertainty_type),
    tag_id_     (_tag_id),
    tag_corners_(4),
    detection_  (_det)
{
    assert(_det.id == _tag_id && "Tag ID and detection ID do not match!");
    setTrackId(_tag_id);
    for (int i = 0; i < 4; i++)
    {
        tag_corners_[i].x = detection_.p[i][0];
        tag_corners_[i].y = detection_.p[i][1];
    }
}

FeatureApriltag::~FeatureApriltag()
{
    //
}

Scalar FeatureApriltag::getTagId() const
{
    return tag_id_;
}

const apriltag_detection_t& FeatureApriltag::getDetection() const
{
    return detection_;
}

const std::vector<cv::Point2d>& FeatureApriltag::getTagCorners() const
{
    return tag_corners_;
}

} // namespace wolf

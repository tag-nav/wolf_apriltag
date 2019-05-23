#ifndef FEATURE_APRILTAG_H_
#define FEATURE_APRILTAG_H_

//Wolf includes
#include "core/feature/feature_base.h"

//std includes

//external library incudes
#include "apriltag/apriltag.h"
#include "apriltag/common/zarray.h"

// opencv
#include <opencv2/features2d.hpp>

namespace wolf {
    
WOLF_PTR_TYPEDEFS(FeatureApriltag);

//class FeatureApriltag
class FeatureApriltag : public FeatureBase
{
    public:

        FeatureApriltag(const Eigen::Vector7s & _measurement, const Eigen::Matrix6s & _meas_covariance,
                        const int _tag_id, const apriltag_detection_t & _det,
                        Scalar _rep_error1, Scalar _rep_error2, bool _use_rotation,
                        UncertaintyType _uncertainty_type = UNCERTAINTY_IS_INFO);
        virtual ~FeatureApriltag();
        
        /** \brief Returns tag id
         * 
         * Returns tag id
         * 
         **/
        Scalar getTagId() const; 

        const apriltag_detection_t& getDetection() const;

        const std::vector<cv::Point2d>& getTagCorners() const;

        Scalar getRepError1() const;
        Scalar getRepError2() const;
        bool getUserotation() const;


    private:
        int tag_id_;
        std::vector<cv::Point2d> tag_corners_;
        apriltag_detection_t detection_;
        Scalar rep_error1_;
        Scalar rep_error2_;
        bool use_rotation_;
        
};

} // namespace wolf

#endif

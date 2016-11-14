#ifndef CAPTURE_GPS_FIX_H_
#define CAPTURE_GPS_FIX_H_

//Wolf includes
#include "capture_base.h"
#include "feature_gps_fix.h"

//std includes
//

namespace wolf {
    
//forward declaration to typedef class pointers
class CaptureGPSFix;
typedef std::shared_ptr<CaptureGPSFix> CaptureGPSFixPtr;
typedef std::shared_ptr<const CaptureGPSFix> CaptureGPSFixConstPtr;
typedef std::weak_ptr<CaptureGPSFix> CaptureGPSFixWPtr;    

//class CaptureGPSFix
class CaptureGPSFix : public CaptureBase
{
    protected:
        Eigen::VectorXs data_; ///< Raw data.
        Eigen::MatrixXs data_covariance_; ///< Noise of the capture.

    public:
        CaptureGPSFix(const TimeStamp& _ts, SensorBasePtr _sensor_ptr, const Eigen::VectorXs& _data);
        CaptureGPSFix(const TimeStamp& _ts, SensorBasePtr _sensor_ptr, const Eigen::VectorXs& _data, const Eigen::MatrixXs& _data_covariance);
        virtual ~CaptureGPSFix();
        virtual void process();
};

} //namespace wolf
#endif

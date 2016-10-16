#include "capture_gps_fix.h"


namespace wolf {

CaptureGPSFix::CaptureGPSFix(const TimeStamp& _ts, SensorBasePtr _sensor_ptr, const Eigen::VectorXs& _data) :
	CaptureBase("GPS FIX", _ts, _sensor_ptr),
	data_(_data)
{
    //
}

CaptureGPSFix::CaptureGPSFix(const TimeStamp& _ts, SensorBasePtr _sensor_ptr, const Eigen::VectorXs& _data, const Eigen::MatrixXs& _data_covariance) :
	CaptureBase("GPS FIX", _ts, _sensor_ptr),
	data_(_data),
	data_covariance_(_data_covariance)
{
    //
}

CaptureGPSFix::~CaptureGPSFix()
{
	//std::cout << "Destroying GPS fix capture...\n";
}

void CaptureGPSFix::process()
{
	// EXTRACT AND ADD FEATURES
    addFeature(std::make_shared<FeatureGPSFix>(data_,data_covariance_));

    // ADD CONSTRAINT
//    getFeatureListPtr()->front()->addConstraint(new ConstraintGPS2D(getFeatureListPtr()->front(), getFramePtr()));
    getFeatureListPtr()->front()->addConstraint(std::make_shared <ConstraintGPS2D>(getFeatureListPtr()->front()));
}


} //namespace wolf

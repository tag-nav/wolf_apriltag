//
// Created by ptirindelli on 16/12/15.
//

#include "base/factor/factor_GPS_pseudorange_2D.h"
#include "base/feature/feature_GPS_pseudorange.h"
#include "base/processor/processor_GPS.h"

#include "base/capture/capture_GPS.h"

namespace wolf
{

ProcessorGPS::ProcessorGPS(ProcessorParamsBasePtr _params) :
        ProcessorBase("GPS", _params),
        capture_gps_ptr_(nullptr)
{
    gps_covariance_ = 10;
}

ProcessorGPS::~ProcessorGPS()
{
}

void ProcessorGPS::init(CaptureBasePtr _capture_ptr)
{
}

void ProcessorGPS::process(CaptureBasePtr _capture_ptr)
{
    std::cout << "ProcessorGPS::process(GPScapture)" << std::endl;
    capture_gps_ptr_ = std::static_pointer_cast<CaptureGPS>(_capture_ptr);

    //std::cout << "Extracting gps features..." << std::endl;
    rawgpsutils::SatellitesObs obs = capture_gps_ptr_->getData();
    for (unsigned int i = 0; i < obs.measurements_.size(); ++i)
    {
        Eigen::Vector3s sat_pos = obs.measurements_[i].sat_position_;
        Scalar pr = obs.measurements_[i].pseudorange_;
        // capture_gps_ptr_->addFeature(std::make_shared<FeatureGPSPseudorange>(sat_pos, pr, gps_covariance_));
        FeatureBase::emplace<FeatureGPSPseudorange>(capture_gps_ptr_, sat_pos, pr, gps_covariance_);
    }
    //std::cout << "gps features extracted" << std::endl;
    //std::cout << "Establishing factors to gps features..." << std::endl;
    for (auto i_it = capture_gps_ptr_->getFeatureList().begin();
            i_it != capture_gps_ptr_->getFeatureList().end(); i_it++)
    {
        // capture_gps_ptr_->getFeatureList().front()->addFactor(std::make_shared<FactorGPSPseudorange2D>((*i_it), shared_from_this()));
        FactorBase::emplace<FactorGPSPseudorange2D>(capture_gps_ptr_->getFeatureList().front(), (*i_it), shared_from_this());
    }
    //std::cout << "Factors established" << std::endl;
}

bool ProcessorGPS::voteForKeyFrame()
{
    return false;
}

void ProcessorGPS::keyFrameCallback(FrameBasePtr, const Scalar& _time_tol)
{
    //
}

ProcessorBasePtr ProcessorGPS::create(const std::string& _unique_name, const ProcessorParamsBasePtr _params, const SensorBasePtr sensor_ptr)
{
    ProcessorGPSPtr prc_ptr = std::make_shared<ProcessorGPS>(_params);
    prc_ptr->setName(_unique_name);
    return prc_ptr;
}

} // namespace wolf

// Register in the SensorFactory
#include "base/processor/processor_factory.h"
namespace wolf {
WOLF_REGISTER_PROCESSOR("GPS",ProcessorGPS)
} // namespace wolf

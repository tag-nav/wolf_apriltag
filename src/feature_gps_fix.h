#ifndef FEATURE_GPS_FIX_H_
#define FEATURE_GPS_FIX_H_

//Wolf includes
#include "feature_base.h"
#include "constraint_gps_2D.h"

//std includes

namespace wolf {
    
WOLF_PTR_TYPEDEFS(FeatureGPSFix);

//class FeatureGPSFix
class FeatureGPSFix : public FeatureBase
{
    public:
        /** \brief Constructor from capture pointer and measure dim
         * 
         * \param _dim_measurement the dimension of the measurement space
         * 
         */
        FeatureGPSFix(unsigned int _dim_measurement);

        /** \brief Constructor from capture pointer and measure
         *
         * \param _measurement the measurement
         * \param _meas_covariance the noise of the measurement
         *
         */
        FeatureGPSFix(const Eigen::VectorXs& _measurement, const Eigen::MatrixXs& _meas_covariance);
        virtual ~FeatureGPSFix();
};

} // namespace wolf

#endif

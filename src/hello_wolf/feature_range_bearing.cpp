/*
 * FeatureRangeBearing2D.cpp
 *
 *  Created on: Nov 30, 2017
 *      Author: jsola
 */

#include "feature_range_bearing.h"

namespace wolf
{

FeatureRangeBearing::FeatureRangeBearing(const Eigen::VectorXs& _measurement, const Eigen::MatrixXs& _meas_covariance) :
        FeatureBase("RANGE BEARING 2D", _measurement, _meas_covariance)
{
    //
}


FeatureRangeBearing::~FeatureRangeBearing()
{
    // TODO Auto-generated destructor stub
}

} // namespace wolf

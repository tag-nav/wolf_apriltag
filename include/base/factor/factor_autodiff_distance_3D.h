/**
 * \file factor_autodiff_distance_3D.h
 *
 *  Created on: Apr 10, 2018
 *      \author: jsola
 */

#ifndef FACTOR_AUTODIFF_DISTANCE_3D_H_
#define FACTOR_AUTODIFF_DISTANCE_3D_H_

#include "base/factor/factor_autodiff.h"

namespace wolf
{

WOLF_PTR_TYPEDEFS(FactorAutodiffDistance3D);

class FactorAutodiffDistance3D : public FactorAutodiff<FactorAutodiffDistance3D, 1, 3, 3>
{
    public:
        FactorAutodiffDistance3D(const FeatureBasePtr&   _feat,
                                 const FrameBasePtr&     _frm_other,
                                 const ProcessorBasePtr& _processor_ptr,
                                 bool                    _apply_loss_function,
                                 FactorStatus            _status) :
            FactorAutodiff("DISTANCE 3D",
                            _frm_other,
                            nullptr,
                            nullptr,
                            nullptr,
                            _processor_ptr,
                            _apply_loss_function,
                            _status,
                            _feat->getFrame()->getP(),
                            _frm_other->getP())
        {
            setFeature(_feat);
        }

        virtual ~FactorAutodiffDistance3D() { /* nothing */ }

        template<typename T>
        bool operator () (const T* const _pos1,
                          const T* const _pos2,
                          T* _residual) const
        {
            using namespace Eigen;

            Map<const Matrix<T,3,1>> pos1(_pos1);
            Map<const Matrix<T,3,1>> pos2(_pos2);
            Map<Matrix<T,1,1>> res(_residual);

            // If pos2 and pos1 are the same, undefined behavior when computing the jacobian
            T norm_squared = ( pos2 - pos1 ).squaredNorm();
            if (norm_squared < (T)1e-8){
                norm_squared += (T)1e-8;
            }
            Matrix<T,1,1> dist_exp ( sqrt(norm_squared) );
            Matrix<T,1,1> dist_meas (getMeasurement().cast<T>() );
            Matrix<T,1,1> sqrt_info_upper = getMeasurementSquareRootInformationUpper().cast<T>();

            res  = sqrt_info_upper * (dist_meas - dist_exp);

            return true;
        }
};

} /* namespace wolf */

#endif /* FACTOR_AUTODIFF_DISTANCE_3D_H_ */

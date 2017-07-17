/**
 * \file processor_odom_2D.h
 *
 *  Created on: Apr 15, 2016
 *      \author: jvallve
 */

#ifndef SRC_PROCESSOR_ODOM_2D_H_
#define SRC_PROCESSOR_ODOM_2D_H_

#include "processor_motion.h"
#include "constraint_odom_2D.h"
#include "rotations.h"


namespace wolf {
    
WOLF_PTR_TYPEDEFS(ProcessorOdom2D);
WOLF_STRUCT_PTR_TYPEDEFS(ProcessorParamsOdom2D);

struct ProcessorParamsOdom2D : public ProcessorParamsBase
{
    Scalar dist_traveled_th_;
    Scalar theta_traveled_th_;
    Scalar cov_det_th_;
    Scalar elapsed_time_th_;
    Scalar unmeasured_perturbation_std_;
};

class ProcessorOdom2D : public ProcessorMotion
{
    public:
        ProcessorOdom2D(const Scalar& _traveled_dist_th             = 1.0,
                        const Scalar& _theta_traveled_th            = 0.17,
                        const Scalar& _cov_det_th                   = 1.0,
                        const Scalar& _elapsed_time_th              = 1.0,
                        const Scalar& _unmeasured_perturbation_std  = 0.001);
        virtual ~ProcessorOdom2D();
        virtual bool voteForKeyFrame();

    protected:
        virtual void data2delta(const Eigen::VectorXs& _data,
                                const Eigen::MatrixXs& _data_cov,
                                const Scalar _dt);
        virtual void deltaPlusDelta(const Eigen::VectorXs& _delta1,
                                    const Eigen::VectorXs& _delta2,
                                    const Scalar _Dt2,
                                    Eigen::VectorXs& _delta1_plus_delta2);
        virtual void deltaPlusDelta(const Eigen::VectorXs& _delta1,
                                    const Eigen::VectorXs& _delta2,
                                    const Scalar _Dt2,
                                    Eigen::VectorXs& _delta1_plus_delta2,
                                    Eigen::MatrixXs& _jacobian1,
                                    Eigen::MatrixXs& _jacobian2);
        virtual void statePlusDelta(const Eigen::VectorXs& _x,
                                const Eigen::VectorXs& _delta,
                                const Scalar _Dt,
                                Eigen::VectorXs& _x_plus_delta);
        virtual Eigen::VectorXs deltaZero() const;
        virtual Motion interpolate(const Motion& _ref,
                                   Motion& _second,
                                   TimeStamp& _ts);

        virtual ConstraintBasePtr emplaceConstraint(FeatureBasePtr _feature_motion, FrameBasePtr _frame_origin);

        virtual void resetDerived();

    protected:
        Scalar dist_traveled_th_;
        Scalar theta_traveled_th_;
        Scalar cov_det_th_;
        Scalar elapsed_time_th_;
        Matrix3s unmeasured_perturbation_cov_; ///< Covariance to be added to the unmeasured perturbation

        // Factory method
    public:
        static ProcessorBasePtr create(const std::string& _unique_name, const ProcessorParamsBasePtr _params, const SensorBasePtr sensor_ptr = nullptr);
};

inline ProcessorOdom2D::ProcessorOdom2D(const Scalar& _dist_traveled_th,
                                        const Scalar& _theta_traveled_th,
                                        const Scalar& _cov_det_th,
                                        const Scalar& _elapsed_time_th,
                                        const Scalar& _unmeasured_perturbation_std) :
        ProcessorMotion("ODOM 2D", 3, 3, 3, 2),
        dist_traveled_th_(_dist_traveled_th),
        theta_traveled_th_(_theta_traveled_th),
        cov_det_th_(_cov_det_th),
        elapsed_time_th_(_elapsed_time_th)
{
    unmeasured_perturbation_cov_ = _unmeasured_perturbation_std*_unmeasured_perturbation_std * Matrix3s::Identity();
}
inline ProcessorOdom2D::~ProcessorOdom2D()
{
}

inline void ProcessorOdom2D::data2delta(const Eigen::VectorXs& _data, const Eigen::MatrixXs& _data_cov, const Scalar _dt)
{
    //std::cout << "ProcessorOdom2d::data2delta" << std::endl;

    assert(_data.size() == data_size_ && "Wrong _data vector size");
    assert(_data_cov.rows() == data_size_ && "Wrong _data_cov size");
    assert(_data_cov.cols() == data_size_ && "Wrong _data_cov size");

    // data  is [dtheta, dr]
    // delta is [dx, dy, dtheta]
    // motion model is 1/2 turn + straight + 1/2 turn
    delta_(0) = cos(_data(1)/2) * _data(0);
    delta_(1) = sin(_data(1)/2) * _data(0);
    delta_(2) = _data(1);

    // Fill delta covariance
    Eigen::MatrixXs J(delta_cov_size_,data_size_);
    J(0,0) =   cos(_data(1) / 2);
    J(1,0) =   sin(_data(1) / 2);
    J(2,0) =   0;
    J(0,1) = - _data(0) * sin(_data(1) / 2) / 2;
    J(1,1) =   _data(0) * cos(_data(1) / 2) / 2;
    J(2,1) =   1;

//    if (getBuffer().get().size() > 2)
//      delta_cov_ = J * _data_cov * J.transpose();
//    else
//      delta_cov_ = J * _data_cov * J.transpose() + std::abs(unmeasured_perturbation_var_*_data(0))*Eigen::MatrixXs::Identity(delta_cov_size_,delta_cov_size_);

    // Since input data is size 2, and delta is size 3, the noise model must be given by:
    // 1. Covariance of the input data:  J*Q*J.tr
    // 2. Fix variance term to be added: var*Id
    delta_cov_ = J * _data_cov * J.transpose() + unmeasured_perturbation_cov_;

    //std::cout << "data      :" << _data.transpose() << std::endl;
    //std::cout << "data cov  :" << std::endl << _data_cov << std::endl;
    //std::cout << "delta     :" << delta_.transpose() << std::endl;
    //std::cout << "delta cov :" << std::endl << delta_cov_ << std::endl;
}

inline void ProcessorOdom2D::statePlusDelta(const Eigen::VectorXs& _x, const Eigen::VectorXs& _delta, const Scalar _Dt, Eigen::VectorXs& _x_plus_delta)
{

    // This is just a frame composition in 2D

    //std::cout << "ProcessorOdom2d::statePlusDelta" << std::endl;

    assert(_x.size() == x_size_ && "Wrong _x vector size");
    assert(_x_plus_delta.size() == x_size_ && "Wrong _x_plus_delta vector size");

//    std::cout << "statePlusDelta ------------------------------------" << std::endl;
//    std::cout << "_x:     " << _x.transpose() << std::endl;
//    std::cout << "_delta: " << _delta.transpose() << std::endl;
//    std::cout << "_x_plus_delta: " << _x_plus_delta.transpose() << std::endl;

    _x_plus_delta.head<2>() = _x.head<2>() + Eigen::Rotation2Ds(_x(2)).matrix() * _delta.head<2>();
    _x_plus_delta(2) = pi2pi(_x(2) + _delta(2));

//    std::cout << "-----------------------------------------------" << std::endl;
//    std::cout << "_x_plus_delta: " << _x_plus_delta.transpose() << std::endl;
}

inline void ProcessorOdom2D::deltaPlusDelta(const Eigen::VectorXs& _delta1, const Eigen::VectorXs& _delta2, const Scalar _Dt2, Eigen::VectorXs& _delta1_plus_delta2)
{
    // This is just a frame composition in 2D

    //std::cout << "ProcessorOdom2d::deltaPlusDelta" << std::endl;
    assert(_delta1.size() == delta_size_ && "Wrong _delta1 vector size");
    assert(_delta2.size() == delta_size_ && "Wrong _delta2 vector size");
    assert(_delta1_plus_delta2.size() == delta_size_ && "Wrong _delta1_plus_delta2 vector size");

//    std::cout << "deltaPlusDelta ------------------------------------" << std::endl;
//    std::cout << "_delta1: " << _delta1.transpose() << std::endl;
//    std::cout << "_delta2: " << _delta2.transpose() << std::endl;
//    std::cout << "_delta1_plus_delta2: " << _delta1_plus_delta2.transpose() << std::endl;

    _delta1_plus_delta2.head<2>() = _delta1.head<2>() + Eigen::Rotation2Ds(_delta1(2)).matrix() * _delta2.head<2>();
    _delta1_plus_delta2(2) = pi2pi(_delta1(2) + _delta2(2));

//    std::cout << "-----------------------------------------------" << std::endl;
//    std::cout << "_delta1_plus_delta2: " << _delta1_plus_delta2.transpose() << std::endl;
}

inline void ProcessorOdom2D::deltaPlusDelta(const Eigen::VectorXs& _delta1, const Eigen::VectorXs& _delta2,
                                            const Scalar _Dt2,
                                            Eigen::VectorXs& _delta1_plus_delta2, Eigen::MatrixXs& _jacobian1,
                                            Eigen::MatrixXs& _jacobian2)
{
    //std::cout << "ProcessorOdom2d::deltaPlusDelta jacobians" << std::endl;
    assert(_delta1.size() == delta_size_ && "Wrong _delta1 vector size");
    assert(_delta2.size() == delta_size_ && "Wrong _delta2 vector size");
    assert(_delta1_plus_delta2.size() == delta_size_ && "Wrong _delta1_plus_delta2 vector size");
    assert(_jacobian1.rows() == delta_cov_size_ && "Wrong _jacobian1 size");
    assert(_jacobian1.cols() == delta_cov_size_ && "Wrong _jacobian1 size");
    assert(_jacobian2.rows() == delta_cov_size_ && "Wrong _jacobian2 size");
    assert(_jacobian2.cols() == delta_cov_size_ && "Wrong _jacobian2 size");

//    std::cout << "deltaPlusDelta ------------------------------------" << std::endl;
//    std::cout << "_delta1: " << _delta1.transpose() << std::endl;
//    std::cout << "_delta2: " << _delta2.transpose() << std::endl;
//    std::cout << "_delta1_plus_delta2: " << _delta1_plus_delta2.transpose() << std::endl;

    _delta1_plus_delta2.head<2>() = _delta1.head<2>() + Eigen::Rotation2Ds(_delta1(2)).matrix() * _delta2.head<2>();
    _delta1_plus_delta2(2) = pi2pi(_delta1(2) + _delta2(2));

    // Jac wrt delta_integrated
    _jacobian1 = Eigen::MatrixXs::Identity(delta_cov_size_,delta_cov_size_);
    _jacobian1(0,2) = -sin(_delta1(2))*_delta2(0) - cos(_delta1(2))*_delta2(1);
    _jacobian1(1,2) =  cos(_delta1(2))*_delta2(0) - sin(_delta1(2))*_delta2(1);

    // jac wrt delta
    _jacobian2 = Eigen::MatrixXs::Identity(delta_cov_size_,delta_cov_size_);
    _jacobian2.topLeftCorner<2,2>() = Eigen::Rotation2Ds(_delta1(2)).matrix();

    //std::cout << "-----------------------------------------------" << std::endl;
    //std::cout << "_delta1_plus_delta2: " << _delta1_plus_delta2.transpose() << std::endl;
}

inline Eigen::VectorXs ProcessorOdom2D::deltaZero() const
{
    return Eigen::VectorXs::Zero(delta_size_);
}

inline ConstraintBasePtr ProcessorOdom2D::emplaceConstraint(FeatureBasePtr _feature_motion, FrameBasePtr _frame_origin)
{
    ConstraintOdom2DPtr ctr_odom = std::make_shared<ConstraintOdom2D>(_feature_motion, _frame_origin);
    _feature_motion->addConstraint(ctr_odom);
    _frame_origin->addConstrainedBy(ctr_odom);
    return ctr_odom;
}

inline Motion ProcessorOdom2D::interpolate(const Motion& _ref, Motion& _second, TimeStamp& _ts)
{
    // TODO: Implement actual interpolation
    // Implementation: motion ref keeps the same
    //
    Motion _interpolated(_ref);
    _interpolated.ts_                   = _ts;
    _interpolated.delta_                = deltaZero();
    _interpolated.delta_cov_            = Eigen::MatrixXs::Zero(delta_size_, delta_size_);
    _interpolated.delta_integr_         = _ref.delta_integr_;
    _interpolated.jacobian_delta_integr_. setIdentity();
    _interpolated.jacobian_delta_       . setZero();
    return _interpolated;
}

inline bool ProcessorOdom2D::voteForKeyFrame()
{
    // Distance criterion
    //std::cout << "ProcessorOdom2D::voteForKeyFrame: traveled distance " << getBufferPtr()->get().back().delta_integr_.norm() << std::endl;
    if (getBuffer().get().back().delta_integr_.head<2>().norm() > dist_traveled_th_)
    {
//        std::cout << "ProcessorOdom2D:: " << id() << " -  VOTE FOR KEY FRAME traveled distance "
//                << getBuffer().get().back().delta_integr_.head<2>().norm() << std::endl;
        return true;
    }

    if (/*std::abs*/(getBuffer().get().back().delta_integr_.tail<1>().norm()) > theta_traveled_th_)
    {
//        std::cout << "ProcessorOdom2D:: " << id() << " -  VOTE FOR KEY FRAME traveled distance "
//                << getBuffer().get().back().delta_integr_.head<2>().norm() << std::endl;
        return true;
    }

    // Uncertainty criterion
    delta_integrated_cov_ = getBuffer().get().back().jacobian_delta_integr_ * delta_integrated_cov_ * getBuffer().get().back().jacobian_delta_integr_.transpose() + getBuffer().get().back().jacobian_delta_ * getBuffer().get().back().delta_cov_ * getBuffer().get().back().jacobian_delta_.transpose();
    if (delta_integrated_cov_.determinant() > cov_det_th_)
    {
//        std::cout << "ProcessorOdom2D:: " << id() << " - VOTE FOR KEY FRAME covariance det "
//                << delta_integrated_cov_.determinant() << std::endl;
        return true;
    }

    // Time criterion
    if (getBuffer().get().back().ts_.get() - origin_ptr_->getFramePtr()->getTimeStamp().get() > elapsed_time_th_)
    {
//        std::cout << "ProcessorOdom2D:: " << id() << " - VOTE FOR KEY FRAME elapsed time "
//                << getBuffer().get().back().ts_.get() - origin_ptr_->getFramePtr()->getTimeStamp().get()
//                << std::endl;
        return true;
    }
    return false;
}

inline void ProcessorOdom2D::resetDerived()
{
    // We want this covariance up-to-date because we use it in vote_for_keyframe().
    delta_integrated_cov_ = integrateBufferCovariance(getBuffer());
}

} // namespace wolf

#endif /* SRC_PROCESSOR_ODOM_2D_H_ */

#ifndef PROCESSOR_IMU_H
#define PROCESSOR_IMU_H

// Wolf
#include "processor_motion.h"
#include "frame_imu.h"


namespace wolf {

class ProcessorIMU : public ProcessorMotion{
    public:
        ProcessorIMU();
        virtual ~ProcessorIMU();

    public:
        const Eigen::Vector3s& getGravity() const;

    protected:

        // Helper functions

        /**
         * @brief Extract data from the IMU and create a delta-state for one IMU step
         * @param _data
         * @param _data_cov
         * @param _dt
         */
        virtual void data2delta(const Eigen::VectorXs& _data, const Eigen::MatrixXs& _data_cov, const Scalar _dt);

        /** \brief composes a delta-state on top of another delta-state
         * \param _delta_preint the first delta-state
         * \param _delta the second delta-state
         * \param _dt the second delta-state's time delta
         * \param _delta_preint_plus_delta the delta2 composed on top of delta1. It has the format of delta-state.
         *
         *
         * _jacobian1 is A (3x9) _jacobian2 should be B (3x6) but not here..
         * This function implements the composition (+) so that _delta1_plus_delta2 = _delta1 (+) _delta2
         *
         * See its definition for more comments about the inner maths.
         */
        virtual void deltaPlusDelta(const Eigen::VectorXs& _delta_preint, const Eigen::VectorXs& _delta,
                                    const Scalar _dt, Eigen::VectorXs& _delta_preint_plus_delta);

        virtual void deltaPlusDelta(const Eigen::VectorXs& _delta_preint, const Eigen::VectorXs& _delta,
                                    const Scalar _dt, Eigen::VectorXs& _delta_preint_plus_delta,
                                    Eigen::MatrixXs& _jacobian1, Eigen::MatrixXs& _jacobian2);

        /** \brief composes a delta-state on top of a state
         * \param _x the initial state
         * \param _delta the delta-state
         * \param _x_plus_delta the updated state. It has the same format as the initial state.
         * \param _Dt the time interval between the origin state and the Delta
         *
         * This function implements the composition (+) so that _x2 = _x1 (+) _delta.
         */
        virtual void xPlusDelta(const Eigen::VectorXs& _x, const Eigen::VectorXs& _delta, const Scalar _Dt,
                                Eigen::VectorXs& _x_plus_delta );


        /** \brief Adds a delta-increment into the pre-integrated Delta-state
        * \param _delta the delta increment to add
        *
        * This function implements the pre-integrated measurements update :
        *   Delta_ik = Delta_ij (+) _delta_jk
        */
        virtual void integrateDelta();

        /** \brief Delta representing the null motion
         */
        virtual Eigen::VectorXs deltaZero() const;

        virtual Motion interpolate(const Motion& _motion_ref, Motion& _motion, TimeStamp& _ts);

        void resetDerived();

        virtual ConstraintBase* createConstraint(FeatureBase* _feature_motion, FrameBase* _frame_origin);

    private:

        /** \brief Compute Jr (Right Jacobian)
         * Right Jacobian for exp map in SO(3) - equation (10.86) and following equations in
         *  G.S. Chirikjian, "Stochastic Models, Information Theory, and Lie Groups", Volume 2, 2008.
         *  expmap( omega + d_omega ) \approx expmap(omega) * expmap(Jr * d_omega)
         *  where Jr = expMapDerivative(omega);
         *  This maps a perturbation in the tangent space (d_omega) to a perturbation on the manifold (expmap(Jr * d_omega))
         *  so that:
         *
         *      exp(omega+domega) = exp(omega)*exp(Jr(omega)*domega)
         */
        Eigen::Matrix3s expMapDerivative(const Eigen::Vector3s& _omega);

        /** \brief Compute Jrinv (inverse of Right Jacobian which corresponds to the jacobian of log)
         *  Right Jacobian for Log map in SO(3) - equation (10.86) and following equations in
         *  G.S. Chirikjian, "Stochastic Models, Information Theory, and Lie Groups", Volume 2, 2008.
         *  logmap( Rhat * expmap(omega) ) \approx logmap( Rhat ) + Jrinv * omega
         *  where Jrinv = logMapDerivative(omega);
         *  This maps a perturbation on the manifold (expmap(omega)) to a perturbation in the tangent space (Jrinv * omega) so that
         *
         *      log(exp(omega)*exp(domega)) = omega + Jrinv(omega)*domega
         */
        Eigen::Matrix3s logMapDerivative(const Eigen::Vector3s& _omega);

    private:

        // Casted pointer to IMU frame
        FrameIMU* frame_imu_ptr_;

        // gravity vector
        const Eigen::Vector3s gravity_;

        // Maps to the biases in the keyframe's state
        Eigen::Map<Eigen::Vector3s> bias_acc_;
        Eigen::Map<Eigen::Vector3s> bias_gyro_;

        // Maps to the received measurements
        Eigen::Map<Eigen::Vector3s> measured_acc_;
        Eigen::Map<Eigen::Vector3s> measured_gyro_;

        // Maps to the pos, quat and vel of the pre-integrated delta
        Eigen::Map<Eigen::Vector3s> position_preint_;
        Eigen::Map<Eigen::Quaternions> orientation_preint_quat_;
        Eigen::Map<Eigen::Vector3s> velocity_preint_;

        // Maps to pos, quat, vel, to be used as temporaries
        Eigen::Map<const Eigen::Vector3s> p_in_1_, p_in_2_;
        Eigen::Map<Eigen::Vector3s> p_out_;
        Eigen::Map<const Eigen::Quaternions> q_in_1_, q_in_2_;
        Eigen::Map<Eigen::Quaternions> q_out_;
        Eigen::Map<const Eigen::Vector3s> v_in_1_, v_in_2_;
        Eigen::Map<Eigen::Vector3s> v_out_;


        // Helper functions to remap several magnitudes
        void remapPQV(const Eigen::VectorXs& _delta1, const Eigen::VectorXs& _delta2, Eigen::VectorXs& _delta_out);
        void remapDelta(Eigen::VectorXs& _delta_out);
        void remapData(const Eigen::VectorXs& _data);

        ///< COVARIANCE OF: [PreintPOSITION PreintVELOCITY PreintROTATION]
        ///< (first-order propagation from *measurementCovariance*).
        Eigen::Matrix<Scalar,9,9> preint_meas_cov_;

        ///Jacobians
        Eigen::Matrix<Scalar,6,3> preintegrated_H_biasAcc_; /// [dP dv]
        Eigen::Matrix<Scalar,10,3> preintegrated_H_biasOmega_; /// [dP dv dq]
        Eigen::Matrix<Scalar,9,9> delta_cov_;

    public:
        static ProcessorBase* create(const std::string& _unique_name, const ProcessorParamsBase* _params);
};

}

/////////////////////////////////////////////////////////
// IMPLEMENTATION. Put your implementation includes here
/////////////////////////////////////////////////////////

// Wolf
#include "state_block.h"

// STL
#include <cmath> //needed to compute logMapDerivative (right jacobian Jr)

namespace wolf{

inline const Eigen::Vector3s& ProcessorIMU::getGravity() const
{
    return gravity_;
}

inline void ProcessorIMU::data2delta(const Eigen::VectorXs& _data, const Eigen::MatrixXs& _data_cov, const Scalar _dt)
{
    assert(_data.size() == data_size_ && "Wrong data size!");

    // remap
    remapData(_data);
    remapDelta(delta_);
    // delta_ is _out_

    /* MATHS of delta creation -- Sola-16
     * dp = 1/2 * (a-a_b) * dt^2 = 1/2 * dv * dt
     * dv = (a-a_b) * dt
     * dq = exp((w-w_b)*dt)
     */

    // create delta
    //Use SOLA-16 convention by default
    v_out_ = (measured_acc_ - bias_acc_) * _dt;
    p_out_ = v_out_ * _dt / 2;
    Eigen::v2q((measured_gyro_ - bias_gyro_) * _dt, q_out_); // q_out_

    //Compute jacobian of delta wrt data

    /*                  MATHS : jacobians
     * substituting (a-a_b) and (w-w_b) respectively by (a-a_b+a_n) and (w-w_b+w_n) (measurement noise is additive)
     *         an        wn
     *   dp [0.5*dt*dt   0  ]       
     *   dv [   dt       0  ]
     *   dR [   0      dt*exp(wj*dt) ]
     */

     Eigen::Matrix<wolf::Scalar,9,6> jacobian_delta_noise = Eigen::Matrix<wolf::Scalar,9,6>::Zero();
     jacobian_delta_noise.block<3,3>(0,0) = Eigen::Matrix3s::Identity() * 0.5 * _dt * _dt;
     jacobian_delta_noise.block<3,3>(3,0) = Eigen::Matrix3s::Identity() * _dt;
     jacobian_delta_noise.block<3,3>(6,3) = Eigen::Matrix3s::Identity() * _dt * skew(measured_gyro_ * _dt); //not so sure about this one

     delta_cov_ = jacobian_delta_noise * _data_cov * jacobian_delta_noise.transpose();
}

inline void ProcessorIMU::deltaPlusDelta(const Eigen::VectorXs& _delta_preint, const Eigen::VectorXs& _delta,
                                         const Scalar _dt, Eigen::VectorXs& _delta_preint_plus_delta,
                                         Eigen::MatrixXs& _jacobian1, Eigen::MatrixXs& _jacobian2)
{
    deltaPlusDelta(_delta_preint, _delta, _dt, _delta_preint_plus_delta);

    /*                                  JACOBIANS according to FORSTER
     *                                      For noise integration :
     * in compact Matrix form, with N_D(i,k) = [R_n(i,k) DV_n(i,k) DP_n(i,k)] and IMU measurement noise N_d(k) = [w_n a_n]
     * with (i,k) meaning "from i to k" and R_n, V_n and P_n respectively the noises associated to Delta_rotation, Delta_velocity and Delta_position
     * We note DR(i,k), DV(i,k) and DP(i,k) respectively the integrated Deltas in rotation, velocity and position from i to k
     * Dt is the total integration time (from origin to current)
     *
     * we have :
     * N_D(i,j) = A(j-1) * N_D(i,j-1) + B(j-1) * N_d(j-1)
     *
     * with A = [DR(j-1,j)                                  0   0
     *          -DR(i,j)*(a(j-1) - a_b(i))*Dt^               1   0
     *          -(1/2)*DR(i,,j-1)*(a(j-1) - a_b(i))*Dt*Dt    Dt  1]
     *
     *     B = [Jr(j-1)*Dt          0
     *              0          DR(i,j-1)*Dt
     *              0       (1/2)*DR(i,j-1)*Dt*Dt]
     *
     * WARNING : (a(j-1) - a_b(i)) is _data.head(3) : means that this operation does not make sense if we compose two integrated Deltas
     */

     /// FORSTER version
     /*
     _jacobian1.resize(3,9);
     _jacobian1.setZero();
     _jacobian1.block<1,3>(1,0) = Eigen::vee(q_in_1_.toRotationMatrix()).transpose(); //check if this is working --> block considered as row_vector ?
     _jacobian1.block<1,3>(2,0) = Eigen::vee(q_in_1_.toRotationMatrix()).transpose() * (-_Dt2); // *_data.head(3)
     _jacobian1.block<1,3>(0,0) = Eigen::vee(q_in_1_.toRotationMatrix()).transpose() * _Dt2 * (-_Dt2/2); // *_data.head(3)
     //Need access to _data here.
     _jacobian1.block<1,3>(0,6) << 1,1,1;
     _jacobian1.block<1,3>(2,3) << 1,1,1;
     _jacobian1.block<1,3>(0,3) << _Dt2, _Dt2, _Dt2;
     */

     /*
      * _jacobian1 and _jacobian2 are jacobians of _delta1_plus_delta2 w.r.t. _delta1 and _delta2
      * let us note this : D3 = D1 (+) D2, with D=[DP, DV, DR] (We will use the minimal form here) 
      * Note : PVQ FORMULATION
      *
      * _jacobian1 =    [1  _Dt2    DP2                     _jacobian2 =    [DR1   0   0
      *                  0    1     DV2                                       0   DR1  0  
      *                  0    1     DR2 ]                                     0    0  DR1]
      */
      Eigen::Matrix3s DR1 = q_in_1_.toRotationMatrix();

      _jacobian1.resize(9,9);
      _jacobian1 = Eigen::Matrix<wolf::Scalar,9,9>::Identity();
      _jacobian1.block<3,3>(0,3) = Eigen::Matrix3s::Identity() * _dt;
      //_jacobian1.block<3,3>(0,6) = Eigen::Matrix3s::Identity() * p_in_2_; // FIXME: THIS IS FALSE --> COEFFICIENT-WISE NEEDED
      //_jacobian1.block<3,3>(3,6) = Eigen::Matrix3s::Identity() * v_in_2_; // FIXME: THIS IS FALSE --> COEFFICIENT-WISE NEEDED
      _jacobian1.block<3,3>(6,6) = q_in_2_.toRotationMatrix();
      _jacobian1.block<3,3>(3,0) = Eigen::Matrix3s::Identity();

      _jacobian2.resize(9,9);
      _jacobian2.setZero();
      _jacobian2.block<3,3>(0,0) = DR1;
      _jacobian2.block<3,3>(3,3) = DR1;
      _jacobian2.block<3,3>(6,6) = DR1;

     /*
     *                                  For biases :
     * The jacobians wrt the biases have the following form, derived from \cite{FORSTER}
     *
     * preintegrated_H_biasAcc_ (6 x 3) =   [ dP/db_a
     *                                        dv/db_a
     *                                                  ] // Note that there is no jacobian of the orientation wrt
     *                                                    // the accelerometer bias
     *
     * preintegrated_H_biasOmega_ (9 x 3) = [ dP/db_g
     *                                        dv/db_g
     *                                        dq/db_g  ] // Note that the orientation jacobian is converted to minimal form
     *
     */

     /// Get the rotation matrix associated to the preintegrated orientation quaternion
     Eigen::Matrix3s orientation_preint_rot_ =  orientation_preint_quat_.toRotationMatrix();

     /// dP/db_a -- Jacobian of postion w.r.t accelerometer bias
     /// dP/db_a += dv/db_a - 0.5 * delta_R * dt * dt
     preintegrated_H_biasAcc_.topRows<3>() += preintegrated_H_biasAcc_.bottomRows<3>() * _dt
                                           -  0.5 * _dt * _dt * orientation_preint_rot_;

     /// dv/db_a -- Jacobian of velocity w.r.t accelerometer bias
     /// dv/db_a -= delta_R * dt
     preintegrated_H_biasAcc_.bottomRows<3>() -= orientation_preint_rot_ * _dt;

     /// dP/db_g -- Jacobian of position w.r.t gyro bias
     /// dP/db_g += dv/db_g * dt - 0.5 * delta_R * (a - b_a)^ * dt * dt * dR/db_g
     preintegrated_H_biasOmega_.topRows<3>() += preintegrated_H_biasOmega_.block<3,3>(3,0) * _dt
                                             -  0.5 * _dt * _dt * orientation_preint_rot_ * Eigen::skew(measured_acc_ - bias_acc_) * preintegrated_H_biasOmega_.bottomRows<3>();

     /// dv/db_g -- Jacobian of velocity w.r.t gyro bias
     /// dv/db_g -= delta_R * dt * (a - b_a)^ * dR/db_g
     preintegrated_H_biasOmega_.block<3,3>(3,0) -= orientation_preint_rot_ *_dt * Eigen::skew(measured_acc_ - bias_acc_) * preintegrated_H_biasOmega_.bottomRows<3>();

     /// dR/db_g -- Jacobian of orientation w.r.t gyro bias
     /// dR/db_g += R.t * dR/db_g * Jr * dt       where Jr  == right Jacobian
     ///                                                R.t == [exp((omega - b_g) * dt)]^{-1}
     /***** FIXME: Optimise this ! ******/
     // JS: if q_tmp = exp(w*dt) then q_tmp.conjugate() = exp(-w*dt) !
     // JS: Or use Rodrigues formula to go directly to R_eq.
     Eigen::Quaternions q_tmp = Eigen::v2q((measured_gyro_ - bias_gyro_) * _dt);
     Eigen::Matrix3s R_eq = q_tmp.conjugate().toRotationMatrix();
     /**********************************/
     preintegrated_H_biasOmega_.bottomRows<3>() = R_eq *  preintegrated_H_biasOmega_.bottomRows<3>() * _dt * expMapDerivative((measured_gyro_ - bias_gyro_) * _dt);
}

inline void ProcessorIMU::deltaPlusDelta(const Eigen::VectorXs& _delta_preint, const Eigen::VectorXs& _delta,
                                         const Scalar _dt, Eigen::VectorXs& _delta_preint_plus_delta)
{
    assert(_delta_preint.size() == 10 && "Wrong _delta1 vector size");
    assert(_delta.size() == 10 && "Wrong _delta2 vector size");
    assert(_delta_preint_plus_delta.size() == 10 && "Wrong _delta1_plus_delta2 vector size");

    remapPQV(_delta_preint, _delta, _delta_preint_plus_delta);
    // _delta1              is _in_1_
    // _delta2              is _in_2_
    // _delta1_plus_delta2  is _out_


    /* MATHS according to Sola-16
     * Dp' = Dp + Dv*dt + 1/2*Dq*(a-a_b)*dt^2    = Dp + Dv*dt + Dq*dp   if  dp = 1/2*(a-a_b)*dt^2
     * Dv' = Dv + Dq*(a-a_b)*dt                  = Dv + Dq*dv           if  dv = (a-a_b)*dt
     * Dq' = Dq * exp((w-w_b)*dt)                = Dq * dq              if  dq = exp((w-w_b)*dt)
     *
     * where (dp, dv, dq) need to be computed in data2delta(), and Dq*dx =is_equivalent_to= Dq*dx*Dq'.
     *
     * warning: All deltas (Dp, Dv, Dq) are physically interpretable: they represent the position, velocity and orientation of a body with
     * respect to a reference frame that is non-rotating and free-falling at the acceleration of gravity.
     */

    // delta pre-integration
    // For the math, use SOLA-16 convention by default
    // Note: we might be (and in fact we are) calling this fcn with the same input and output:
    //     deltaPlusDelta(delta_integrated_, delta_, dt_, delta_integrated_);
    // that is, _delta1 and _delta1_plus_delta2 point to the same memory locations.
    // Therefore, to avoid aliasing, we proceed in the order p -> v -> q
    p_out_ = p_in_1_ + v_in_1_ * _dt + q_in_1_ * p_in_2_;
    v_out_ = v_in_1_ + q_in_1_ * v_in_2_;
    q_out_ = q_in_1_ * q_in_2_;

}

inline void ProcessorIMU::xPlusDelta(const Eigen::VectorXs& _x, const Eigen::VectorXs& _delta, const Scalar _Dt,
                                     Eigen::VectorXs& _x_plus_delta)
{
    assert(_x.size() == 16 && "Wrong _x vector size");
    assert(_delta.size() == 10 && "Wrong _delta vector size");
    assert(_x_plus_delta.size() == 16 && "Wrong _x_plus_delta vector size");
    assert(_Dt > 0 && "Time interval _Dt is not positive!");

    remapPQV(_x, _delta, _x_plus_delta);
    // _x               is _in_1_
    // _delta           is _in_2_
    // _x_plus_delta    is _out_

    Eigen::Vector3s gdt = gravity_ * _Dt;

    // state updates
    p_out_ = q_in_1_ * p_in_2_ + p_in_1_ + v_in_1_ * _Dt + gdt * _Dt / 2 ;
    v_out_ = q_in_1_ * v_in_2_ + v_in_1_ + gdt;
    q_out_ = q_in_1_ * q_in_2_;

    // bypass constant biases
    _x_plus_delta.tail(6) = _x.tail(6);
}

inline void ProcessorIMU::integrateDelta()
{
    /* In the case of IMU, updating the pre-integrated measurements
     * corresponds to adding latest delta to the pre-integrated measurements
     * with the composition rules defined by the state :
     *
     * delta_preintegrated_ik = delta_preintegrated_ij (+) delta_jk
     *
     * */
    deltaPlusDelta(delta_integrated_, delta_, dt_, delta_integrated_);
}

inline Eigen::VectorXs ProcessorIMU::deltaZero() const
{
    return (Eigen::VectorXs(10) << 0,0,0,  0,0,0,1,  0,0,0 ).finished(); // p, q, v
}

inline Motion ProcessorIMU::interpolate(const Motion& _motion_ref, Motion& _motion, TimeStamp& _ts)
{
    Motion tmp(_motion_ref);
    tmp.ts_ = _ts;
    tmp.delta_ = deltaZero();
    tmp.delta_cov_ = Eigen::MatrixXs::Zero(delta_size_, delta_size_);
    return tmp;
}


inline void ProcessorIMU::resetDerived()
{
    // Remap biases for the integration at the origin frame's biases
    frame_imu_ptr_ = (FrameIMU*)((origin_ptr_->getFramePtr()));
    new (&bias_acc_)  Eigen::Map<const Eigen::Vector3s>(frame_imu_ptr_->getBAPtr()->getVector().data()); // acc  bias
    new (&bias_gyro_) Eigen::Map<const Eigen::Vector3s>(frame_imu_ptr_->getBGPtr()->getVector().data()); // gyro bias
}


inline ConstraintBase* ProcessorIMU::createConstraint(FeatureBase* _feature_motion, FrameBase* _frame_origin)
{
    // return new ConstraintIMU(_feature_motion, _frame_origin);
    return nullptr;
}

inline void ProcessorIMU::remapPQV(const Eigen::VectorXs& _delta1, const Eigen::VectorXs& _delta2, Eigen::VectorXs& _delta_out)
{
    new (&p_in_1_) Eigen::Map<const Eigen::Vector3s>(_delta1.data());
    new (&q_in_1_) Eigen::Map<const Eigen::Quaternions>(_delta1.data() + 3);
    new (&v_in_1_) Eigen::Map<const Eigen::Vector3s>(_delta1.data() + 7);

    new (&p_in_2_) Eigen::Map<const Eigen::Vector3s>(_delta2.data());
    new (&q_in_2_) Eigen::Map<const Eigen::Quaternions>(_delta2.data() + 3);
    new (&v_in_2_) Eigen::Map<const Eigen::Vector3s>(_delta2.data() + 7);

    new (&p_out_) Eigen::Map<Eigen::Vector3s>(_delta_out.data());
    new (&q_out_) Eigen::Map<Eigen::Quaternions>(_delta_out.data() + 3);
    new (&v_out_) Eigen::Map<Eigen::Vector3s>(_delta_out.data() + 7);
}

inline void ProcessorIMU::remapDelta(Eigen::VectorXs& _delta_out)
{
    new (&p_out_) Eigen::Map<Eigen::Vector3s>(_delta_out.data());
    new (&q_out_) Eigen::Map<Eigen::Quaternions>(_delta_out.data() + 3);
    new (&v_out_) Eigen::Map<Eigen::Vector3s>(_delta_out.data() + 7);
}

inline void ProcessorIMU::remapData(const Eigen::VectorXs& _data)
{
    new (&measured_acc_) Eigen::Map<const Eigen::Vector3s>(_data.data());
    new (&measured_gyro_) Eigen::Map<const Eigen::Vector3s>(_data.data() + 3);
}

inline Eigen::Matrix3s ProcessorIMU::logMapDerivative(const Eigen::Vector3s& _omega)
{
    using std::cos;
    using std::sin;

    Scalar theta2 = _omega.dot(_omega);
    if (theta2 <= Constants::EPS_SMALL)
        return Eigen::Matrix3s::Identity(); //Or should we use
    Scalar theta = std::sqrt(theta2);  // rotation angle
    Eigen::Matrix3s W;
    W = Eigen::skew(_omega);
    Eigen::Matrix3s m1;
    m1.noalias() = (1 / theta2 - (1 + cos(theta)) / (2 * theta * sin(theta))) * (W * W);
    return Eigen::Matrix3s::Identity() + 0.5 * W + m1; //is this really more optimized?
}


Eigen::Matrix3s ProcessorIMU::expMapDerivative(const Eigen::Vector3s& _omega)
{
    using std::cos;
    using std::sin;

    Scalar theta2 = _omega.dot(_omega);
    if (theta2 <= Constants::EPS_SMALL)
        return Eigen::Matrix3s::Identity();
    Scalar theta = std::sqrt(theta2);  // rotation angle
    Eigen::Matrix3s W;
    W = Eigen::skew(_omega);
    Eigen::Matrix3s m1, m2;
    m1.noalias() = (1 - cos(theta)) / theta2 * W;
    m2.noalias() = (theta - sin(theta)) / (theta2 * theta) * (W * W);
    return Eigen::Matrix3s::Identity() + m1 + m2;
}


} // namespace wolf

#endif // PROCESSOR_IMU_H

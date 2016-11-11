#include "processor_odom_3D.h"
namespace wolf
{

ProcessorOdom3D::ProcessorOdom3D() :
        ProcessorMotion("ODOM 3D", 7, 7, 6, 6),
        k_disp_to_disp_(0.1), k_disp_to_rot_(0.1), k_rot_to_rot_(0.1),
        min_disp_var_(0.1), min_rot_var_(0.1),
        p1_(nullptr), p2_(nullptr), p_out_(nullptr),
        q1_(nullptr), q2_(nullptr), q_out_(nullptr)
{
    jacobian_delta_preint_.setIdentity(delta_cov_size_, delta_cov_size_);
    jacobian_delta_.setIdentity(delta_cov_size_, delta_cov_size_);
}

ProcessorOdom3D::~ProcessorOdom3D()
{
}

void ProcessorOdom3D::setup(SensorOdom3D::Ptr sen_ptr)
{
    k_disp_to_disp_ = sen_ptr->getDispVarToDispNoiseFactor();
    k_disp_to_rot_ = sen_ptr->getDispVarToRotNoiseFactor();
    k_rot_to_rot_ = sen_ptr->getRotVarToRotNoiseFactor();
    min_disp_var_ = sen_ptr->getMinDispVar();
    min_rot_var_ = sen_ptr->getMinRotVar();
}

void ProcessorOdom3D::data2delta(const Eigen::VectorXs& _data, const Eigen::MatrixXs& _data_cov, const Scalar _dt)
{
    assert((_data.size() == 6 || _data.size() == 7) && "Wrond data size. Must be 6 or 7 for 3D.");
    Scalar disp, rot; // displacement and rotation of this motion step
    if (_data.size() == 6)
    {
        // rotation in vector form
        delta_.head<3>() = _data.head<3>();
        new (&q_out_) Eigen::Map<Eigen::Quaternions>(delta_.data() + 3);
        q_out_ = v2q(_data.tail<3>());
        disp = _data.head<3>().norm();
        rot = _data.tail<3>().norm();
    }
    else
    {
        // rotation in quaternion form
        delta_ = _data;
        disp = _data.head<3>().norm();
        rot = 2 * acos(_data(3));
    }
    /* Jacobians of d = data2delta(data, dt)
     * with: d =    [Dp Dq]
     *       data = [dp do]
     *
     *       Dp = dp
     *       Dq = v2q(do)
     *
     * dDp/ddp = I
     * dDp/ddo = 0
     * dDo/ddp = 0
     * dDo/ddo = I
     *
     * so, J = I, and delta_cov = _data_cov
     */
    // We discard _data_cov and create a new one from the measured motion
    Scalar disp_var = min_disp_var_ + k_disp_to_disp_ * disp;
    Scalar rot_var = min_rot_var_ + k_disp_to_rot_ * disp + k_rot_to_rot_ * rot;
    Eigen::Matrix6s data_cov(Eigen::Matrix6s::Identity());
    data_cov(0, 0) = data_cov(1, 1) = data_cov(2, 2) = disp_var;
    data_cov(3, 3) = data_cov(4, 4) = data_cov(5, 5) = rot_var;
    delta_cov_ = data_cov;
}

void ProcessorOdom3D::deltaPlusDelta(const Eigen::VectorXs& _delta1, const Eigen::VectorXs& _delta2, const Scalar _Dt2,
                                     Eigen::VectorXs& _delta1_plus_delta2, Eigen::MatrixXs& _jacobian1,
                                     Eigen::MatrixXs& _jacobian2)
{
    assert(_delta1.size() == delta_size_ && "Wrong _delta1 vector size");
    assert(_delta2.size() == delta_size_ && "Wrong _delta2 vector size");
    assert(_delta1_plus_delta2.size() == delta_size_ && "Wrong _delta1_plus_delta2 vector size");
    assert(_jacobian1.rows() == delta_cov_size_ && _jacobian1.cols() == delta_cov_size_ && "Wrong _jacobian1 size");
    assert(_jacobian2.rows() == delta_cov_size_ && _jacobian2.cols() == delta_cov_size_ && "Wrong _jacobian2 size");
    remap(_delta1, _delta2, _delta1_plus_delta2);
    /* Jacobians of D' = D (+) d
     * with: D = [Dp Dq]
     *       d = [dp dq]
     *
     * dDp'/dDp = I
     * dDp'/dDo = -DR * skew(dp)   // (Sola 16, ex. B.3.2 and Sec. 7.2.3)
     * dDo'/dDp = 0
     * dDo'/dDo = dR.tr            // (Sola 16, Sec. 7.2.3)
     *
     * dDp'/ddp = DR
     * dDp'/ddo = 0
     * dDo'/ddp = 0
     * dDo'/ddo = I
     */
    // temporaries
    Eigen::Matrix3s DR = q1_.matrix();
    Eigen::Matrix3s dR = q2_.matrix();
    // fill Jacobians -- parts not shown are constant and set at construction time.
    _jacobian1.block<3, 3>(0, 3) = -DR * skew(p2_); // (Sola 16, ex. B.3.2 and Sec. 7.2.3)
    _jacobian1.block<3, 3>(3, 3) = dR.transpose(); // (Sola 16, Sec. 7.2.3)
    _jacobian2.block<3, 3>(0, 0) = DR; // (Sola 16, Sec. 7.2.3)
    // perform composition here to avoid aliasing problems if _delta1 and _delta_plus_delta share the same storage
    p_out_ = p1_ + q1_ * p2_;
    q_out_ = q1_ * q2_;
}

void ProcessorOdom3D::deltaPlusDelta(const Eigen::VectorXs& _delta1, const Eigen::VectorXs& _delta2, const Scalar _Dt2,
                                     Eigen::VectorXs& _delta1_plus_delta2)
{
    assert(_delta1.size() == delta_size_ && "Wrong _delta1 vector size");
    assert(_delta2.size() == delta_size_ && "Wrong _delta2 vector size");
    assert(_delta1_plus_delta2.size() == delta_size_ && "Wrong _delta1_plus_delta2 vector size");
    remap(_delta1, _delta2, _delta1_plus_delta2);
    p_out_ = p1_ + q1_ * p2_;
    q_out_ = q1_ * q2_;
}

void ProcessorOdom3D::xPlusDelta(const Eigen::VectorXs& _x, const Eigen::VectorXs& _delta, const Scalar _Dt,
                                 Eigen::VectorXs& _x_plus_delta)
{
    assert(_x.size() == x_size_ && "Wrong _x vector size");
    assert(_delta.size() == delta_size_ && "Wrong _delta vector size");
    assert(_x_plus_delta.size() == x_size_ && "Wrong _x_plus_delta vector size");
    remap(_x, _delta, _x_plus_delta);
    p_out_ = p1_ + q1_ * p2_;
    q_out_ = q1_ * q2_;
}

Motion ProcessorOdom3D::interpolate(const Motion& _motion_ref, Motion& _motion, TimeStamp& _ts)
{
    using namespace Eigen;
    // Interpolate between motion_ref and motion, as in:
    //
    // motion_ref ------ ts_ ------ motion
    //                 return
    //
    // and return the value at the given time_stamp ts_.
    //
    // The position receives linear interpolation:
    //    p_ret = (ts - t_ref) / dt * (p - p_ref)
    //
    // the quaternion receives a slerp interpolation
    //    q_ret = q_ref.slerp( (ts - t_ref) / dt , q);
    //
    // See extensive documentation in ProcessorMotion::interpolate().
    WOLF_TRACE("");
    // reference
    TimeStamp t_ref = _motion_ref.ts_;
    // final
    TimeStamp t = _motion.ts_;
    Map<VectorXs> dp(_motion.delta_.data(), 3);
    Map<Quaternions> dq(_motion.delta_.data() + 3);
    // interpolated
    Motion motion_int = motionZero(_ts);
    Map<VectorXs> dp_int(motion_int.delta_.data(), 3);
    Map<Quaternions> dq_int(motion_int.delta_.data() + 3);
    // Jacobians for covariance propagation
    MatrixXs J_ref(delta_cov_size_, delta_cov_size_);
    MatrixXs J_int(delta_cov_size_, delta_cov_size_);
    // interpolate delta
    Scalar tau = (_ts - t_ref) / (t - t_ref); // interpolation factor (0 to 1)
    motion_int.ts_ = _ts;
    dp_int = tau * dp;
    dq_int = Quaternions::Identity().slerp(tau, dq);
    deltaPlusDelta(_motion_ref.delta_integr_, motion_int.delta_, (t - t_ref), motion_int.delta_integr_, J_ref, J_int);
    // interpolate covariances
    WOLF_TRACE("J_ref\n", J_ref);
    WOLF_TRACE("J_int\n", J_int);
    motion_int.delta_cov_ = tau * _motion.delta_cov_;
    WOLF_TRACE("d_cov\n", _motion.delta_cov_);
    WOLF_TRACE("D_cov\n", _motion_ref.delta_integr_cov_);
    motion_int.delta_integr_cov_ = J_ref * _motion_ref.delta_integr_cov_ * J_ref.transpose()
            + J_int * _motion.delta_cov_ * J_int.transpose();
    WOLF_TRACE("");
    // update second delta ( in place update )
    dp = dq_int.conjugate() * ((1 - tau) * dp);
    dq = dq_int.conjugate() * dq;
    _motion.delta_cov_ = (1 - tau) * _motion.delta_cov_; // easy interpolation // TODO check for correcness
    //Dp            = Dp; // trivial, just leave the code commented
    //Dq            = Dq; // trivial, just leave the code commented
    //_motion.delta_integr_cov_ = _motion.delta_integr_cov_; // trivial, just leave the code commented
    return motion_int;
}

ProcessorBasePtr ProcessorOdom3D::create(const std::string& _unique_name, const ProcessorParamsBasePtr _params, const SensorBasePtr _sen_ptr)
{
    std::shared_ptr<ProcessorOdom3D> prc_ptr = std::make_shared<ProcessorOdom3D>();
    prc_ptr->setup(std::static_pointer_cast<SensorOdom3D>(_sen_ptr));
    prc_ptr->setName(_unique_name);
    return prc_ptr;
}

}


// Register in the SensorFactory
#include "processor_factory.h"
namespace wolf {
WOLF_REGISTER_PROCESSOR("ODOM 3D", ProcessorOdom3D)
} // namespace wolf

#ifndef _FACTOR_AUTODIFF_APRILTAG_H_
#define _FACTOR_AUTODIFF_APRILTAG_H_

//Wolf includes
#include "core/common/wolf.h"
#include "core/math/rotations.h"
#include "core/factor/factor_autodiff.h"
#include "core/sensor/sensor_base.h"
#include "apriltag/landmark/landmark_apriltag.h"
#include "apriltag/feature/feature_apriltag.h"

namespace wolf
{

WOLF_PTR_TYPEDEFS(FactorAutodiffApriltag);

class FactorAutodiffApriltag : public FactorAutodiff<FactorAutodiffApriltag, 6, 3, 4, 3, 4, 3, 4>
{
    public:

        /** \brief Class constructor
         */
        FactorAutodiffApriltag(
                const SensorBasePtr& _sensor_ptr,
                const FrameBasePtr& _frame_ptr,
                const LandmarkApriltagPtr& _landmark_other_ptr,
                const FeatureApriltagPtr& _feature_ptr,
                bool _apply_loss_function,
                FactorStatus _status);

        /** \brief Class Destructor
         */
        virtual ~FactorAutodiffApriltag();
 
        /** brief : compute the residual from the state blocks being iterated by the solver.
         **/
        template<typename T>
        bool operator ()( const T* const _p_camera, 
                          const T* const _o_camera, 
                          const T* const _p_keyframe, 
                          const T* const _o_keyframe, 
                          const T* const _p_landmark, 
                          const T* const _o_landmark, 
                          T* _residuals) const;

        Eigen::Vector6s residual() const;
        Scalar cost() const;

        // print function only for double (not Jet)
        template<typename T, int Rows, int Cols>
        void print(int kf, int lmk, const std::string s, const Eigen::Matrix<T, Rows, Cols> _M) const
        {
            // jet prints nothing
        }
        template<int Rows, int Cols>
        void print(int kf, int lmk, const std::string s, const Eigen::Matrix<Scalar, Rows, Cols> _M) const
        {
            // double prints stuff
            WOLF_TRACE("KF", kf, " L", lmk, "; ", s, _M);
        }
};

} // namespace wolf
 
// Include here all headers for this class
//#include <YOUR_HEADERS.h>

namespace wolf
{

FactorAutodiffApriltag::FactorAutodiffApriltag(
        const SensorBasePtr& _sensor_ptr,
        const FrameBasePtr& _frame_ptr,
        const LandmarkApriltagPtr& _landmark_other_ptr,
        const FeatureApriltagPtr& _feature_ptr,
        bool _apply_loss_function,
        FactorStatus _status) :
            FactorAutodiff("AUTODIFF APRILTAG",
                               nullptr,
                               nullptr,
                               nullptr,
                               _landmark_other_ptr,
                               nullptr,
                               _apply_loss_function,
                               _status,
                               _sensor_ptr->getP(),         _sensor_ptr->getO(),
                               _frame_ptr->getP(),          _frame_ptr->getO(),
                               _landmark_other_ptr->getP(), _landmark_other_ptr->getO()
                               )
{


}

/** \brief Class Destructor
 */
FactorAutodiffApriltag::~FactorAutodiffApriltag()
{
    //
}

template<typename T> bool FactorAutodiffApriltag::operator ()( const T* const _p_camera, const T* const _o_camera, const T* const _p_keyframe, const T* const _o_keyframe, const T* const _p_landmark, const T* const _o_landmark, T* _residuals) const
{
    // Maps
    Eigen::Map<const Eigen::Matrix<T,3,1>> p_r_c(_p_camera);
    Eigen::Map<const Eigen::Quaternion<T>> q_r_c(_o_camera);
    Eigen::Map<const Eigen::Matrix<T,3,1>> p_w_r(_p_keyframe);
    Eigen::Map<const Eigen::Quaternion<T>> q_w_r(_o_keyframe);
    Eigen::Map<const Eigen::Matrix<T,3,1>> p_w_l(_p_landmark);
    Eigen::Map<const Eigen::Quaternion<T>> q_w_l(_o_landmark);
    Eigen::Map<Eigen::Matrix<T,6,1>> residuals(_residuals);

    // Expected measurement
    Eigen::Quaternion<T> q_c_w = (q_w_r * q_r_c).conjugate();
    Eigen::Quaternion<T> q_c_l = q_c_w * q_w_l;
    Eigen::Matrix<T,3,1> p_c_l = q_c_w * (-(p_w_r + q_w_r * p_r_c) + p_w_l);

    // Measurement
    Eigen::Vector3s      p_c_l_meas(getMeasurement().head<3>());
    Eigen::Quaternions   q_c_l_meas(getMeasurement().data() + 3 );
    Eigen::Quaternion<T> q_l_c_meas = q_c_l_meas.conjugate().cast<T>();
    //Eigen::Matrix<T,3,1> p_l_c_meas = -q_l_c_meas * p_c_l_meas.cast<T>();

    // Error
    Eigen::Matrix<T, 6, 1> err;
    err.head(3) = q_l_c_meas * (p_c_l_meas.cast<T>() - p_c_l);
    //err.tail(3) = wolf::log_q(q_l_c_meas * q_c_l);
    err.tail(3) = T(2)*(q_l_c_meas * q_c_l).vec();

    // Residual
    residuals = getMeasurementSquareRootInformationUpper().cast<T>() * err;

    /*//states
    Eigen::Translation<T,3> p_camera    (_p_camera[0]  , _p_camera[1]  , _p_camera[2]),
                            p_keyframe  (_p_keyframe[0], _p_keyframe[1], _p_keyframe[2]),
                            p_landmark  (_p_landmark[0], _p_landmark[1], _p_landmark[2]);
    Eigen::Quaternion<T> q_camera   (_o_camera),
                         q_keyframe (_o_keyframe),
                         q_landmark (_o_landmark);

    //Measurements T and Q
    Eigen::Translation3ds  p_measured(getMeasurement().head(3));
    Eigen::Quaternions     q_measured(getMeasurement().data() + 3 );
    // landmark wrt camera, measure
    Eigen::Transform<T, 3, Eigen::Isometry> c_M_l_meas = p_c_l_meas.cast<T>() * q_measured.cast<T>();

    // Create transformation matrices to compose
    // robot wrt world
    Eigen::Transform<T, 3, Eigen::Isometry> w_M_r = p_keyframe * q_keyframe;
    // camera wrt robot
    Eigen::Transform<T, 3, Eigen::Isometry> r_M_c = p_camera * q_camera;
    // landmark wrt world
    Eigen::Transform<T, 3, Eigen::Isometry> w_M_l = p_landmark * q_landmark;
    // landmark wrt camera, estimated
    Eigen::Transform<T, 3, Eigen::Isometry> c_M_l_est = (w_M_r * r_M_c).inverse() * w_M_l;

    // expectation error, in camera frame
    // right-minus
    Eigen::Transform<T, 3, Eigen::Isometry> c_M_err = c_M_l_meas.inverse() * c_M_l_est;
    // opposite of the previous formula and therefore equivalent
//    Eigen::Transform<T, 3, Eigen::Isometry> c_M_err = c_M_l_est.inverse() * c_M_l_meas;


    // error
    Eigen::Matrix<T, 6, 1> err;
    err.block(0,0,3,1) = c_M_err.translation();
    Eigen::Matrix<T, 3, 3> R_err(c_M_err.linear());
    err.block(3,0,3,1) = wolf::log_R(R_err);

    // debug stuff
//    int kf = getFeature()->getCapture()->getFrame()->id();
//    int lmk = getLandmarkOther()->id();
//
//    print(kf, lmk, "w_M_c  : \n", (w_M_r*r_M_c).matrix());
//    print(kf, lmk, "w_M_l  : \n", w_M_l.matrix());
//    print(kf, lmk, "c_M_l_e: \n", c_M_l_est.matrix());
//    print(kf, lmk, "c_M_l_m: \n", c_M_l_meas.matrix());
//    print(kf, lmk, "error  : \n", err.transpose().eval());

    // residual
    Eigen::Map<Eigen::Matrix<T, 6, 1>> res(_residuals);

    res = getFeature()->getMeasurementSquareRootInformationUpper().cast<T>() * err;
    */
    return true;
}

Eigen::Vector6s FactorAutodiffApriltag::residual() const
{
    Eigen::Vector6s res;
    Scalar * p_camera, * o_camera, * p_frame, * o_frame, * p_tag, * o_tag;
    p_camera = getCapture()->getSensorP()->getState().data();
    o_camera = getCapture()->getSensorO()->getState().data();
    p_frame  = getCapture()->getFrame()->getP()->getState().data();
    o_frame  = getCapture()->getFrame()->getO()->getState().data();
    p_tag    = getLandmarkOther()->getP()->getState().data();
    o_tag    = getLandmarkOther()->getO()->getState().data();

    operator() (p_camera, o_camera, p_frame, o_frame, p_tag, o_tag, res.data());

    return res;
}

Scalar FactorAutodiffApriltag::cost() const
{
    return residual().squaredNorm();
}

} // namespace wolf

#endif /* _CONSTRAINT_AUTODIFF_APRILTAG_H_ */

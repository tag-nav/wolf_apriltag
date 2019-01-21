#ifndef _CONSTRAINT_AUTODIFF_APRILTAG_H_
#define _CONSTRAINT_AUTODIFF_APRILTAG_H_

//Wolf includes
#include "wolf.h"
#include "rotations.h"
#include "constraint_autodiff.h"
#include "sensor_base.h"
#include "landmark_apriltag.h"
#include "features/feature_apriltag.h"

namespace wolf
{

WOLF_PTR_TYPEDEFS(ConstraintAutodiffApriltag);

class ConstraintAutodiffApriltag : public ConstraintAutodiff<ConstraintAutodiffApriltag, 6, 3, 4, 3, 4, 3, 4>
{
    public:

        /** \brief Class constructor
         */
        ConstraintAutodiffApriltag(
                const SensorBasePtr& _sensor_ptr,
                const FrameBasePtr& _frame_ptr,
                const LandmarkApriltagPtr& _landmark_other_ptr,
                const FeatureApriltagPtr& _feature_ptr,
                bool _apply_loss_function,
                ConstraintStatus _status);

        /** \brief Class Destructor
         */
        virtual ~ConstraintAutodiffApriltag();
 
        /** brief : compute the residual from the state blocks being iterated by the solver.
         **/
        template<typename T>
        bool operator ()( const T* const _p_camera, const T* const _o_camera, const T* const _p_keyframe, const T* const _o_keyframe, const T* const _p_landmark, const T* const _o_landmark, T* _residuals) const;

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

ConstraintAutodiffApriltag::ConstraintAutodiffApriltag(
        const SensorBasePtr& _sensor_ptr,
        const FrameBasePtr& _frame_ptr,
        const LandmarkApriltagPtr& _landmark_other_ptr,
        const FeatureApriltagPtr& _feature_ptr,
        bool _apply_loss_function,
        ConstraintStatus _status) :
            ConstraintAutodiff("AUTODIFF APRILTAG",
                               nullptr,
                               nullptr,
                               nullptr,
                               _landmark_other_ptr,
                               nullptr,
                               false,
                               CTR_ACTIVE,
                               _sensor_ptr->getPPtr(),         _sensor_ptr->getOPtr(),
                               _frame_ptr->getPPtr(),          _frame_ptr->getOPtr(),
                               _landmark_other_ptr->getPPtr(), _landmark_other_ptr->getOPtr()
                               )
{


}

/** \brief Class Destructor
 */
ConstraintAutodiffApriltag::~ConstraintAutodiffApriltag()
{
    //
}

template<typename T> bool ConstraintAutodiffApriltag::operator ()( const T* const _p_camera, const T* const _o_camera, const T* const _p_keyframe, const T* const _o_keyframe, const T* const _p_landmark, const T* const _o_landmark, T* _residuals) const
{
    //states
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
    Eigen::Transform<T, 3, Eigen::Affine> c_M_l_meas = p_measured.cast<T>() * q_measured.cast<T>();

    // Create transformation matrices to compose
    // robot wrt world
    Eigen::Transform<T, 3, Eigen::Affine> w_M_r = p_keyframe * q_keyframe;
    // camera wrt robot
    Eigen::Transform<T, 3, Eigen::Affine> r_M_c = p_camera * q_camera;
    // landmark wrt world
    Eigen::Transform<T, 3, Eigen::Affine> w_M_l = p_landmark * q_landmark;
    // landmark wrt camera, estimated
    Eigen::Transform<T, 3, Eigen::Affine> c_M_l_est = (w_M_r * r_M_c).inverse() * w_M_l;

    // expectation error, in camera frame
    // left-minus
    Eigen::Transform<T, 3, Eigen::Affine> c_M_err = c_M_l_meas.inverse() * c_M_l_est;
    // right-minus also works
//    Eigen::Transform<T, 3, Eigen::Affine> c_M_err = c_M_l_est.inverse() * c_M_l_meas;


    // error
    Eigen::Matrix<T, 6, 1> err;
    err.block(0,0,3,1) = c_M_err.translation();
    Eigen::Matrix<T, 3, 3> R_err(c_M_err.linear());
    err.block(3,0,3,1) = wolf::log_R(R_err);

    // debug stuff
//    int kf = getFeaturePtr()->getCapturePtr()->getFramePtr()->id();
//    int lmk = getLandmarkOtherPtr()->id();
//
//    print(kf, lmk, "w_M_c  : \n", (w_M_r*r_M_c).matrix());
//    print(kf, lmk, "w_M_l  : \n", w_M_l.matrix());
//    print(kf, lmk, "c_M_l_e: \n", c_M_l_est.matrix());
//    print(kf, lmk, "c_M_l_m: \n", c_M_l_meas.matrix());
//    print(kf, lmk, "error  : \n", err.transpose().eval());

    // residual
    Eigen::Map<Eigen::Matrix<T, 6, 1>> res(_residuals);

    res = getFeaturePtr()->getMeasurementSquareRootInformationUpper().cast<T>() * err;

    return true;
}

} // namespace wolf

#endif /* _CONSTRAINT_AUTODIFF_APRILTAG_H_ */

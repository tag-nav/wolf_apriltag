//--------LICENSE_START--------
//
// Copyright (C) 2020,2021,2022 Institut de Robòtica i Informàtica Industrial, CSIC-UPC.
// Authors: Joan Solà Ortega (jsola@iri.upc.edu)
// All rights reserved.
//
// This file is part of WOLF
// WOLF is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
//--------LICENSE_END--------
#ifndef _FACTOR_APRILTAG_PROJ_H_
#define _FACTOR_APRILTAG_PROJ_H_

//Wolf includes
#include "core/common/wolf.h"
#include "core/math/rotations.h"
#include "core/factor/factor_autodiff.h"
#include "core/sensor/sensor_base.h"
#include <vision/math/pinhole_tools.h>
#include "apriltag/landmark/landmark_apriltag.h"
#include "apriltag/feature/feature_apriltag_proj.h"

namespace wolf
{

WOLF_PTR_TYPEDEFS(FactorApriltagProj);

class FactorApriltagProj : public FactorAutodiff<FactorApriltagProj, 8, 3, 4, 3, 4, 3, 4>
{
    public:

        /** \brief Class constructor
         */
        FactorApriltagProj(
                const SensorBasePtr& _sensor_ptr,
                const FrameBasePtr& _frame_ptr,
                const LandmarkApriltagPtr& _landmark_other_ptr,
                const FeatureApriltagProjPtr& _feature_ptr,
                const ProcessorBasePtr& _processor_ptr,
                bool _apply_loss_function,
                FactorStatus _status);

        /** \brief Class Destructor
         */
        ~FactorApriltagProj() override;
 
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

        Eigen::Vector8d residual() const;
        double cost() const;

        // print function only for double (not Jet)
        template<typename T, int Rows, int Cols>
        void print(int kf, int lmk, const std::string s, const Eigen::Matrix<T, Rows, Cols> _M) const
        {
            // jet prints nothing
        }
        template<int Rows, int Cols>
        void print(int kf, int lmk, const std::string s, const Eigen::Matrix<double, Rows, Cols> _M) const
        {
            // double prints stuff
            WOLF_TRACE("KF", kf, " L", lmk, "; ", s, _M);
        }
        
        template<typename D1>
        static Eigen::Matrix<typename D1::Scalar, 2, 1> pinholeProj(const Eigen::Matrix3d& K,
                                                             const Eigen::MatrixBase<D1>& p_c_l,
                                                             const Eigen::Quaternion<typename D1::Scalar>& q_c_l,
                                                             const Eigen::Vector3d& l_corn);
};

} // namespace wolf
 
// Include here all headers for this class
//#include <YOUR_HEADERS.h>

namespace wolf
{

FactorApriltagProj::FactorApriltagProj(
        const SensorBasePtr& _sensor_ptr,
        const FrameBasePtr& _frame_ptr,
        const LandmarkApriltagPtr& _landmark_other_ptr,
        const FeatureApriltagProjPtr& _feature_ptr,
        const ProcessorBasePtr& _processor_ptr,
        bool _apply_loss_function,
        FactorStatus _status) :
            FactorAutodiff("FactorApriltagProj",
                           TOP_LMK,
                           _feature_ptr,
                           nullptr,
                           nullptr,
                           nullptr,
                           _landmark_other_ptr,
                           _processor_ptr,
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
FactorApriltagProj::~FactorApriltagProj()
{
    //
}


template<typename D1>
Eigen::Matrix<typename D1::Scalar, 2, 1> FactorApriltagProj::pinholeProj(const Eigen::Matrix3d& K,
                                                                         const Eigen::MatrixBase<D1>& p_c_l,
                                                                         const Eigen::Quaternion<typename D1::Scalar>& q_c_l,
                                                                         const Eigen::Vector3d& l_corn)
{
    MatrixSizeCheck<3,1>::check(p_c_l);

    typedef typename D1::Scalar T;
    Eigen::Matrix<T, 3, 1> h =  K.cast<T>() * (p_c_l + q_c_l * l_corn.cast<T>());
    Eigen::Matrix<T, 2, 1> pix; pix << h(0)/h(2), h(1)/h(2);

    return pix;
}


template<typename T> 
bool FactorApriltagProj::operator ()(const T* const _p_camera, 
                                     const T* const _o_camera, 
                                     const T* const _p_keyframe, 
                                     const T* const _o_keyframe, 
                                     const T* const _p_landmark, 
                                     const T* const _o_landmark, 
                                     T* _residuals) const
{
    // Maps
    Eigen::Map<const Eigen::Matrix<T,3,1>> p_r_c(_p_camera);
    Eigen::Map<const Eigen::Quaternion<T>> q_r_c(_o_camera);
    Eigen::Map<const Eigen::Matrix<T,3,1>> p_w_r(_p_keyframe);
    Eigen::Map<const Eigen::Quaternion<T>> q_w_r(_o_keyframe);
    Eigen::Map<const Eigen::Matrix<T,3,1>> p_w_l(_p_landmark);
    Eigen::Map<const Eigen::Quaternion<T>> q_w_l(_o_landmark);
    Eigen::Map<Eigen::Matrix<T,8,1>> residuals(_residuals);

    // Expected relative camera-lmk transformation
    // Expected measurement
    Eigen::Quaternion<T> q_c_w = (q_w_r * q_r_c).conjugate();
    Eigen::Quaternion<T> q_c_l = q_c_w * q_w_l;
    Eigen::Matrix<T,3,1> p_c_l = q_c_w * (-(p_w_r + q_w_r * p_r_c) + p_w_l);

    //////////////////////////////////////
    // Expected corner projections

    //////////////////////////////////////
    // Camera matrix (put somewhere else)
    // get camera intrinsic parameters
    Eigen::Vector4d k = getSensor()->getIntrinsic()->getState(); //[cx cy fx fy]
    Eigen::Matrix3d K;
    K << k(2), 0,    k(0),
         0,    k(3), k(1),
         0,    0,    1;
    //////////////////////////////////////


    auto feata = std::dynamic_pointer_cast<FeatureApriltagProj>(getFeature());
    double tag_width = feata->getTagWidth();

    // Same order as the 2d corners (anti clockwise, looking at the tag).
    // Looking at the tag, the reference frame is
    // X = Right, Y = Down, Z = Inside the plane
    // >>>>>>> JV: COULD THIS BE PRECOMPUTED AT CONSTRUCTION TIME?
    double s = tag_width/2;
    Eigen::Vector3d l_corn1; l_corn1 << -s,  s, 0; // bottom left
    Eigen::Vector3d l_corn2; l_corn2 <<  s,  s, 0; // bottom right
    Eigen::Vector3d l_corn3; l_corn3 <<  s, -s, 0; // top right
    Eigen::Vector3d l_corn4; l_corn4 << -s, -s, 0; // top left
    

    Eigen::Matrix<T, 8, 1> corners_exp;
    corners_exp.segment(0,2) = pinholeProj(K, p_c_l, q_c_l, l_corn1);
    corners_exp.segment(2,2) = pinholeProj(K, p_c_l, q_c_l, l_corn2);
    corners_exp.segment(4,2) = pinholeProj(K, p_c_l, q_c_l, l_corn3);
    corners_exp.segment(6,2) = pinholeProj(K, p_c_l, q_c_l, l_corn4);

    residuals = getMeasurementSquareRootInformationUpper().cast<T>() * (corners_exp - getMeasurement().cast<T>());

    return true;
}


Eigen::Vector8d FactorApriltagProj::residual() const
{
    Eigen::Vector8d res;
    double * p_camera, * o_camera, * p_frame, * o_frame, * p_tag, * o_tag;
    p_camera = getCapture()->getSensorP()->getState().data();
    o_camera = getCapture()->getSensorO()->getState().data();
    p_frame  = getCapture()->getFrame()->getP()->getState().data();
    o_frame  = getCapture()->getFrame()->getO()->getState().data();
    p_tag    = getLandmarkOther()->getP()->getState().data();
    o_tag    = getLandmarkOther()->getO()->getState().data();

    operator() (p_camera, o_camera, p_frame, o_frame, p_tag, o_tag, res.data());

    return res;
}


double FactorApriltagProj::cost() const
{
    return residual().squaredNorm();
}

} // namespace wolf

#endif /* _CONSTRAINT_AUTODIFF_APRILTAG_H_ */

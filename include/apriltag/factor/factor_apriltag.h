// WOLF - Copyright (C) 2020,2021,2022,2023
// Institut de Robòtica i Informàtica Industrial, CSIC-UPC.
// Authors: Joan Solà Ortega (jsola@iri.upc.edu) and
// Joan Vallvé Navarro (jvallve@iri.upc.edu)
// All rights reserved.
//
// This file is part of WOLF: http://www.iri.upc.edu/wolf
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

#pragma once

// Wolf includes
#include "apriltag/common/apriltag.h"
#include "core/math/rotations.h"
#include "core/factor/factor_autodiff.h"
#include "core/sensor/sensor_base.h"
#include "apriltag/landmark/landmark_apriltag.h"
#include "apriltag/feature/feature_apriltag.h"

namespace wolf
{

WOLF_PTR_TYPEDEFS(FactorApriltag);

class FactorApriltag : public FactorAutodiff<FactorApriltag, 6, 3, 4, 3, 4, 3, 4>
{
  public:
    /** \brief Class constructor
     */
    FactorApriltag(const SensorBasePtr&       _sensor_ptr,
                   const FrameBasePtr&        _frame_ptr,
                   const LandmarkApriltagPtr& _landmark_other_ptr,
                   const FeatureApriltagPtr&  _feature_ptr,
                   const ProcessorBasePtr&    _processor_ptr,
                   bool                       _apply_loss_function,
                   FactorStatus               _status);

    /** \brief Class Destructor
     */
    ~FactorApriltag() override;

    /** brief : compute the residual from the state blocks being iterated by the solver.
     **/
    template <typename T>
    bool operator()(const T* const _p_camera,
                    const T* const _o_camera,
                    const T* const _p_keyframe,
                    const T* const _o_keyframe,
                    const T* const _p_landmark,
                    const T* const _o_landmark,
                    T*             _residuals) const;

    Eigen::Vector6d residual() const;
    double          cost() const;

    // print function only for double (not Jet)
    template <typename T, int Rows, int Cols>
    void print(int kf, int lmk, const std::string s, const Eigen::Matrix<T, Rows, Cols> _M) const
    {
        // jet prints nothing
    }
    template <int Rows, int Cols>
    void print(int kf, int lmk, const std::string s, const Eigen::Matrix<double, Rows, Cols> _M) const
    {
        // double prints stuff
        WOLF_TRACE("KF", kf, " L", lmk, "; ", s, _M);
    }
};

}  // namespace wolf

// Include here all headers for this class
// #include <YOUR_HEADERS.h>

namespace wolf
{

FactorApriltag::FactorApriltag(const SensorBasePtr&       _sensor_ptr,
                               const FrameBasePtr&        _frame_ptr,
                               const LandmarkApriltagPtr& _landmark_other_ptr,
                               const FeatureApriltagPtr&  _feature_ptr,
                               const ProcessorBasePtr&    _processor_ptr,
                               bool                       _apply_loss_function,
                               FactorStatus               _status)
    : FactorAutodiff("FactorApriltag",
                     TOP_LMK,
                     _feature_ptr,
                     nullptr,
                     nullptr,
                     nullptr,
                     _landmark_other_ptr,
                     _processor_ptr,
                     _apply_loss_function,
                     _status,
                     _sensor_ptr->getP(),
                     _sensor_ptr->getO(),
                     _frame_ptr->getP(),
                     _frame_ptr->getO(),
                     _landmark_other_ptr->getP(),
                     _landmark_other_ptr->getO())
{
}

/** \brief Class Destructor
 */
FactorApriltag::~FactorApriltag()
{
    //
}

template <typename T>
bool FactorApriltag::operator()(const T* const _p_camera,
                                const T* const _o_camera,
                                const T* const _p_keyframe,
                                const T* const _o_keyframe,
                                const T* const _p_landmark,
                                const T* const _o_landmark,
                                T*             _residuals) const
{
    // Maps
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> p_r_c(_p_camera);
    Eigen::Map<const Eigen::Quaternion<T>>   q_r_c(_o_camera);
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> p_w_r(_p_keyframe);
    Eigen::Map<const Eigen::Quaternion<T>>   q_w_r(_o_keyframe);
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> p_w_l(_p_landmark);
    Eigen::Map<const Eigen::Quaternion<T>>   q_w_l(_o_landmark);
    Eigen::Map<Eigen::Matrix<T, 6, 1>>       residuals(_residuals);

    // Expected measurement
    Eigen::Quaternion<T>   q_c_w = (q_w_r * q_r_c).conjugate();
    Eigen::Quaternion<T>   q_c_l = q_c_w * q_w_l;
    Eigen::Matrix<T, 3, 1> p_c_l = q_c_w * (-(p_w_r + q_w_r * p_r_c) + p_w_l);

    // Measurement
    Eigen::Vector3d      p_c_l_meas(getMeasurement().head<3>());
    Eigen::Quaterniond   q_c_l_meas(getMeasurement().data() + 3);
    Eigen::Quaternion<T> q_l_c_meas = q_c_l_meas.conjugate().cast<T>();
    // Eigen::Matrix<T,3,1> p_l_c_meas = -q_l_c_meas * p_c_l_meas.cast<T>();

    // Error
    Eigen::Matrix<T, 6, 1> err;
    err.head(3) = q_l_c_meas * (p_c_l_meas.cast<T>() - p_c_l);
    // err.tail(3) = wolf::log_q(q_l_c_meas * q_c_l);
    err.tail(3) = T(2) * (q_l_c_meas * q_c_l).vec();

    // Residual
    residuals = getMeasurementSquareRootInformationUpper().cast<T>() * err;

    return true;
}

Eigen::Vector6d FactorApriltag::residual() const
{
    Eigen::Vector6d res;
    double *        p_camera, *o_camera, *p_frame, *o_frame, *p_tag, *o_tag;
    p_camera = getCapture()->getSensorP()->getState().data();
    o_camera = getCapture()->getSensorO()->getState().data();
    p_frame  = getCapture()->getFrame()->getP()->getState().data();
    o_frame  = getCapture()->getFrame()->getO()->getState().data();
    p_tag    = getLandmarkOther()->getP()->getState().data();
    o_tag    = getLandmarkOther()->getO()->getState().data();

    operator()(p_camera, o_camera, p_frame, o_frame, p_tag, o_tag, res.data());

    return res;
}

double FactorApriltag::cost() const
{
    return residual().squaredNorm();
}

}  // namespace wolf
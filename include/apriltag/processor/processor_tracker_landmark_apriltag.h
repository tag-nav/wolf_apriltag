//--------LICENSE_START--------
//
// Copyright (C) 2020,2021,2022,2023 Institut de Robòtica i Informàtica Industrial, CSIC-UPC.
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
#ifndef _PROCESSOR_TRACKER_LANDMARK_APRILTAG_H_
#define _PROCESSOR_TRACKER_LANDMARK_APRILTAG_H_

// Wolf apriltag includes
#include "apriltag/feature/feature_apriltag.h"
#include "apriltag/feature/feature_apriltag_proj.h"
#include "apriltag/feature/feature_apriltag_pose.h"
#include "apriltag/landmark/landmark_apriltag.h"
#include "apriltag/factor/factor_apriltag_proj.h"

// IPPE (copy from https://github.com/tobycollins/IPPE)
#include "ippe.h"  /// use opencv solvePNP with square option instead

// Wolf vision
#include <vision/math/pinhole_tools.h>
#include <vision/capture/capture_image.h>
#include <vision/sensor/sensor_camera.h>

// Wolf core
#include <core/math/rotations.h>
// #include <core/state_block/state_quaternion.h>  /// REMOVE?
#include <core/factor/factor_relative_pose_3d_with_extrinsics.h>
#include <core/processor/processor_tracker_landmark.h>
#include <core/processor/motion_provider.h>

// apriltag detection Michigan library
#include <apriltag.h>
#include <common/homography.h>
#include <common/zarray.h>
#include <tag16h5.h>
#include <tag25h9.h>
#include <tag36h11.h>
#include <tagCircle21h7.h>
#include <tagCircle49h12.h>
#include <tagCustom48h12.h>
#include <tagCustom52h12.h>
#include <tagStandard41h12.h>
#include <tagStandard52h13.h>
#include <common/homography.h>
#include <common/zarray.h>

// opencv
// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/eigen.hpp>

// Eigen
#include <Eigen/Dense>

namespace wolf
{

WOLF_STRUCT_PTR_TYPEDEFS(ParamsProcessorTrackerLandmarkApriltag);

struct ParamsProcessorTrackerLandmarkApriltag : public ParamsProcessorTrackerLandmark, public ParamsMotionProvider
{
    //tag parameters
    std::string tag_family_;

    // tag sizes
    double tag_width_default_;
    std::map<int, double> tag_widths_;

    //detector parameters
    double quad_decimate_;
    double quad_sigma_;
    unsigned int nthreads_;
    bool debug_;
    bool refine_edges_;

    double std_pix_;
    
    bool use_proj_factor_;

    double min_time_span_;
    double max_time_span_;
    int nb_vote_for_every_first_;
    double april_min_decision_margin_;
    double ippe_min_ratio_;
    double ippe_max_rep_error_;

    ParamsProcessorTrackerLandmarkApriltag() = default;
    ParamsProcessorTrackerLandmarkApriltag(std::string _unique_name, const ParamsServer& _server):
        ParamsProcessorTrackerLandmark(_unique_name, _server),
        ParamsMotionProvider(_unique_name, _server)
    {
        tag_family_                 = _server.getParam<std::string>(prefix + _unique_name            + "/tag_family");
        tag_width_default_          = _server.getParam<double>(prefix + _unique_name                 + "/tag_width_default");
        tag_widths_                 = _server.getParam<std::map<int, double>>(prefix + _unique_name  + "/tag_widths");
        quad_decimate_              = _server.getParam<double>(prefix + _unique_name                 + "/quad_decimate");
        quad_sigma_                 = _server.getParam<double>(prefix + _unique_name                 + "/quad_sigma");
        nthreads_                   = _server.getParam<unsigned int>(prefix + _unique_name           + "/nthreads");
        debug_                      = _server.getParam<bool>(prefix + _unique_name                   + "/debug");
        refine_edges_               = _server.getParam<bool>(prefix + _unique_name                   + "/refine_edges");
        std_pix_                    = _server.getParam<double>(prefix + _unique_name                 + "/std_pix");
        use_proj_factor_            = _server.getParam<bool>(prefix + _unique_name                   + "/use_proj_factor");
        min_time_span_              = _server.getParam<double>(prefix + _unique_name                 + "/keyframe_vote/min_time_span");
        max_time_span_              = _server.getParam<double>(prefix + _unique_name                 + "/keyframe_vote/max_time_span");
        nb_vote_for_every_first_    = _server.getParam<int>(prefix + _unique_name                    + "/keyframe_vote/nb_vote_for_every_first");
        april_min_decision_margin_  = _server.getParam<double>(prefix + _unique_name                 + "/april_min_decision_margin");
        ippe_min_ratio_             = _server.getParam<double>(prefix + _unique_name                 + "/ippe_min_ratio");
        ippe_max_rep_error_         = _server.getParam<double>(prefix + _unique_name                 + "/ippe_max_rep_error");
    }
    std::string print() const override
    {
        return ParamsProcessorTrackerLandmark::print()                                  + "\n"
        + "tag_family_: "               + tag_family_                                   + "\n"
        + "tag_width_default_: "        + std::to_string(tag_width_default_)            + "\n"
        + "tag_widths_: "               + converter<std::string>::convert(tag_widths_)  + "\n"
        + "quad_decimate_: "            + std::to_string(quad_decimate_)                + "\n"
        + "quad_sigma_: "               + std::to_string(quad_sigma_)                   + "\n"
        + "nthreads_: "                 + std::to_string(nthreads_)                     + "\n"
        + "debug_: "                    + std::to_string(debug_)                        + "\n"
        + "refine_edges_: "             + std::to_string(refine_edges_)                 + "\n"
        + "std_pix_: "                  + std::to_string(std_pix_)                      + "\n"
        + "use_proj_factor_: "          + std::to_string(use_proj_factor_)              + "\n"
        + "min_time_span_: "            + std::to_string(min_time_span_)                + "\n"
        + "max_time_span_: "            + std::to_string(max_time_span_)                + "\n"
        + "nb_vote_for_every_first_: "  + std::to_string(nb_vote_for_every_first_)      + "\n"
        + "april_min_decision_margin_: "+ std::to_string(april_min_decision_margin_)    + "\n"
        + "ippe_min_ratio_: "           + std::to_string(ippe_min_ratio_)               + "\n"
        + "ippe_max_rep_error_: "       + std::to_string(ippe_max_rep_error_)           + "\n";
    }
};



WOLF_PTR_TYPEDEFS(ProcessorTrackerLandmarkApriltag);

class ProcessorTrackerLandmarkApriltag : public ProcessorTrackerLandmark, public MotionProvider
{
    public:


        /** \brief Class constructor
         */
        ProcessorTrackerLandmarkApriltag( ParamsProcessorTrackerLandmarkApriltagPtr _params_tracker_landmark_apriltag);
        WOLF_PROCESSOR_CREATE(ProcessorTrackerLandmarkApriltag, ParamsProcessorTrackerLandmarkApriltag);

        /** \brief Class Destructor
         */
        ~ProcessorTrackerLandmarkApriltag();

        void preProcess() override;
        void postProcess() override;

        /** \brief Find provided landmarks as features in the provided capture
         * \param _landmarks_in input list of landmarks to be found
         * \param _capture the capture in which the _landmarks_in should be searched
         * \param _features_out returned list of features  found in \b _capture corresponding to a landmark of _landmarks_in
         * \param _feature_landmark_correspondences returned map of landmark correspondences: _feature_landmark_correspondences[_feature_out_ptr] = landmark_in_ptr
         *
         * \return the number of landmarks found
         */
        unsigned int findLandmarks(const LandmarkBasePtrList& _landmarks_in,
                                   const CaptureBasePtr& _capture,
                                   FeatureBasePtrList& _features_out,
                                   LandmarkMatchMap& _feature_landmark_correspondences) override;

        /** \brief Vote for KeyFrame generation
         *
         * If a KeyFrame criterion is validated, this function returns true,
         * meaning that it wants to create a KeyFrame at the \b last Capture.
         *
         * WARNING! This function only votes! It does not create KeyFrames!
         */
        bool voteForKeyFrame() const override;

        /** \brief Detect new Features
         * \param _max_features maximum number of features detected (-1: unlimited. 0: none)
         * \param _capture The capture in which the new features should be detected.
         * \param _features_out The list of detected Features in _capture.
         * \return The number of detected Features.
         *
         * This function detects Features that do not correspond to known Features/Landmarks in the system.
         *
         * The function is called in ProcessorTrackerLandmark::processNew() to set the member new_features_last_,
         * the list of newly detected features of the capture last_ptr_.
         */
        unsigned int detectNewFeatures(const int& _max_new_features,
                                               const CaptureBasePtr& _capture,
                                               FeatureBasePtrList& _features_out) override;

        /** \brief Emplace one landmark
         *
         */
        LandmarkBasePtr emplaceLandmark(FeatureBasePtr _feature_ptr) override;

        /** \brief Emplace a new factor
         * \param _feature_ptr pointer to the Feature
         * \param _landmark_ptr LandmarkBase pointer to the Landmark.
         */
        FactorBasePtr emplaceFactor(FeatureBasePtr _feature_ptr, LandmarkBasePtr _landmark_ptr) override;

        void configure(SensorBasePtr _sensor) override;

        ////////////////////////////////////////////////
        // MotionProvider virtual methods implementation
        TimeStamp       getTimeStamp() const override;
        VectorComposite getState(const StateStructure& _structure = "") const override;
        VectorComposite getState(const TimeStamp& _ts, const StateStructure& _structure = "") const override;
        ////////////////////////////////////////////////


    public:
        double getTagWidth(int _id) const;
        std::string getTagFamily() const;
        FeatureBasePtrList getIncomingDetections() const;
        FeatureBasePtrList getIncomingDetections();
        FeatureBasePtrList getLastDetections() const;
        FeatureBasePtrList getLastDetections();
        void ippePoseEstimation(apriltag_detection_t *_det,
                                cv::Mat_<double> _K,
                                double _tag_width,
                                Eigen::Isometry3d &_M1,
                                double &_rep_error1,
                                Eigen::Isometry3d &_M2,
                                double &_rep_error2);
        Eigen::Matrix6d computeInformation(Eigen::Vector3d const &_t,
                                           Eigen::Matrix3d const &_R,
                                           Eigen::Matrix3d const &_K,
                                           double const &_tag_width,
                                           double const &_sig_q);
        void pinholeHomogeneous(Eigen::Matrix3d const & _K,
                                Eigen::Vector3d const & _t,
                                Eigen::Matrix3d const & _R,
                                Eigen::Vector3d const & _p,
                                Eigen::Vector3d &_h,
                                Eigen::Matrix3d &_J_h_T,
                                Eigen::Matrix3d &_J_h_R);
        void cornersToPose(const std::vector<cv::Point2d> &_img_pts,
                           const std::vector<double> &_k_vec,
                           Eigen::Isometry3d &_M);

    protected:
        void advanceDerived() override;
        void resetDerived() override;

    private:
        std::map<int, double> tag_widths_;  ///< each tag's width indexed by tag_id
        double tag_width_default_;          ///< all tags widths defaut to this
        cv::Mat grayscale_image_;
        apriltag_detector_t* detector_;
        apriltag_family_t* tag_family_;
        double std_pix_;                    ///< pixel error to be propagated to a camera to tag transformation covariance
        bool use_proj_factor_;
        double april_min_decision_margin_;
        double ippe_min_ratio_;
        double ippe_max_rep_error_;
        Matrix3d K_;
        cv::Mat_<double> cv_K_;
        int n_reset_;

    protected:
        FeatureBasePtrList detections_incoming_;   ///< detected tags in wolf form, incoming image
        FeatureBasePtrList detections_last_;       ///< detected tags in wolf form, last image


    // To be able to access them in unit tests
    protected:
        double          min_time_span_;
        double          max_time_span_;
        unsigned int    min_features_for_keyframe_;
        int             nb_vote_for_every_first_;
        int             nb_vote_;

    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

} // namespace wolf

#endif /* _PROCESSOR_TRACKER_LANDMARK_APRILTAG_H_ */

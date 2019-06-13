#ifndef _PROCESSOR_TRACKER_LANDMARK_APRILTAG_H_
#define _PROCESSOR_TRACKER_LANDMARK_APRILTAG_H_

//Wolf includes
#include "core/common/wolf.h"
#include "core/processor/processor_tracker_landmark.h"
#include "vision/sensor/sensor_camera.h"
#include "core/factor/factor_autodiff_distance_3D.h"

// Apriltag
#include <apriltag/apriltag.h>

// open cv
#include <opencv/cv.h>


namespace wolf
{

WOLF_STRUCT_PTR_TYPEDEFS(ProcessorParamsTrackerLandmarkApriltag);

struct ProcessorParamsTrackerLandmarkApriltag : public ProcessorParamsTrackerLandmark
{
    //tag parameters
    std::string tag_family_;

    // tag sizes
    Scalar tag_width_default_;
    std::map<int, Scalar> tag_widths_;

    //detector parameters
    Scalar quad_decimate_;
    Scalar quad_sigma_;
    unsigned int nthreads_;
    bool debug_;
    bool refine_edges_;

    Scalar std_xy_, std_z_, std_rpy_;
    Scalar std_pix_;
    Scalar min_time_vote_;
    Scalar max_time_vote_;
    int max_features_diff_;
    int nb_vote_for_every_first_;
    bool enough_info_necessary_;
    bool add_3D_cstr_;
    Scalar ippe_min_ratio_;
    Scalar ippe_max_rep_error_;

    bool reestimate_last_frame_;
};



WOLF_PTR_TYPEDEFS(ProcessorTrackerLandmarkApriltag);

class ProcessorTrackerLandmarkApriltag : public ProcessorTrackerLandmark
{
    public:


        /** \brief Class constructor
         */
        ProcessorTrackerLandmarkApriltag( ProcessorParamsTrackerLandmarkApriltagPtr _params_tracker_landmark_apriltag);

        /** \brief Class Destructor
         */
        virtual ~ProcessorTrackerLandmarkApriltag();

        void preProcess();
        void postProcess();

        /** \brief Find provided landmarks as features in the provided capture
         * \param _landmarks_in input list of landmarks to be found
         * \param _capture the capture in which the _landmarks_in should be searched
         * \param _features_out returned list of features  found in \b _capture corresponding to a landmark of _landmarks_in
         * \param _feature_landmark_correspondences returned map of landmark correspondences: _feature_landmark_correspondences[_feature_out_ptr] = landmark_in_ptr
         *
         * \return the number of landmarks found
         */
        virtual unsigned int findLandmarks(const LandmarkBasePtrList& _landmarks_in,
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
        virtual bool voteForKeyFrame();

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
        virtual unsigned int detectNewFeatures(const int& _max_new_features,
                                               const CaptureBasePtr& _capture,
                                               FeatureBasePtrList& _features_out) override;

        /** \brief Emplace one landmark
         *
         */
        virtual LandmarkBasePtr emplaceLandmark(FeatureBasePtr _feature_ptr);

        /** \brief Emplace a new factor
         * \param _feature_ptr pointer to the Feature
         * \param _landmark_ptr LandmarkBase pointer to the Landmark.
         */
        virtual FactorBasePtr emplaceFactor(FeatureBasePtr _feature_ptr, LandmarkBasePtr _landmark_ptr);

        virtual void configure(SensorBasePtr _sensor);

        void reestimateLastFrame();

        // for factory
        static ProcessorBasePtr create(const std::string& _unique_name, const ProcessorParamsBasePtr _params, const SensorBasePtr sensor_ptr = nullptr);

    public:
        Scalar getTagWidth(int _id) const;
        std::string getTagFamily() const;
        Eigen::Vector6s getVarVec();
        FeatureBasePtrList getIncomingDetections() const;
        FeatureBasePtrList getLastDetections() const;
        Eigen::Isometry3d opencvPoseEstimation(apriltag_detection_t *_det, cv::Mat_<Scalar>, double _tag_width);
        Eigen::Isometry3d umichPoseEstimation(apriltag_detection_t *_det, cv::Mat_<Scalar>, double _tag_width);
        void ippePoseEstimation(apriltag_detection_t *_det, cv::Mat_<Scalar>, double _tag_width,
                                    Eigen::Isometry3d &_M1,
                                    double &_rep_error1,
                                    Eigen::Isometry3d &_M2,
                                    double &_rep_error2);
        Eigen::Matrix6s computeInformation(Eigen::Vector3s const &_t, Eigen::Matrix3s const &_R, Eigen::Matrix3s const &_K, double const &_tag_width, double const &_sig_q);
        void pinholeHomogeneous(Eigen::Matrix3s const & _K, Eigen::Vector3s const & _t,
                                Eigen::Matrix3s const & _R, Eigen::Vector3s const & _p,
                                Eigen::Vector3s &_h, Eigen::Matrix3s &_J_h_T, Eigen::Matrix3s &_J_h_R);
        void cornersToPose(const std::vector<cv::Point2d> &_img_pts,
                           const std::vector<Scalar> &_k_vec,
                           Eigen::Isometry3ds &_M);

    protected:
        void advanceDerived();
        void resetDerived();

    private:
        std::map<int, Scalar> tag_widths_;  ///< each tag's width indexed by tag_id
        Scalar tag_width_default_;          ///< all tags widths defaut to this
        cv::Mat grayscale_image_;
        apriltag_detector_t detector_;
        apriltag_family_t tag_family_;
        Scalar std_xy_, std_z_, std_rpy_;   ///< dummy std values for covariance computation
        Scalar std_pix_;                    ///< pixel error to be propagated to a camera to tag transformation covariance
        Scalar ippe_min_ratio_;
        Scalar ippe_max_rep_error_;
//        Eigen::Isometry3ds c_M_ac_;           ///< aprilCamera-to-camera transform not used with solvePnP
//        double cx_, cy_, fx_, fy_;
        Matrix3s K_;
        cv::Mat_<Scalar> cv_K_;
        bool reestimate_last_frame_;
        int n_reset_;

    protected:
        FeatureBasePtrList detections_incoming_;   ///< detected tags in wolf form, incoming image
        FeatureBasePtrList detections_last_;       ///< detected tags in wolf form, last image


    // To be able to access them in unit tests
    protected:
        Scalar          min_time_vote_;
        Scalar          max_time_vote_;
        unsigned int    min_features_for_keyframe_;
        int             max_features_diff_;
        int             nb_vote_for_every_first_;
        bool            enough_info_necessary_;
        bool            add_3D_cstr_;
        int             nb_vote_;
};

} // namespace wolf

#endif /* _PROCESSOR_TRACKER_LANDMARK_APRILTAG_H_ */

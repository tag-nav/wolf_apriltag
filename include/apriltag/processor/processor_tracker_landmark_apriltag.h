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
    ProcessorParamsTrackerLandmarkApriltag() = default;
    ProcessorParamsTrackerLandmarkApriltag(std::string _unique_name, const paramsServer& _server):
        ProcessorParamsTrackerLandmark(_unique_name, _server)
        {
            tag_family_ = _server.getParam<std::string>(_unique_name + "/tag_family");
            tag_width_default_ = _server.getParam<Scalar>(_unique_name + "/tag_width_default");
            tag_widths_ = _server.getParam<std::map<int, Scalar>>(_unique_name + "/tag_widths");
            quad_decimate_ = _server.getParam<Scalar>(_unique_name + "/quad_decimate");
            quad_sigma_ = _server.getParam<Scalar>(_unique_name + "/quad_sigma");
            nthreads_ = _server.getParam<unsigned int>(_unique_name + "/nthreads");
            debug_ = _server.getParam<bool>(_unique_name + "/debug");
            refine_edges_ = _server.getParam<bool>(_unique_name + "/refine_edges");
            std_xy_ = _server.getParam<Scalar>(_unique_name + "/std_xy");
            std_z_ = _server.getParam<Scalar>(_unique_name + "/std_z");
            std_rpy_ = _server.getParam<Scalar>(_unique_name + "/std_rpy");
            std_pix_ = _server.getParam<Scalar>(_unique_name + "/std_pix");
            min_time_vote_ = _server.getParam<Scalar>(_unique_name + "/min_time_vote");
            max_time_vote_ = _server.getParam<Scalar>(_unique_name + "/max_time_vote");
            max_features_diff_ = _server.getParam<int>(_unique_name + "/max_features_diff");
            nb_vote_for_every_first_ = _server.getParam<int>(_unique_name + "/nb_vote_for_every_first");
            enough_info_necessary_ = _server.getParam<bool>(_unique_name + "/enough_info_necessary");
            add_3D_cstr_ = _server.getParam<bool>(_unique_name + "/add_3D_cstr");
            ippe_min_ratio_ = _server.getParam<Scalar>(_unique_name + "/ippe_min_ratio");
            ippe_max_rep_error_ = _server.getParam<Scalar>(_unique_name + "/ippe_max_rep_error");
            reestimate_last_frame_ = _server.getParam<bool>(_unique_name + "/reestimate_last_frame");
        }
    std::string print()
    {
        return "\n" + ProcessorParamsTrackerLandmark::print()
            + "tag_family_: " + tag_family_ + "\n"
            + "tag_width_default_: " + std::to_string(tag_width_default_) + "\n"
            + "tag_widths_: " + converter<std::string>::convert(tag_widths_) + "\n"
            + "quad_decimate_: " + std::to_string(quad_decimate_) + "\n"
            + "quad_sigma_: " + std::to_string(quad_sigma_) + "\n"
            + "nthreads_: " + std::to_string(nthreads_) + "\n"
            + "debug_: " + std::to_string(debug_) + "\n"
            + "refine_edges_: " + std::to_string(refine_edges_) + "\n"
            + "std_xy_: " + std::to_string(std_xy_) + "\n"
            + "std_z_: " + std::to_string(std_z_) + "\n"
            + "std_rpy_: " + std::to_string(std_rpy_) + "\n"
            + "std_pix_: " + std::to_string(std_pix_) + "\n"
            + "min_time_vote_: " + std::to_string(min_time_vote_) + "\n"
            + "max_time_vote_: " + std::to_string(max_time_vote_) + "\n"
            + "max_features_diff_: " + std::to_string(max_features_diff_) + "\n"
            + "nb_vote_for_every_first_: " + std::to_string(nb_vote_for_every_first_) + "\n"
            + "enough_info_necessary_: " + std::to_string(enough_info_necessary_) + "\n"
            + "add_3D_cstr_: " + std::to_string(add_3D_cstr_) + "\n"
            + "ippe_min_ratio_: " + std::to_string(ippe_min_ratio_) + "\n"
            + "ippe_max_rep_error_: " + std::to_string(ippe_max_rep_error_) + "\n"
            + "reestimate_last_frame_: " + std::to_string(reestimate_last_frame_) + "\n";
    }
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

        /** \brief Find provided landmarks in the incoming capture
         * \param _landmark_list_in input list of landmarks to be found in incoming
         * \param _feature_list_out returned list of incoming features corresponding to a landmark of _landmark_list_in
         * \param _feature_landmark_correspondences returned map of landmark correspondences: _feature_landmark_correspondences[_feature_out_ptr] = landmark_in_ptr
         * \return the number of landmarks found
         */
        virtual unsigned int findLandmarks(const LandmarkBasePtrList& _landmark_list_in, FeatureBasePtrList& _feature_list_out,
                                           LandmarkMatchMap& _feature_landmark_correspondences);

        /** \brief Vote for KeyFrame generation
         *
         * If a KeyFrame criterion is validated, this function returns true,
         * meaning that it wants to create a KeyFrame at the \b last Capture.
         *
         * WARNING! This function only votes! It does not create KeyFrames!
         */
        virtual bool voteForKeyFrame();

        /** \brief Detect new Features in last Capture
         * \param _max_features The maximum number of features to detect.
         * \return The number of detected Features.
         *
         * This function detects Features that do not correspond to known Landmarks in the system.
         *
         * The function sets the member new_features_list_, the list of newly detected features
         * in last_ptr_ to be used for landmark initialization.
         */
        virtual unsigned int detectNewFeatures(const int& _max_features, FeatureBasePtrList& _feature_list_out);

        /** \brief Create one landmark
         *
         * Implement in derived classes to build the type of landmark you need for this tracker.
         */
        virtual LandmarkBasePtr createLandmark(FeatureBasePtr _feature_ptr);

        /** \brief Create a new constraint
         * \param _feature_ptr pointer to the Feature to constrain
         * \param _landmark_ptr LandmarkBase pointer to the Landmark constrained.
         *
         * Implement this method in derived classes.
         */
        virtual FactorBasePtr createFactor(FeatureBasePtr _feature_ptr, LandmarkBasePtr _landmark_ptr);

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

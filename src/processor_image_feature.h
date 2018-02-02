#ifndef PROCESSOR_IMAGE_FEATURE_H
#define PROCESSOR_IMAGE_FEATURE_H

// Wolf includes
#include "sensor_camera.h"
#include "capture_image.h"
#include "feature_point_image.h"
#include "state_block.h"
#include "state_quaternion.h"
#include "processor_tracker_feature.h"
#include "constraint_epipolar.h"
#include "processor_image_params.h"

// Vision utils
#include <vision_utils.h>
#include <vision_utils/detectors.h>
#include <vision_utils/descriptors.h>
#include <vision_utils/matchers.h>
#include <vision_utils/algorithms.h>

// General includes
#include <cmath>
#include <complex>      // std::complex, std::norm

namespace wolf
{

WOLF_PTR_TYPEDEFS(ProcessorImageFeature);

//class
class ProcessorImageFeature : public ProcessorTrackerFeature
{
    protected:

		vision_utils::DetectorBasePtr det_ptr_;
		vision_utils::DescriptorBasePtr des_ptr_;
		vision_utils::MatcherBasePtr mat_ptr_;
		vision_utils::AlgorithmACTIVESEARCHPtr active_search_ptr_;  // Active Search

		ProcessorParamsImage params_;           // Struct with parameters of the processors
        cv::Mat image_last_, image_incoming_;   // Images of the "last" and "incoming" Captures

        struct
        {
                unsigned int width_; ///< width of the image
                unsigned int height_; ///< height of the image
        } image_;

        unsigned int complete_target_size_ = 1;
        unsigned int target_size_ = 1;

    public:

        // Lists to store values to debug
        std::list<cv::Rect> tracker_roi_;
        std::list<cv::Rect> tracker_roi_inflated_;
        std::list<cv::Rect> detector_roi_;
        std::list<cv::Point> tracker_target_;

        ProcessorImageFeature(ProcessorParamsImage _params);
        virtual ~ProcessorImageFeature();

        virtual void setup(SensorCameraPtr _camera_ptr);

    protected:

        /**
         * \brief Does cast of the images and renews the active grid.
         */
        void preProcess();

        /**
         * \brief Does the drawing of the features.
         *
         * Used for debugging
         */
        void postProcess();

        void advance()
        {
            ProcessorTrackerFeature::advance();
            image_last_ = image_incoming_;
        }

        void reset()
        {
            ProcessorTrackerFeature::reset();
            image_last_ = image_incoming_;
        }

        virtual unsigned int trackFeatures(const FeatureBaseList& _feature_list_in, FeatureBaseList& _feature_list_out,
                                           FeatureMatchMap& _feature_correspondences);

        /** \brief Correct the drift in incoming feature by re-comparing against the corresponding feature in origin.
         * \param _last_feature input feature in last capture tracked
         * \param _incoming_feature input/output feature in incoming capture to be corrected
         * \return false if the the process discards the correspondence with origin's feature
         */
        virtual bool correctFeatureDrift(const FeatureBasePtr _origin_feature, const FeatureBasePtr _last_feature,
                                         FeatureBasePtr _incoming_feature);

        /** \brief Vote for KeyFrame generation
         *
         * If a KeyFrame criterion is validated, this function returns true,
         * meaning that it wants to create a KeyFrame at the \b last Capture.
         *
         * WARNING! This function only votes! It does not create KeyFrames!
         */
        virtual bool voteForKeyFrame();

        /** \brief Detect new Features
         *
         * This is intended to create Features that are not among the Features already known in the Map.
         *
         * This function sets new_features_last_, the list of newly detected features.
         *
         * \return The number of detected Features.
         */
        virtual unsigned int detectNewFeatures(const unsigned int& _max_new_features);

        virtual ConstraintBasePtr createConstraint(FeatureBasePtr _feature_ptr, FeatureBasePtr _feature_other_ptr);

    private:

        /**
         * \brief Detects keypoints its descriptors in a specific roi of the image
         * \param _image input image in which the algorithm will search
         * \param _roi input roi used to define the area of search within the image
         * \param _new_keypoints output keypoints obtained in the function
         * \param new_descriptors output descriptors obtained in the function
         * \return the number of detected features
         */
        virtual unsigned int detect(cv::Mat _image, cv::Rect& _roi, std::vector<cv::KeyPoint>& _new_keypoints,
                                    cv::Mat& new_descriptors);

        /**
         * \brief Does the match between a target descriptor and (potentially) multiple candidate descriptors of a Feature.
         * \param _target_descriptor descriptor of the target
         * \param _candidate_descriptors descriptors of the candidates
         * \param _cv_matches output variable in which the best result will be stored (in the position [0])
         * \return normalized score of similarity (1 - exact match; 0 - complete mismatch)
         */
        virtual Scalar match(cv::Mat _target_descriptor, cv::Mat _candidate_descriptors,
                             std::vector<cv::DMatch>& _cv_matches);

        // These only to debug, will disappear one day
    public:
        virtual void drawFeatures(cv::Mat _image);
        virtual void drawTarget(cv::Mat _image, std::list<cv::Point> _target_list);
        virtual void drawRoi(cv::Mat _image, std::list<cv::Rect> _roi_list, cv::Scalar _color);
        virtual void resetVisualizationFlag(FeatureBaseList& _feature_list_last);

    public:
        static ProcessorBasePtr create(const std::string& _unique_name, const ProcessorParamsBasePtr _params,
                                       const SensorBasePtr sensor_ptr = nullptr);

};

inline bool ProcessorImageFeature::voteForKeyFrame()
{
    return (incoming_ptr_->getFeatureList().size() < params_.algorithm.min_features_for_keyframe);
}

} // namespace wolf

#endif // PROCESSOR_IMAGE_FEATURE_H

/**
 * \file processor_tracker_landmark.h
 *
 *  Created on: Apr 7, 2016
 *      \author: jvallve
 */

#ifndef PROCESSOR_TRACKER_LANDMARK_H_
#define PROCESSOR_TRACKER_LANDMARK_H_

//wolf includes
#include "base/processor/processor_tracker.h"
#include "base/capture/capture_base.h"
#include "base/landmark/landmark_match.h"

namespace wolf
{

WOLF_STRUCT_PTR_TYPEDEFS(ProcessorParamsTrackerLandmark);

struct ProcessorParamsTrackerLandmark : public ProcessorParamsTracker
{
    //
};

WOLF_PTR_TYPEDEFS(ProcessorTrackerLandmark);
    
/** \brief Landmark tracker processor
 *
 * This is an abstract class.
 *
 * This class implements the incremental landmark tracker.
 *
 * The incremental tracker contains three pointers to three Captures of type CaptureBase,
 * named \b origin, \b last and \b incoming:
 *   - \b origin: this points to a Capture where all Feature tracks start.
 *   - \b last: the last Capture tracked by the tracker.
 *     A sufficient subset of the Features in \b origin is still alive in \b last.
 *   - \b incoming: the capture being received. The tracker operates on this Capture,
 *     establishing correspondences between the features here and the features in \b origin.
 *     Each successful correspondence
 *     results in an extension of the track of the Feature up to the \b incoming Capture.
 *
 * This processor creates landmarks for new detected Features, and establishes factors Feature-Landmark.
 *
 * This tracker builds on top of the ProcessorTracker by implementing some of its pure virtual functions.
 * As a reminder, we sketch here the pipeline of the parent ProcessorTracker process() function.
 * We highlight the functions implemented here with a sign  '<--- IMPLEMENTED', and the ones to be implemented by derived classes with '<=== IMPLEMENT'
 *
 *   - On each incoming Capture,
 *     - processKnown() : Track known features in the \b incoming Capture           <--- IMPLEMENTED
 *     - Check if enough Features are still tracked, and vote for a new KeyFrame if this number is too low:
 *     - if voteForKeyFrame()                                                       <=== IMPLEMENT
 *       - processNew() : Populate the tracker with new Features                    <--- IMPLEMENTED
 *       - makeFrame(), setKey() : Make a KeyFrame with the \b last Capture         <--- IMPLEMENTED
 *       - establishFactors() : Establish factors of the new Features       <--- IMPLEMENTED
 *       - reset() : Reset the tracker with the \b last Capture as the new \b origin<--- IMPLEMENTED
 *     - else
 *       - advance() : Advance the tracker one Capture ahead                        <--- IMPLEMENTED
 *
 * The most important implemented methods are:
 *   - processKnown() : which calls the pure virtual, to be implemented in derived classes:
 *     - findLandmarks() : find Landmarks from the \b map in \b incoming            <=== IMPLEMENT
 *   - processNew() : which calls the pure virtuals:
 *     - detectNewFeatures() : detects new Features in \b last                      <=== IMPLEMENT
 *     - createLandmark() : creates a Landmark using a new Feature                  <=== IMPLEMENT
 *     - findLandmarks() : find the new Landmarks again in \b incoming              <=== IMPLEMENT
 *   - establishFactors() : which calls the pure virtual:
 *     - createFactor() : create a Feature-Landmark factor of the correct derived type <=== IMPLEMENT
 *
 * Should you need extra functionality for your derived types, you can overload these two methods,
 *
 *   -  preProcess()  { }
 *   -  postProcess() { }
 *
 * which are called at the beginning and at the end of process() respectively.
 * See the doc of these functions for more info.
 */
class ProcessorTrackerLandmark : public ProcessorTracker
{
    public:
        ProcessorTrackerLandmark(const std::string& _type,
                                 ProcessorParamsTrackerLandmarkPtr _params_tracker_landmark);
        virtual ~ProcessorTrackerLandmark();

    protected:

        ProcessorParamsTrackerLandmarkPtr params_tracker_landmark_;
        LandmarkBasePtrList new_landmarks_;        ///< List of new detected landmarks
        LandmarkMatchMap matches_landmark_from_incoming_;
        LandmarkMatchMap matches_landmark_from_last_;

        /** \brief Tracker function
         * \return The number of successful tracks.
         *
         * Find Features in \b incoming Capture corresponding to known landmarks in the \b Map.
         *
         * This is the tracker function to be implemented in derived classes.
         * It operates on the \b incoming capture pointed by incoming_ptr_.
         *
         * The function must populate the list of Features in the \b incoming Capture.
         * Features need to be of the correct type, derived from FeatureBase.
         */
        virtual unsigned int processKnown();

        /** \brief Find provided landmarks in the incoming capture
         * \param _landmarks_in input list of landmarks to be found in incoming
         * \param _features_incoming_out returned list of incoming features corresponding to a landmark of _landmarks_in
         * \param _feature_landmark_correspondences returned map of landmark correspondences: _feature_landmark_correspondences[_feature_out_ptr] = landmark_in_ptr
         * \return the number of landmarks found
         */
        virtual unsigned int findLandmarks(const LandmarkBasePtrList&  _landmarks_in,
                                           FeatureBasePtrList&         _features_incoming_out,
                                           LandmarkMatchMap&        _feature_landmark_correspondences) = 0;

        /** \brief Vote for KeyFrame generation
         *
         * If a KeyFrame criterion is validated, this function returns true,
         * meaning that it wants to create a KeyFrame at the \b last Capture.
         *
         * WARNING! This function only votes! It does not create KeyFrames!
         */
        virtual bool voteForKeyFrame() = 0;

        // We overload the advance and reset functions to update the lists of matches
        void advanceDerived();
        void resetDerived();

        /** \brief Process new Features
         *
         */
        unsigned int processNew(const unsigned int& _max_features);

        /** \brief Detect new Features in last Capture
         * \param _max_features The maximum number of features to detect.
         * \return The number of detected Features.
         *
         * This function detects Features that do not correspond to known Landmarks in the system.
         *
         * The function sets the member new_features_list_, the list of newly detected features
         * in last_ptr_ to be used for landmark initialization.
         */
        virtual unsigned int detectNewFeatures(const unsigned int& _max_features, FeatureBasePtrList& _features_incoming_out) = 0;

        /** \brief Creates a landmark for each of new_features_last_
         **/
        virtual void createNewLandmarks();

        /** \brief Create one landmark
         *
         * Implement in derived classes to build the type of landmark you need for this tracker.
         */
        virtual LandmarkBasePtr createLandmark(FeatureBasePtr _feature_ptr) = 0;

        /** \brief Create a new factor
         * \param _feature_ptr pointer to the Feature to constrain
         * \param _landmark_ptr LandmarkBase pointer to the Landmark constrained.
         *
         * Implement this method in derived classes.
         */
        virtual FactorBasePtr createFactor(FeatureBasePtr _feature_ptr, LandmarkBasePtr _landmark_ptr) = 0;

        /** \brief Establish factors between features in Capture \b last and landmarks
         */
        virtual void establishFactors();
};

}// namespace wolf

// IMPLEMENTATION
#include "base/landmark/landmark_base.h"

#endif /* PROCESSOR_TRACKER_LANDMARK_H_ */

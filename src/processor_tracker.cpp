/*
 * ProcessorTracker.cpp
 *
 *  Created on: Apr 7, 2016
 *      Author: jvallve
 */

#include "processor_tracker.h"

namespace wolf
{

ProcessorTracker::ProcessorTracker(ProcessorType _tp, const std::string& _type, const unsigned int _max_new_features, const Scalar& _time_tolerance) :
        ProcessorBase(_tp, _type, _time_tolerance), origin_ptr_(nullptr), last_ptr_(nullptr), incoming_ptr_(nullptr),
        max_new_features_(_max_new_features)
{
    //
}

ProcessorTracker::~ProcessorTracker()
{
    if (last_ptr_ != nullptr && last_ptr_->getFramePtr() == nullptr)
        last_ptr_->destruct();

    if (incoming_ptr_ != nullptr && incoming_ptr_->getFramePtr() == nullptr)
        incoming_ptr_->destruct();

    while (!new_features_last_.empty())
    {
        new_features_last_.front()->destruct();
        new_features_last_.pop_front();
    }
    while (!new_features_incoming_.empty())
    {
        new_features_incoming_.front()->destruct();
        new_features_incoming_.pop_front();
    }
}

void ProcessorTracker::process(CaptureBasePtr const _incoming_ptr)
{
    std::cout << "-----ProcessorTracker::process():" << std::endl;
    //std::cout << "\tlast features: " << (last_ptr_ == nullptr ? 0 : last_ptr_->getFeatureListPtr()->size()) << std::endl;
    //std::cout << "\tlast new features: " << new_features_last_.size() << std::endl;
    //std::cout << "\tincoming features: " << (incoming_ptr_ == nullptr ? 0 : incoming_ptr_->getFeatureListPtr()->size()) << std::endl;
    //std::cout << "\tincoming new features: " << new_features_incoming_.size() << std::endl;

    incoming_ptr_ = _incoming_ptr;

    preProcess();
    // FIRST TIME
    if (origin_ptr_ == nullptr && last_ptr_ == nullptr)
    {
        std::cout << "FIRST TIME" << std::endl;
        //std::cout << "Features in origin: " << 0 << "; in last: " << 0 << std::endl;

        // advance
        advance();

        // advance this
        last_ptr_ = incoming_ptr_;
        incoming_ptr_ = nullptr;

        // keyframe creation on last
        makeFrame(last_ptr_, KEY_FRAME);

        // Detect new Features, initialize Landmarks, create Constraints, ...
        processNew(max_new_features_);

        // Establish constraints from last
        establishConstraints();

        //std::cout << "Features in last: " << last_ptr_->getFeatureListPtr()->size() << std::endl;
    }
    // SECOND TIME or after KEY FRAME CALLBACK
    else if (origin_ptr_ == nullptr)
    {
        std::cout << "SECOND TIME or after KEY FRAME CALLBACK" << std::endl;
        //std::cout << "Features in origin: " << origin_ptr_->getFeatureListPtr()->size() << "; in last: " << last_ptr_->getFeatureListPtr()->size() << std::endl;

        // First we track the known Features
        processKnown();

        // Create a new non-key Frame in the Trajectory with the incoming Capture
        makeFrame(incoming_ptr_);

        // Reset the derived Tracker
        reset();

        // Reset this
        origin_ptr_ = last_ptr_;
        last_ptr_ = incoming_ptr_;
        incoming_ptr_ = nullptr; // This line is not really needed, but it makes things clearer.

        //std::cout << "Features in origin: " << origin_ptr_->getFeatureListPtr()->size() << "; in last: " << last_ptr_->getFeatureListPtr()->size() << std::endl;
    }
    // OTHER TIMES
    else
    {
        std::cout << "OTHER TIMES" << std::endl;
        //std::cout << "Features in origin: " << origin_ptr_->getFeatureListPtr()->size() << "; in last: " << last_ptr_->getFeatureListPtr()->size() << std::endl;

        // 1. First we track the known Features and create new constraints as needed

        processKnown();

        // 2. Then we see if we want and we are allowed to create a KeyFrame
        if (!(voteForKeyFrame() && permittedKeyFrame()))
        {
            // We did not create a KeyFrame:

            // advance the derived tracker
            advance();

            // Advance this
            last_ptr_->getFramePtr()->addCapture(incoming_ptr_); // Add incoming Capture to the tracker's Frame
            last_ptr_->destruct();
            incoming_ptr_->getFramePtr()->setTimeStamp(incoming_ptr_->getTimeStamp());
            last_ptr_ = incoming_ptr_; // Incoming Capture takes the place of last Capture
            incoming_ptr_ = nullptr; // This line is not really needed, but it makes things clearer.
        }
        else
        {
            // 2.b. Detect new Features, initialize Landmarks, create Constraints, ...
            processNew(max_new_features_);

            // Create a new non-key Frame in the Trajectory with the incoming Capture
            makeFrame(incoming_ptr_);

            // Make the last Capture's Frame a KeyFrame so that it gets into the solver
            setKeyFrame(last_ptr_);

            // Establish constraints between last and origin
            establishConstraints();

            // Reset the derived Tracker
            reset();

            // Reset this
            origin_ptr_ = last_ptr_;
            last_ptr_ = incoming_ptr_;
            incoming_ptr_ = nullptr; // This line is not really needed, but it makes things clearer.
        }
        //std::cout << "Features in origin: " << origin_ptr_->getFeatureListPtr()->size() << "; in last: " << last_ptr_->getFeatureListPtr()->size() << std::endl;

    }
    postProcess();

    //std::cout << "-----End of process():" << std::endl;
    //std::cout << "\tlast features: " << (last_ptr_ == nullptr ? 0 : last_ptr_->getFeatureListPtr()->size()) << std::endl;
    //std::cout << "\tlast new features: " << new_features_last_.size() << std::endl;
    //std::cout << "\tincoming features: " << (incoming_ptr_ == nullptr ? 0 : incoming_ptr_->getFeatureListPtr()->size()) << std::endl;
    //std::cout << "\tincoming new features: " << new_features_incoming_.size() << std::endl;
}

bool ProcessorTracker::keyFrameCallback(FrameBasePtr _keyframe_ptr, const Scalar& _time_tol)
{
    assert((last_ptr_ == nullptr || last_ptr_->getFramePtr() != nullptr) && "ProcessorTracker::keyFrameCallback: last_ptr_ must have a frame allways");
    Scalar time_tol = std::max(time_tolerance_, _time_tol);

    // Nothing to do if:
    //   - there is no last
    //   - last frame is already a key frame
    //   - last frame is too far in time from keyframe
    if (last_ptr_ == nullptr || last_ptr_->getFramePtr()->isKey() || std::abs(last_ptr_->getTimeStamp() - _keyframe_ptr->getTimeStamp()) > time_tol)
        return false;

    //std::cout << "ProcessorTracker::keyFrameCallback in sensor " << getSensorPtr()->id() << std::endl;

    // Capture last_ is added to the new keyframe
    FrameBasePtr last_old_frame = last_ptr_->getFramePtr();
    last_old_frame->unlinkCapture(last_ptr_);
    last_old_frame->destruct();
    _keyframe_ptr->addCapture(last_ptr_);

    // Detect new Features, initialize Landmarks, create Constraints, ...
    processNew(max_new_features_);

    // Establish constraints between last and origin
    establishConstraints();

    // Set ready to go to 2nd case in process()
    origin_ptr_ = nullptr;

    return true;
}

void ProcessorTracker::setKeyFrame(CaptureBasePtr _capture_ptr)
{
    assert(_capture_ptr != nullptr && _capture_ptr->getFramePtr() != nullptr && "ProcessorTracker::setKeyFrame: null capture or capture without frame");
    if (!_capture_ptr->getFramePtr()->isKey())
    {
        // Set key
        _capture_ptr->getFramePtr()->setKey();
        // Set state to the keyframe
        _capture_ptr->getFramePtr()->setState(getProblem()->getStateAtTimeStamp(_capture_ptr->getTimeStamp()));
        // Call the new keyframe callback in order to let the other processors to establish their constraints
        getProblem()->keyFrameCallback(_capture_ptr->getFramePtr(), (ProcessorBasePtr)((this)), time_tolerance_);
    }
}


} // namespace wolf


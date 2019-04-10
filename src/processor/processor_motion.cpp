#include "base/processor/processor_motion.h"
namespace wolf
{

ProcessorMotion::ProcessorMotion(const std::string& _type,
                                 SizeEigen _state_size,
                                 SizeEigen _delta_size,
                                 SizeEigen _delta_cov_size,
                                 SizeEigen _data_size,
                                 SizeEigen _calib_size,
                                 ProcessorParamsMotionPtr _params_motion) :
        ProcessorBase(_type, _params_motion),
        params_motion_(_params_motion),
        processing_step_(RUNNING_WITHOUT_PACK),
        x_size_(_state_size),
        data_size_(_data_size),
        delta_size_(_delta_size),
        delta_cov_size_(_delta_cov_size),
        calib_size_(_calib_size),
        origin_ptr_(),
        last_ptr_(),
        incoming_ptr_(),
        dt_(0.0), x_(_state_size),
        data_(_data_size),
        delta_(_delta_size),
        delta_cov_(_delta_cov_size, _delta_cov_size),
        delta_integrated_(_delta_size),
        delta_integrated_cov_(_delta_cov_size, _delta_cov_size),
        calib_preint_(_calib_size),
        jacobian_delta_preint_(delta_cov_size_, delta_cov_size_),
        jacobian_delta_(delta_cov_size_, delta_cov_size_),
        jacobian_calib_(delta_size_, calib_size_)
{
    //
}

ProcessorMotion::~ProcessorMotion()
{
//    std::cout << "destructed     -p-Mot" << id() << std::endl;
}

void ProcessorMotion::splitBuffer(const wolf::CaptureMotionPtr& _capture_source,
                                  TimeStamp _ts_split,
                                  const FrameBasePtr& _keyframe_target,
                                  const wolf::CaptureMotionPtr& _capture_target)
{
    // split the buffer
    // and give the part of the buffer before the new keyframe to the capture for the KF callback
    _capture_source->getBuffer().split(_ts_split, _capture_target->getBuffer());

    // interpolate individual delta which has been cut by the split timestamp
    if (!_capture_source->getBuffer().get().empty()
            && _capture_target->getBuffer().get().back().ts_ != _ts_split)
    {
        // interpolate Motion at the new time stamp
        Motion motion_interpolated = interpolate(_capture_target->getBuffer().get().back(),  // last Motion of old buffer
                                                 _capture_source->getBuffer().get().front(), // first motion of new buffer
                                                 _ts_split,
                                                 _capture_source->getBuffer().get().front());

        // add to old buffer
        _capture_target->getBuffer().get().push_back(motion_interpolated);
    }

    // Update the existing capture
    _capture_source->setOriginFramePtr(_keyframe_target);

//    // Get optimized calibration params from 'origin' keyframe
//    VectorXs calib_preint_optim = _capture_source->getOriginFrame()->getCaptureOf(getSensor())->getCalibration();
//
//    // Write the calib params into the capture before re-integration
//    _capture_source->setCalibrationPreint(calib_preint_optim);

    // re-integrate target capture's buffer -- note: the result of re-integration is stored in the same buffer!
    reintegrateBuffer(_capture_target);

    // re-integrate source capture's buffer -- note: the result of re-integration is stored in the same buffer!
    reintegrateBuffer(_capture_source);
}

void ProcessorMotion::process(CaptureBasePtr _incoming_ptr)
{
    if (_incoming_ptr == nullptr)
    {
        WOLF_ERROR("Received capture is nullptr.");
        return;
    }

    incoming_ptr_ = std::static_pointer_cast<CaptureMotion>(_incoming_ptr);

    preProcess(); // Derived class operations

    PackKeyFramePtr pack = computeProcessingStep();
    if (pack)
        kf_pack_buffer_.removeUpTo( pack->key_frame->getTimeStamp() );

    switch(processing_step_)
    {

        case RUNNING_WITHOUT_PACK :
        case RUNNING_WITH_PACK_ON_ORIGIN :
            break;

        case RUNNING_WITH_PACK_BEFORE_ORIGIN :
        {
            // extract pack elements
            FrameBasePtr keyframe_from_callback = pack->key_frame;
            TimeStamp ts_from_callback = keyframe_from_callback->getTimeStamp();

            // find the capture whose buffer is affected by the new keyframe
            auto existing_capture = findCaptureContainingTimeStamp(ts_from_callback); // k

            // Find the frame acting as the capture's origin
            auto keyframe_origin = existing_capture->getOriginFrame(); // i

            auto capture_origin = std::static_pointer_cast<CaptureMotion>(keyframe_origin->getCaptureOf(getSensor()));

            // emplace a new motion capture to the new keyframe
            auto capture_for_keyframe_callback = emplaceCapture(keyframe_from_callback, // j
                                                                getSensor(),
                                                                ts_from_callback,
                                                                Eigen::VectorXs::Zero(data_size_),
                                                                capture_origin->getDataCovariance(),
                                                                capture_origin->getCalibration(),
                                                                capture_origin->getCalibration(),
                                                                keyframe_origin);

            // split the buffer
            // and give the part of the buffer before the new keyframe to the capture for the KF callback
            splitBuffer(existing_capture, ts_from_callback, keyframe_from_callback, capture_for_keyframe_callback);

            // create motion feature and add it to the capture
            auto new_feature = emplaceFeature(capture_for_keyframe_callback);

            // create motion factor and add it to the feature, and constrain to the other capture (origin)
            emplaceFactor(new_feature, keyframe_origin->getCaptureOf(getSensor()) );

            // modify existing feature and factor (if they exist in the existing capture)
            if (!existing_capture->getFeatureList().empty())
            {
                auto existing_feature = existing_capture->getFeatureList().back(); // there is only one feature!

                // Modify existing feature --------
                existing_feature->setMeasurement          (existing_capture->getBuffer().get().back().delta_integr_);
                existing_feature->setMeasurementCovariance(existing_capture->getBuffer().get().back().delta_integr_cov_);

                // Modify existing factor --------
                // Instead of modifying, we remove one ctr, and create a new one.
                auto ctr_to_remove  = existing_feature->getFactorList().back(); // there is only one factor!
                auto new_ctr        = emplaceFactor(existing_feature, capture_for_keyframe_callback);
                ctr_to_remove       ->remove();  // remove old factor now (otherwise c->remove() gets propagated to f, C, F, etc.)
            }
            break;
        }

        case RUNNING_WITH_PACK_AFTER_ORIGIN :
        {
            // extract pack elements
            FrameBasePtr keyframe_from_callback = pack->key_frame;
            TimeStamp    ts_from_callback       = keyframe_from_callback->getTimeStamp();

            // Find the frame acting as the capture's origin
            auto keyframe_origin = last_ptr_->getOriginFrame();

            // Get calibration params for preintegration from origin, since it has chances to have optimized values
            VectorXs calib_preint = origin_ptr_->getCalibration();

            // emplace a new motion capture to the new keyframe
            auto capture_for_keyframe_callback = emplaceCapture(keyframe_from_callback,
                                                                getSensor(),
                                                                ts_from_callback,
                                                                Eigen::VectorXs::Zero(data_size_),
                                                                origin_ptr_->getDataCovariance(),
                                                                calib_preint,
                                                                calib_preint,
                                                                keyframe_origin);

            // split the buffer
            // and give the part of the buffer before the new keyframe to the capture for the KF callback
            splitBuffer(last_ptr_, ts_from_callback, keyframe_from_callback, capture_for_keyframe_callback);

            // create motion feature and add it to the capture
            auto feature_for_keyframe_callback = emplaceFeature(capture_for_keyframe_callback);

            // create motion factor and add it to the feature, and constrain to the other capture (origin)
            emplaceFactor(feature_for_keyframe_callback, keyframe_origin->getCaptureOf(getSensor()) );

            // reset processor origin
            origin_ptr_ = capture_for_keyframe_callback;

            break;
        }

        default :
            break;
    }

    ////////////////////////////////////////////////////
    // NOW on with the received data

    // integrate data
    integrateOneStep();

    // Update state and time stamps
    last_ptr_->setTimeStamp(getCurrentTimeStamp());
    last_ptr_->getFrame()->setTimeStamp(getCurrentTimeStamp());
    last_ptr_->getFrame()->setState(getCurrentState());

    if (voteForKeyFrame() && permittedKeyFrame())
    {
        // Set the frame of last_ptr as key
        auto key_frame_ptr = last_ptr_->getFrame();
        key_frame_ptr->setKey();

        // create motion feature and add it to the key_capture
        auto key_feature_ptr = emplaceFeature(last_ptr_);

        // create motion factor and link it to parent feature and other frame (which is origin's frame)
        auto ctr_ptr = emplaceFactor(key_feature_ptr, origin_ptr_);

        // create a new frame
        auto new_frame_ptr = getProblem()->emplaceFrame(NON_KEY_FRAME,
                                                        getCurrentState(),
                                                        getCurrentTimeStamp());
        // create a new capture
        auto new_capture_ptr = emplaceCapture(new_frame_ptr,
                                              getSensor(),
                                              key_frame_ptr->getTimeStamp(),
                                              Eigen::VectorXs::Zero(data_size_),
                                              Eigen::MatrixXs::Zero(data_size_, data_size_),
                                              last_ptr_->getCalibration(),
                                              last_ptr_->getCalibration(),
                                              key_frame_ptr);
        // reset the new buffer
        new_capture_ptr->getBuffer().get().push_back( motionZero(key_frame_ptr->getTimeStamp()) ) ;

        // reset integrals
        delta_                  = deltaZero();
        delta_cov_              . setZero();
        delta_integrated_       = deltaZero();
        delta_integrated_cov_   . setZero();
        jacobian_calib_         . setZero();

        // reset derived things
        resetDerived();

        // Update pointers
        origin_ptr_     = last_ptr_;
        last_ptr_       = new_capture_ptr;

        // callback to other processors
        getProblem()->keyFrameCallback(key_frame_ptr, shared_from_this(), params_motion_->time_tolerance);
    }

    resetDerived();

    // clear incoming just in case
    incoming_ptr_ = nullptr; // This line is not really needed, but it makes things clearer.

    postProcess();
}

bool ProcessorMotion::getState(const TimeStamp& _ts, Eigen::VectorXs& _x)
{
    CaptureMotionPtr capture_motion;
    if (origin_ptr_ && _ts >= origin_ptr_->getTimeStamp())
        // timestamp found in the current processor buffer
        capture_motion = last_ptr_;
    else
        // We need to search in previous keyframes for the capture containing a motion buffer with the queried time stamp
        capture_motion = findCaptureContainingTimeStamp(_ts);

    if (capture_motion)  // We found a CaptureMotion whose buffer contains the time stamp
    {
        // Get origin state and calibration
        VectorXs state_0          = capture_motion->getOriginFrame()->getState();
        CaptureBasePtr cap_orig   = capture_motion->getOriginFrame()->getCaptureOf(getSensor());
        VectorXs calib            = cap_orig->getCalibration();

        // Get delta and correct it with new calibration params
        VectorXs calib_preint     = capture_motion->getBuffer().getCalibrationPreint();
        Motion   motion           = capture_motion->getBuffer().getMotion(_ts);
        
        VectorXs delta_step       = motion.jacobian_calib_ * (calib - calib_preint);
        VectorXs delta            = capture_motion->correctDelta( motion.delta_integr_, delta_step);

        // Compose on top of origin state using the buffered time stamp, not the query time stamp
        Scalar dt = motion.ts_ - capture_motion->getBuffer().get().front().ts_;
        statePlusDelta(state_0, delta, dt, _x);
    }
    else
    {
        // We could not find any CaptureMotion for the time stamp requested
        WOLF_ERROR("Could not find any Capture for the time stamp requested. ");
        WOLF_TRACE("Did you forget to call Problem::setPrior() in your application?")
        return false;
    }
    return true;
}

FrameBasePtr ProcessorMotion::setOrigin(const Eigen::VectorXs& _x_origin, const TimeStamp& _ts_origin)
{
    FrameBasePtr key_frame_ptr = getProblem()->emplaceFrame(KEY_FRAME, _x_origin, _ts_origin);
    setOrigin(key_frame_ptr);

    return key_frame_ptr;
}

void ProcessorMotion::setOrigin(FrameBasePtr _origin_frame)
{
    assert(_origin_frame && "ProcessorMotion::setOrigin: Provided frame pointer is nullptr.");
    assert(_origin_frame->getTrajectory() != nullptr
            && "ProcessorMotion::setOrigin: origin frame must be in the trajectory.");
    assert(_origin_frame->isKey() && "ProcessorMotion::setOrigin: origin frame must be KEY FRAME.");

    // -------- ORIGIN ---------
    // emplace (empty) origin Capture
    origin_ptr_ = emplaceCapture(_origin_frame,
                                 getSensor(),
                                 _origin_frame->getTimeStamp(),
                                 Eigen::VectorXs::Zero(data_size_),
                                 Eigen::MatrixXs::Zero(data_size_, data_size_),
                                 getSensor()->getCalibration(),
                                 getSensor()->getCalibration(),
                                 nullptr);

    // ---------- LAST ----------
    // Make non-key-frame for last Capture
    auto new_frame_ptr = getProblem()->emplaceFrame(NON_KEY_FRAME,
                                                    _origin_frame->getState(),
                                                    _origin_frame->getTimeStamp());
    // emplace (emtpy) last Capture
    last_ptr_ = emplaceCapture(new_frame_ptr,
                               getSensor(),
                               _origin_frame->getTimeStamp(),
                               Eigen::VectorXs::Zero(data_size_),
                               Eigen::MatrixXs::Zero(data_size_, data_size_),
                               getSensor()->getCalibration(),
                               getSensor()->getCalibration(),
                               _origin_frame);

    // clear and reset buffer
    getBuffer().get().push_back(motionZero(_origin_frame->getTimeStamp()));

    // Reset integrals
    delta_                  = deltaZero();
    delta_cov_              . setZero();
    delta_integrated_       = deltaZero();
    delta_integrated_cov_   . setZero();
    jacobian_calib_         . setZero();

    // Reset derived things
    resetDerived();
}

void ProcessorMotion::integrateOneStep()
{
    // Set dt
    dt_ = updateDt();
    assert(dt_ >= 0 && "Time interval _dt is negative!");

    // get vector of parameters to calibrate
    calib_preint_ = getBuffer().getCalibrationPreint();

    // get data and convert it to delta, and obtain also the delta covariance
    computeCurrentDelta(incoming_ptr_->getData(),
                        incoming_ptr_->getDataCovariance(),
                        calib_preint_,
                        dt_,
                        delta_,
                        delta_cov_,
                        jacobian_delta_calib_);

    // integrate the current delta to pre-integrated measurements, and get Jacobians
    deltaPlusDelta(getBuffer().get().back().delta_integr_,
                   delta_,
                   dt_,
                   delta_integrated_,
                   jacobian_delta_preint_,
                   jacobian_delta_);

    // integrate Jacobian wrt calib
    if ( hasCalibration() )
        jacobian_calib_.noalias()
            = jacobian_delta_preint_ * getBuffer().get().back().jacobian_calib_
            + jacobian_delta_ * jacobian_delta_calib_;

    // Integrate covariance
    delta_integrated_cov_.noalias()
            = jacobian_delta_preint_ * getBuffer().get().back().delta_integr_cov_ * jacobian_delta_preint_.transpose()
            + jacobian_delta_        * delta_cov_                                 * jacobian_delta_.transpose();

    // push all into buffer
    getBuffer().get().emplace_back(incoming_ptr_->getTimeStamp(),
                                   incoming_ptr_->getData(),
                                   incoming_ptr_->getDataCovariance(),
                                   delta_,
                                   delta_cov_,
                                   delta_integrated_,
                                   delta_integrated_cov_,
                                   jacobian_delta_,
                                   jacobian_delta_preint_,
                                   jacobian_calib_);
}

void ProcessorMotion::reintegrateBuffer(CaptureMotionPtr _capture_ptr)
{
    // start with empty motion
    _capture_ptr->getBuffer().get().push_front(motionZero(_capture_ptr->getOriginFrame()->getTimeStamp()));

    VectorXs calib = _capture_ptr->getCalibrationPreint();

    // Iterate all the buffer
    auto motion_it = _capture_ptr->getBuffer().get().begin();
    auto prev_motion_it = motion_it;
    motion_it++;
    while (motion_it != _capture_ptr->getBuffer().get().end())
    {
        // get dt
        const Scalar dt = motion_it->ts_ - prev_motion_it->ts_;

        // re-convert data to delta with the new calibration parameters
        computeCurrentDelta(motion_it->data_,
                            motion_it->data_cov_,
                            calib,
                            dt,
                            motion_it->delta_,
                            motion_it->delta_cov_,
                            jacobian_delta_calib_);

        // integrate delta into delta_integr, and rewrite the buffer
        deltaPlusDelta(prev_motion_it->delta_integr_,
                       motion_it->delta_,
                       dt,
                       motion_it->delta_integr_,
                       motion_it->jacobian_delta_integr_,
                       motion_it->jacobian_delta_);

        // integrate Jacobian wrt calib
        if ( hasCalibration() )
            motion_it->jacobian_calib_ = motion_it->jacobian_delta_integr_ * prev_motion_it->jacobian_calib_ + motion_it->jacobian_delta_ * jacobian_delta_calib_;

        // Integrate covariance
        motion_it->delta_integr_cov_ = motion_it->jacobian_delta_integr_ * prev_motion_it->delta_integr_cov_ * motion_it->jacobian_delta_integr_.transpose()
                                     + motion_it->jacobian_delta_        * motion_it->delta_cov_             * motion_it->jacobian_delta_.transpose();

        // advance in buffer
        motion_it++;
        prev_motion_it++;
    }
}

Motion ProcessorMotion::interpolate(const Motion& _ref, Motion& _second, TimeStamp& _ts)
{
    // Check time bounds
    assert(_ref.ts_ <= _second.ts_ && "Interpolation bounds not causal.");
    assert(_ts >= _ref.ts_    && "Interpolation time is before the _ref    motion.");
    assert(_ts <= _second.ts_ && "Interpolation time is after  the _second motion.");

    // Fraction of the time interval
    Scalar tau    = (_ts - _ref.ts_) / (_second.ts_ - _ref.ts_);

    if (tau < 0.5)
    {
        // _ts is closest to _ref
        Motion interpolated                 ( _ref );
        interpolated.ts_                    = _ts;
        interpolated.data_                  . setZero();
        interpolated.data_cov_              . setZero();
        interpolated.delta_                 = deltaZero();
        interpolated.delta_cov_             . setZero();
        interpolated.jacobian_delta_integr_ . setIdentity();
        interpolated.jacobian_delta_        . setZero();

        return interpolated;
    }
    else
    {
        // _ts is closest to _second
        Motion interpolated                 ( _second );
        interpolated.ts_                    = _ts;
        _second.data_                       . setZero();
        _second.data_cov_                   . setZero();
        _second.delta_                      = deltaZero();
        _second.delta_cov_                  . setZero();
        _second.jacobian_delta_integr_      . setIdentity();
        _second.jacobian_delta_             . setZero();

        return interpolated;
    }

}

Motion ProcessorMotion::interpolate(const Motion& _ref1, const Motion& _ref2, const TimeStamp& _ts, Motion& _second)
{
    // Check time bounds
    assert(_ref1.ts_ <= _ref2.ts_ && "Interpolation bounds not causal.");
    assert(_ts >= _ref1.ts_       && "Interpolation time is before the _ref1 motion.");
    assert(_ts <= _ref2.ts_       && "Interpolation time is after  the _ref2 motion.");

    // Fraction of the time interval
    Scalar tau    = (_ts - _ref1.ts_) / (_ref2.ts_ - _ref1.ts_);




    Motion interpolated(_ref1);  
    interpolated.ts_        = _ts;
    interpolated.data_      = (1-tau)*_ref1.data_ + tau*_ref2.data_;
    interpolated.data_cov_  = (1-tau)*_ref1.data_cov_ + tau*_ref2.data_cov_;  // bof
    computeCurrentDelta(interpolated.data_,
                        interpolated.data_cov_,
                        calib_preint_,
                        _ts.get() - _ref1.ts_.get(),
                        interpolated.delta_,
                        interpolated.delta_cov_,
                        interpolated.jacobian_calib_);
    deltaPlusDelta(_ref1.delta_integr_,
                   interpolated.delta_,
                   _ts.get() - _ref1.ts_.get(),
                   interpolated.delta_integr_,
                   interpolated.jacobian_delta_integr_,
                   interpolated.jacobian_delta_);

    _second.ts_       = _ref2.ts_;
    _second.data_     = tau*_ref1.data_ + (1-tau)*_ref2.data_;
    _second.data_cov_ = tau*_ref1.data_cov_ + (1-tau)*_ref2.data_cov_;  // bof
    computeCurrentDelta(_second.data_,
                        _second.data_cov_,
                        calib_preint_,
                        _ref2.ts_.get() - _ts.get(),
                        _second.delta_,
                        _second.delta_cov_,
                        _second.jacobian_calib_);
    deltaPlusDelta(_second.delta_integr_,
                   _second.delta_,
                   _second.ts_.get() - _ref1.ts_.get(),
                   _second.delta_integr_,
                   _second.jacobian_delta_integr_,
                   _second.jacobian_delta_);

    return interpolated;




    // if (tau < 0.5)
    // {
    //     // _ts is closest to _ref1
    //     Motion interpolated                 ( _ref1 );
    //     // interpolated.ts_                    = _ref1.ts_;
    //     // interpolated.data_                  = _ref1.data_;
    //     // interpolated.data_cov_              = _ref1.data_cov_;
    //     interpolated.delta_                 = deltaZero();
    //     interpolated.delta_cov_             . setZero();
    //     // interpolated.delta_integr_          = _ref1.delta_integr_;
    //     // interpolated.delta_integr_cov_      = _ref1.delta_integr_cov_;
    //     interpolated.jacobian_delta_integr_ . setIdentity();
    //     interpolated.jacobian_delta_        . setZero();

    //     _second = _ref2;

    //     return interpolated;
    // }
    // else
    // {
    //     // _ts is closest to _ref2
    //     Motion interpolated                 ( _ref2 );

    //     _second                             = _ref2;
    //     // _second.data_                       = _ref2.data_;
    //     // _second.data_cov_                   = _ref2.data_cov_;
    //     _second.delta_                      = deltaZero();
    //     _second.delta_cov_                  . setZero();
    //     // _second.delta_integr_               = _ref2.delta_integr_;
    //     // _second.delta_integr_cov_           = _ref2.delta_integr_cov_;
    //     _second.jacobian_delta_integr_      . setIdentity();
    //     _second.jacobian_delta_             . setZero();

    //     return interpolated;
    // }

}

CaptureMotionPtr ProcessorMotion::findCaptureContainingTimeStamp(const TimeStamp& _ts) const
{
    // We need to search in previous keyframes for the capture containing a motion buffer with the queried time stamp
    // Note: since the buffer goes from a KF in the past until the next KF, we need to:
    //  1. See that the KF contains a CaptureMotion
    //  2. See that the TS is smaller than the KF's TS
    //  3. See that the TS is bigger than the KF's first Motion in the CaptureMotion's buffer
    FrameBasePtr     frame          = nullptr;
    CaptureBasePtr   capture        = nullptr;
    CaptureMotionPtr capture_motion = nullptr;
    for (auto frame_rev_iter = getProblem()->getTrajectory()->getFrameList().rbegin();
            frame_rev_iter != getProblem()->getTrajectory()->getFrameList().rend();
            ++frame_rev_iter)
    {
        frame   = *frame_rev_iter;
        capture = frame->getCaptureOf(getSensor());
        if (capture != nullptr)
        {
            // Rule 1 satisfied! We found a Capture belonging to this processor's Sensor ==> it is a CaptureMotion
            capture_motion = std::static_pointer_cast<CaptureMotion>(capture);
            if (_ts >= capture_motion->getBuffer().get().front().ts_)
                // Found time stamp satisfying rule 3 above !! ==> break for loop
                break;
        }
    }
    return capture_motion;
}

CaptureMotionPtr ProcessorMotion::emplaceCapture(const FrameBasePtr& _frame_own,
                                                 const SensorBasePtr& _sensor,
                                                 const TimeStamp& _ts,
                                                 const VectorXs& _data,
                                                 const MatrixXs& _data_cov,
                                                 const VectorXs& _calib,
                                                 const VectorXs& _calib_preint,
                                                 const FrameBasePtr& _frame_origin)
{
    CaptureMotionPtr capture = createCapture(_ts,
                                             _sensor,
                                             _data,
                                             _data_cov,
                                             _frame_origin);

    capture->setCalibration(_calib);
    capture->setCalibrationPreint(_calib_preint);

    // add to wolf tree
    _frame_own->addCapture(capture);
    return capture;
}

FeatureBasePtr ProcessorMotion::emplaceFeature(CaptureMotionPtr _capture_motion)
{
    FeatureBasePtr feature = createFeature(_capture_motion);
    _capture_motion->addFeature(feature);
    return feature;
}

PackKeyFramePtr ProcessorMotion::computeProcessingStep()
{
    if (!getProblem()->priorIsSet())
    {
        WOLF_WARN ("||*||");
        WOLF_INFO (" ... It seems you missed something!");
        WOLF_ERROR("ProcessorMotion received data before being initialized.");
        WOLF_INFO ("Did you forget to issue a Problem::setPrior()?");
        throw std::runtime_error("ProcessorMotion received data before being initialized.");
    }

    PackKeyFramePtr pack = kf_pack_buffer_.selectFirstPackBefore(last_ptr_, params_motion_->time_tolerance);

    if (pack)
    {
        if (kf_pack_buffer_.checkTimeTolerance(pack->key_frame->getTimeStamp(), pack->time_tolerance, origin_ptr_->getTimeStamp(), params_motion_->time_tolerance))
        {
            WOLF_WARN("||*||");
            WOLF_INFO(" ... It seems you missed something!");
            WOLF_ERROR("Pack's KF and origin's KF have matching time stamps (i.e. below time tolerances)");
            //            throw std::runtime_error("Pack's KF and origin's KF have matching time stamps (i.e. below time tolerances)");
            processing_step_ = RUNNING_WITH_PACK_ON_ORIGIN;
        }
        else if (pack->key_frame->getTimeStamp() < origin_ptr_->getTimeStamp())
            processing_step_ = RUNNING_WITH_PACK_BEFORE_ORIGIN;

        else
            processing_step_ = RUNNING_WITH_PACK_AFTER_ORIGIN;

    }
    else
        processing_step_ = RUNNING_WITHOUT_PACK;

    return pack;
}

}

/**
 * \file processor_motion2.h
 *
 *  Created on: 15/03/2016
 *      \author: jsola
 */

#ifndef PROCESSOR_MOTION2_H_
#define PROCESSOR_MOTION2_H_

// Wolf
#include "processor_base.h"
#include "capture_motion2.h"
#include "time_stamp.h"
#include "wolf.h"


/** \brief class for Motion processors
 * \param MotionDeltaType The type of the motion delta and the motion integrated delta. It can be an Eigen::VectorXs (default) or any other construction, most likely a struct.
 *        Generalized Delta types allow for optimized algorithms.
 *        For example, for 3D odometry, a Eigen::VectorXs(6) is sufficient, and is provided as the default template type,
 * \code
 *   typedef Eigen::VectorXs Odo3dDeltaType; // 6-vector with position increment and orientation increment
 * \endcode
 *        If desired, this delta can also be defined as a position delta and an orientation delta,
 * \code
 *   struct Odo3dDeltaType {
 *     Eigen::Vector3s     dp;  // Position delta
 *     Eigen::Vector3s dtheta;  // Orientation delta
 *   } ;
 * \endcode
 *        It can also be defined as a position delta and a quaternion delta,
 * \code
 *   struct Odo3dDeltaType {
 *     Eigen::Vector3s    dp;  // Position delta
 *     Eigen::Quaternions dq;  // Quaternion delta
 *   } ;
 * \endcode
 *        or even as a position delta and a rotation matrix delta,
 * \code
 *   struct Odo3dDeltaType {
 *     Eigen::Vector3s dp;     // Position delta
 *     Eigen::Matrix3s dR;     // Rotation matrix delta
 *   } ;
 * \endcode
 *        As a more challenging example, in an IMU, the Delta type might be defined as:
 * \code
 *   struct ImuDeltaType {
 *     Eigen::Vector3s    dp;  // Position delta
 *     Eigen::Quaternions dq;  // Quaternion delta
 *     Eigen::Vector3s    dv;  // Velocity delta
 *     Eigen::Vector3s    dab; // Acc. bias delta
 *     Eigen::Vector3s    dwb; // Gyro bias delta
 *   } ;
 * \endcode
 *     or using a rotation matrix instead of the quaternion,
 * \code
 *   struct ImuDeltaType {
 *     Eigen::Vector3s dp;     // Position delta
 *     Eigen::Matrix3s dR;     // Rotation matrix delta
 *     Eigen::Vector3s dv;     // Velocity delta
 *     Eigen::Vector3s dab;    // Acc. bias delta
 *     Eigen::Vector3s dwb;    // Gyro bias delta
 *   } ;
 * \endcode
 *       See more examples in the documentation of CaptureMotion2.
 */
template <class MotionDeltaType = Eigen::VectorXs>
class ProcessorMotion2 : public ProcessorBase
{

        // This is the main public interface
    public:
        ProcessorMotion2(ProcessorType _tp, WolfScalar _dt, size_t _state_size, size_t _data_size);
        virtual ~ProcessorMotion2();

        // Instructions to the processor:

        void init(CaptureMotion2<MotionDeltaType>* _origin_ptr);
        virtual void process(CaptureBase* _incoming_ptr);
        void reset(const TimeStamp& _ts);
        void makeKeyFrame(const TimeStamp& _ts);

        // Queries to the processor:

        virtual bool voteForKeyFrame();

        /** \brief Fills a reference to the state integrated so far
         * \param the returned state vector
         */
        const void state(Eigen::VectorXs& _x);
        /** \brief Gets a constant reference to the state integrated so far
         * \return the state vector
         */
        const Eigen::VectorXs state();
        /** \brief Fills the state corresponding to the provided time-stamp
         * \param _t the time stamp
         * \param _x the returned state
         */
        void state(const TimeStamp& _ts, Eigen::VectorXs& _x);
        /** \brief Gets the state corresponding to the provided time-stamp
         * \param _t the time stamp
         * \return the state vector
         */
        Eigen::VectorXs state(const TimeStamp& _ts);
        /** \brief Provides the delta-state integrated so far
         * \return a reference to the integrated delta state
         */
        const MotionDeltaType& deltaState() const;
        /** \brief Provides the delta-state between two time-stamps
         * \param _t1 initial time
         * \param _t2 final time
         * \param _Delta the integrated delta-state between _t1 and _t2
         */
        void deltaState(const TimeStamp& _t1, const TimeStamp& _t2, MotionDeltaType& _Delta);
        /** Composes the deltas in two pre-integrated Captures
         * \param _cap1_ptr pointer to the first Capture
         * \param _cap2_ptr pointer to the second Capture. This is local wrt. the first Capture.
         * \param _delta1_plus_delta2 the concatenation of the deltas of Captures 1 and 2.
         */
        void sumDeltas(CaptureMotion2<MotionDeltaType>* _cap1_ptr, CaptureMotion2<MotionDeltaType>* _cap2_ptr,
                       MotionDeltaType& _delta1_plus_delta2);

        // Helper functions:
    protected:

        void integrate();

        void updateDt();

        typename CaptureMotion2<MotionDeltaType>::MotionBuffer* getBufferPtr();

        const typename CaptureMotion2<MotionDeltaType>::MotionBuffer* getBufferPtr() const;

        // These are the pure virtual functions doing the mathematics
    protected:

         /** \brief convert raw CaptureMotion data to the delta-state format
          *
          * This function accesses the members data_ (as produced by extractData()) and dt_,
          * and computes the value of the delta-state delta_.
          *
          * \param _data the raw motion data
          * \param _dt the time step (not always needed)
          * \param _delta the returned motion delta
          *
          * Rationale:
          *
          * The delta-state format must be compatible for integration using
          * the composition functions doing the math in this class: xPlusDelta(), deltaPlusDelta() and deltaMinusDelta().
          * See the class documentation for some MotionDeltaType suggestions.
          *
          * The data format is generally not the same as the delta format,
          * because it is the format of the raw data provided by the Capture,
          * which is unaware of the needs of this processor.
          *
          * Additionally, sometimes the data format is in the form of a
          * velocity, while the delta is in the form of an increment.
          * In such cases, converting from data to delta-state needs integrating
          * the data over the period dt.
          *
          * Two trivial implementations would establish:
          *  - If data_ is an increment: delta_ = data_;
          *  - If data_ is a velocity: delta_ = data_* dt_.
          *
          *  However, other more complicated relations are possible.
          */
         virtual void data2delta(const Eigen::VectorXs& _data, const WolfScalar _dt, MotionDeltaType& _delta) = 0;

        /** \brief composes a delta-state on top of a state
         * \param _x the initial state
         * \param _delta the delta-state
         * \param _x_plus_delta the updated state. It has the same format as the initial state.
         *
         * This function implements the composition (+) so that _x2 = _x1 (+) _delta.
         */
        virtual void xPlusDelta(const Eigen::VectorXs& _x, const MotionDeltaType& _delta,
                                Eigen::VectorXs& _x_plus_delta) = 0;

        /** \brief composes a delta-state on top of another delta-state
         * \param _delta1 the first delta-state
         * \param _delta2 the second delta-state
         * \param _delta1_plus_delta2 the delta2 composed on top of delta1. It has the format of delta-state.
         *
         * This function implements the composition (+) so that _delta1_plus_delta2 = _delta1 (+) _delta2
         */
        virtual void deltaPlusDelta(const MotionDeltaType& _delta1, const MotionDeltaType& _delta2,
                                    MotionDeltaType& _delta1_plus_delta2) = 0;

        /** \brief Computes the delta-state that goes from one delta-state to another
         * \param _delta1 the initial delta
         * \param _delta2 the final delta
         * \param _delta2_minus_delta1 the delta-state. It has the format of a delta-state.
         *
         * This function implements the composition (-) so that _delta2_minus_delta1 = _delta2 (-) _delta1.
         */
        virtual void deltaMinusDelta(const MotionDeltaType& _delta1, const MotionDeltaType& _delta2,
                                     MotionDeltaType& _delta2_minus_delta1) = 0;

        /** \brief Delta zero
         * \return a delta state equivalent to the null motion.
         *
         * Examples (see documentation of the the class for info on MotionDeltaType):
         *   - 2D odometry: a 3-vector with all zeros, e.g. Vector3s::Zero()
         *   - 3D odometry: different examples:
         *     - delta type is a PQ vector: 7-vector with [0,0,0,0,0,0,1]
         *     - delta type is a {P,Q} struct with {[0,0,0],[0,0,0,1]}
         *     - delta type is a {P,R} struct with {[0,0,0],Matrix3s::Identity()}
         *   - IMU: examples:
         *     - delta type is a {P,Q,V} struct with {[0,0,0],[0,0,0,1],[0,0,0]}
         *     - delta type is a {P,Q,V,Ab,Wb} struct with {[0,0,0],[0,0,0,1],[0,0,0],[0,0,0],[0,0,0]}
         */
        virtual MotionDeltaType deltaZero() const = 0;

    protected:
        // Attributes
        size_t x_size_;    ///< The size of the state vector
        size_t data_size_; ///< the size of the incoming data
        CaptureMotion2<MotionDeltaType>* origin_ptr_;
        CaptureMotion2<MotionDeltaType>* last_ptr_;
        CaptureMotion2<MotionDeltaType>* incoming_ptr_;

    protected:
        // helpers to avoid allocation
        WolfScalar dt_; ///< Time step
        Eigen::VectorXs x_; ///< state temporary
        MotionDeltaType delta_, delta_integrated_; ///< current delta and integrated deltas
        Eigen::VectorXs data_; ///< current data

};

template<class MotionDeltaType>
inline ProcessorMotion2<MotionDeltaType>::ProcessorMotion2(ProcessorType _tp, WolfScalar _dt, size_t _state_size,
                                                           size_t _data_size) :
        ProcessorBase(_tp), x_size_(_state_size), data_size_(_data_size),
        origin_ptr_(nullptr), last_ptr_(nullptr), incoming_ptr_(nullptr),
        dt_(_dt), x_(_state_size), data_(_data_size)
{
    //
}

template<class MotionDeltaType>
inline ProcessorMotion2<MotionDeltaType>::~ProcessorMotion2()
{
    //
}

template<class MotionDeltaType>
inline void ProcessorMotion2<MotionDeltaType>::process(CaptureBase* _incoming_ptr)
{
    incoming_ptr_ = (CaptureMotion2<MotionDeltaType>*)(_incoming_ptr);

    integrate();

    if (voteForKeyFrame() && permittedKeyFrame())
    {
        // TODO:
        // Make KeyFrame
        //        makeKeyFrame();
        // Reset the Tracker
        //        reset();
    }
}

template<class MotionDeltaType>
inline void ProcessorMotion2<MotionDeltaType>::init(CaptureMotion2<MotionDeltaType>* _origin_ptr)
{
    origin_ptr_ = _origin_ptr;
    last_ptr_ = _origin_ptr;
    incoming_ptr_ = nullptr;
    delta_integrated_ = deltaZero();
    getBufferPtr()->clear();
    getBufferPtr()->pushBack(_origin_ptr->getTimeStamp(), delta_integrated_);
}

template<class MotionDeltaType>
inline void ProcessorMotion2<MotionDeltaType>::reset(const TimeStamp& _ts)
{
    // TODO what to do?
    //cut the buffer in 2 parts at _ts
    // create a new Capture for the future
    // Create a
}

template<class MotionDeltaType>
inline void ProcessorMotion2<MotionDeltaType>::makeKeyFrame(const TimeStamp& _ts)
{
    //TODO: see how to adapt this code from ProcessorTracker::makeKeyFrame(void)
    // Create a new non-key Frame in the Trajectory with the incoming Capture
    getWolfProblem()->createFrame(NON_KEY_FRAME, state(_ts), _ts);
    getWolfProblem()->getLastFramePtr()->addCapture(incoming_ptr_); // Add incoming Capture to the new Frame
    // Make the last Capture's Frame a KeyFrame so that it gets into the solver
    last_ptr_->getFramePtr()->setKey();
}

template<class MotionDeltaType>
inline bool ProcessorMotion2<MotionDeltaType>::voteForKeyFrame()
{
    return false;
}

template<class MotionDeltaType>
inline Eigen::VectorXs ProcessorMotion2<MotionDeltaType>::state(const TimeStamp& _ts)
{
    state(_ts, x_);
    return x_;
}

template<class MotionDeltaType>
inline void ProcessorMotion2<MotionDeltaType>::state(const TimeStamp& _ts, Eigen::VectorXs& _x)
{
    xPlusDelta(origin_ptr_->getFramePtr()->getState(), getBufferPtr()->getDelta(_ts), _x);
}

template<class MotionDeltaType>
inline const Eigen::VectorXs ProcessorMotion2<MotionDeltaType>::state()
{
    state(x_);
    return x_;
}

template<class MotionDeltaType>
inline const void ProcessorMotion2<MotionDeltaType>::state(Eigen::VectorXs& _x)
{
    xPlusDelta(origin_ptr_->getFramePtr()->getState(), getBufferPtr()->getDelta(), _x);
}

template<class MotionDeltaType>
inline void ProcessorMotion2<MotionDeltaType>::deltaState(const TimeStamp& _t1, const TimeStamp& _t2,
                                                          MotionDeltaType& _Delta)
{
    deltaMinusDelta(getBufferPtr()->getDelta(_t2), getBufferPtr()->getDelta(_t2), _Delta);
}

template<class MotionDeltaType>
inline const MotionDeltaType& ProcessorMotion2<MotionDeltaType>::deltaState() const
{
    return getBufferPtr()->getDelta();
}

template<class MotionDeltaType>
inline void ProcessorMotion2<MotionDeltaType>::sumDeltas(CaptureMotion2<MotionDeltaType>* _cap1_ptr,
                                                         CaptureMotion2<MotionDeltaType>* _cap2_ptr,
                                                         MotionDeltaType& _delta1_plus_delta2)
{
    deltaPlusDelta(_cap1_ptr->getDelta(), _cap2_ptr->getDelta(), _delta1_plus_delta2);
}

template<class MotionDeltaType>
inline void ProcessorMotion2<MotionDeltaType>::integrate()
{
    // Set dt
    updateDt();
    // get data and convert it to delta
    data2delta(incoming_ptr_->getData(), dt_, delta_);
    // then integrate
    deltaPlusDelta(getBufferPtr()->getDelta(), delta_, delta_integrated_);
    // then push it into buffer
    getBufferPtr()->pushBack(incoming_ptr_->getTimeStamp(), delta_integrated_);
}

template<class MotionDeltaType>
inline void ProcessorMotion2<MotionDeltaType>::updateDt()
{
    dt_ = incoming_ptr_->getTimeStamp() - getBufferPtr()->getTimeStamp();
}

template<class MotionDeltaType>
inline const typename CaptureMotion2<MotionDeltaType>::MotionBuffer* ProcessorMotion2<MotionDeltaType>::getBufferPtr() const
{
    return last_ptr_->getBufferPtr();
}

template<class MotionDeltaType>
inline typename CaptureMotion2<MotionDeltaType>::MotionBuffer* ProcessorMotion2<MotionDeltaType>::getBufferPtr()
{
    return last_ptr_->getBufferPtr();
}

#endif /* PROCESSOR_MOTION2_H_ */

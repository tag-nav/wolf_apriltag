
#ifndef FRAME_BASE_H_
#define FRAME_BASE_H_

// Fwd refs
namespace wolf{
class TrajectoryBase;
class CaptureBase;
class StateBlock;
}

//Wolf includes
#include "wolf.h"
#include "time_stamp.h"

//std includes

namespace wolf {

//class FrameBase
class FrameBase : public NodeBase // NodeConstrained<TrajectoryBase,CaptureBase>
{
    private:
        ProblemPtr problem_ptr_;
        TrajectoryBasePtr trajectory_ptr_;
        CaptureBaseList capture_list_;

        static unsigned int frame_id_count_;
    protected:
        unsigned int frame_id_;
        FrameKeyType type_id_;         ///< type of frame. Either NON_KEY_FRAME or KEY_FRAME. (types defined at wolf.h)
        TimeStamp time_stamp_;   ///< frame time stamp
        StateStatus status_;     ///< status of the estimation of the frame state
        StateBlock* p_ptr_;      ///< Position state block pointer
        StateBlock* o_ptr_;      ///< Orientation state block pointer
        StateBlock* v_ptr_;      ///< Linear velocity state block pointer
        
    public:

        /** \brief Constructor of non-key Frame with only time stamp
         *
         * Constructor with only time stamp
         * \param _ts is the time stamp associated to this frame, provided in seconds
         * \param _p_ptr StateBlock pointer to the position (default: nullptr)
         * \param _o_ptr StateBlock pointer to the orientation (default: nullptr). Pass a StateQuaternion if needed.
         * \param _v_ptr StateBlock pointer to the velocity (default: nullptr).
         **/
        FrameBase(const TimeStamp& _ts, StateBlock* _p_ptr, StateBlock* _o_ptr = nullptr, StateBlock* _v_ptr = nullptr);
        
        /** \brief Constructor with type, time stamp and state pointer
         * 
         * Constructor with type, time stamp and state pointer
         * \param _tp indicates frame type. Generally either NON_KEY_FRAME or KEY_FRAME. (types defined at wolf.h)
         * \param _ts is the time stamp associated to this frame, provided in seconds
         * \param _p_ptr StateBlock pointer to the position (default: nullptr)
         * \param _o_ptr StateBlock pointer to the orientation (default: nullptr)
         * \param _v_ptr StateBlock pointer to the velocity (default: nullptr).
         **/        
        FrameBase(const FrameKeyType & _tp, const TimeStamp& _ts, StateBlock* _p_ptr, StateBlock* _o_ptr = nullptr, StateBlock* _v_ptr = nullptr);

        /** \brief Default destructor (not recommended)
         *
         * Default destructor (please use destruct() instead of delete for guaranteeing the wolf tree integrity)
         * 
         **/
        virtual ~FrameBase();

        unsigned int id();



        // Frame properties -----------------------------------------------

        // KeyFrame / NonKeyFrame
        bool isKey() const;
        void setKey();

        // Fixed / Estimated
        void fix();
        void unfix();
        bool isFixed() const;



        // Frame values ------------------------------------------------

        void setTimeStamp(const TimeStamp& _ts);
        TimeStamp getTimeStamp() const;
        void getTimeStamp(TimeStamp& _ts) const;

        StateBlock* getPPtr() const;
        StateBlock* getOPtr() const;
        StateBlock* getVPtr() const;

        void setState(const Eigen::VectorXs& _st);
        virtual Eigen::VectorXs getState() const;
        void getState(Eigen::VectorXs& state) const;


        // Wolf tree access ---------------------------------------------------

        TrajectoryBasePtr getTrajectoryPtr() const;
        void setTrajectoryPtr(TrajectoryBasePtr _trj_ptr);

        FrameBasePtr getPreviousFrame() const;
        FrameBasePtr getNextFrame() const;

        CaptureBaseList* getCaptureListPtr();
        CaptureBasePtr addCapture(CaptureBase* _capt_ptr);
        void removeCapture(CaptureBaseIter& _capt_iter);
        void removeCapture(CaptureBasePtr _capt_ptr);
        CaptureBasePtr hasCaptureOf(const SensorBasePtr _sensor_ptr);

        void getConstraintList(ConstraintBaseList & _ctr_list);

        /** \brief Adds all stateBlocks of the frame to the wolfProblem list of new stateBlocks
         **/
        virtual void registerNewStateBlocks();

        ProblemPtr getProblem(){return problem_ptr_;}
        void setProblem(Problem* _prob_ptr){problem_ptr_ = _prob_ptr;}



    private:
        /** \brief Gets the Frame status (see wolf.h for Frame status)
         **/
        StateStatus getStatus() const;
        /** \brief Sets the Frame status (see wolf.h for Frame status)
         **/
        void setStatus(StateStatus _st);




};

} // namespace wolf

#include "capture_base.h"

namespace wolf {

// IMPLEMENTATION //

inline unsigned int FrameBase::id()
{
    return frame_id_;
}

inline bool FrameBase::isKey() const
{
    return (type_id_ == KEY_FRAME);
}

inline void FrameBase::fix()
{
    this->setStatus(ST_FIXED);
}

inline void FrameBase::unfix()
{
    //std::cout << "Unfixing frame " << nodeId() << std::endl;
    this->setStatus(ST_ESTIMATED);
}

inline bool FrameBase::isFixed() const
{
    return status_ == ST_FIXED;
}

inline void FrameBase::setTimeStamp(const TimeStamp& _ts)
{
    time_stamp_ = _ts;
}

inline void FrameBase::getTimeStamp(TimeStamp& _ts) const
{
    _ts = time_stamp_.get();
}

inline TimeStamp FrameBase::getTimeStamp() const
{
    return time_stamp_.get();
}

inline StateBlock* FrameBase::getPPtr() const
{
    return p_ptr_;
}

inline StateBlock* FrameBase::getOPtr() const
{
    return o_ptr_;
}

inline StateBlock* FrameBase::getVPtr() const
{
    return v_ptr_;
}

inline TrajectoryBase* FrameBase::getTrajectoryPtr() const
{
    return trajectory_ptr_;
//    return upperNodePtr();
}

inline CaptureBaseList* FrameBase::getCaptureListPtr()
{
    return & capture_list_;
//    return getDownNodeListPtr();
}

inline CaptureBase* FrameBase::addCapture(CaptureBase* _capt_ptr)
{
    capture_list_.push_back(_capt_ptr);
    _capt_ptr->setFramePtr(this);
//    addDownNode(_capt_ptr);
    return _capt_ptr;
}

inline void FrameBase::removeCapture(CaptureBaseIter& _capt_iter)
{
    //std::cout << "removing capture " << (*_capt_iter)->nodeId() << " from Frame " << nodeId() << std::endl;
    removeDownNode(_capt_iter);
}

inline void FrameBase::removeCapture(CaptureBase* _capt_ptr)
{
    //std::cout << "removing capture " << (*_capt_iter)->nodeId() << " from Frame " << nodeId() << std::endl;
    removeDownNode(_capt_ptr);
}

inline StateStatus FrameBase::getStatus() const
{
    return status_;
}

} // namespace wolf

#endif

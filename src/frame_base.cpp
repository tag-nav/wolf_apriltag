
#include "frame_base.h"
#include "constraint_base.h"
#include "trajectory_base.h"
#include "capture_base.h"
#include "state_block.h"

namespace wolf {

unsigned int FrameBase::frame_id_count_ = 0;

FrameBase::FrameBase(const TimeStamp& _ts, StateBlock* _p_ptr, StateBlock* _o_ptr, StateBlock* _v_ptr) :
            NodeBase("FRAME", "BASE"),
            trajectory_ptr_(),
            frame_id_(++frame_id_count_),
            type_id_(NON_KEY_FRAME),
            time_stamp_(_ts),
			status_(ST_ESTIMATED),
			p_ptr_(_p_ptr),
            o_ptr_(_o_ptr),
            v_ptr_(_v_ptr)
{
    //
    if (isKey())
        std::cout << "constructed +KF" << id() << std::endl;
    else
        std::cout << "constructed  +F" << id() << std::endl;
}

FrameBase::FrameBase(const FrameKeyType & _tp, const TimeStamp& _ts, StateBlock* _p_ptr, StateBlock* _o_ptr, StateBlock* _v_ptr) :
            NodeBase("FRAME", "BASE"),
            trajectory_ptr_(),
            frame_id_(++frame_id_count_),
            type_id_(_tp),
            time_stamp_(_ts),
			status_(ST_ESTIMATED),
			p_ptr_(_p_ptr),
            o_ptr_(_o_ptr),
            v_ptr_(_v_ptr)
{
    //
    if (isKey())
        std::cout << "constructed +KF" << id() << std::endl;
    else
        std::cout << "constructed  +F" << id() << std::endl;
}
                
FrameBase::~FrameBase()
{
    if (isKey())
        std::cout << "destructed   KF" << id() << std::endl;
    else
        std::cout << "destructed   -F" << id() << std::endl;
}

void FrameBase::remove()
{
    std::cout << "Remove        F" << id() << std::endl;
    if (!is_removing_)
    {
        std::cout << "Removing      F" << id() << std::endl;
        is_removing_ = true;
        FrameBasePtr this_F = shared_from_this(); // keep this alive while removing it
        TrajectoryBasePtr trj = trajectory_ptr_.lock();
        if (trj)
        {
            std::cout << "Removing      F" << id() << " from T" << std::endl;
            trj->getFrameList().remove(this_F); // remove from upstream
        }

        while (!capture_list_.empty())
        {
            capture_list_.front()->remove(); // remove downstream
            capture_list_.pop_front();
        }
        while (!constrained_by_list_.empty())
        {
            constrained_by_list_.front()->remove(); // remove constrained
            constrained_by_list_.pop_front();
        }

        // Remove Frame State Blocks
        if (p_ptr_ != nullptr)
        {
            if (getProblem() != nullptr && type_id_ == KEY_FRAME)
                getProblem()->removeStateBlockPtr(p_ptr_);
            delete p_ptr_;
            p_ptr_ = nullptr;
        }
        if (o_ptr_ != nullptr)
        {
            if (getProblem() != nullptr && type_id_ == KEY_FRAME)
                getProblem()->removeStateBlockPtr(o_ptr_);
            delete o_ptr_;
            o_ptr_ = nullptr;
        }
        if (v_ptr_ != nullptr)
        {
            if (getProblem() != nullptr && type_id_ == KEY_FRAME)
                getProblem()->removeStateBlockPtr(v_ptr_);
            delete v_ptr_;
            v_ptr_ = nullptr;
        }
        std::cout << "Removed       F" << id() << std::endl;
    }
}

void FrameBase::registerNewStateBlocks()
{
    if (getProblem() != nullptr)
    {
        if (p_ptr_ != nullptr)
            getProblem()->addStateBlockPtr(p_ptr_);

        if (o_ptr_ != nullptr)
            getProblem()->addStateBlockPtr(o_ptr_);

        if (v_ptr_ != nullptr)
            getProblem()->addStateBlockPtr(v_ptr_);
    }
}

void FrameBase::setKey()
{
    if (type_id_ != KEY_FRAME)
    {
        type_id_ = KEY_FRAME;
        registerNewStateBlocks();

        if (getTrajectoryPtr()->getLastKeyFramePtr() == nullptr || getTrajectoryPtr()->getLastKeyFramePtr()->getTimeStamp() < time_stamp_)
            getTrajectoryPtr()->setLastKeyFramePtr(shared_from_this());

        getTrajectoryPtr()->sortFrame(shared_from_this());
    }
}

void FrameBase::setState(const Eigen::VectorXs& _st)
{

    assert(_st.size() == ((p_ptr_==nullptr ? 0 : p_ptr_->getSize())  +
                          (o_ptr_==nullptr ? 0 : o_ptr_->getSize())  +
                          (v_ptr_==nullptr ? 0 : v_ptr_->getSize())) &&
                          "In FrameBase::setState wrong state size");

    unsigned int index = 0;
    if (p_ptr_!=nullptr)
    {
        p_ptr_->setVector(_st.head(p_ptr_->getSize()));
        index += p_ptr_->getSize();
    }
    if (o_ptr_!=nullptr)
    {
        o_ptr_->setVector(_st.segment(index, o_ptr_->getSize()));
        index += p_ptr_->getSize();
    }
    if (v_ptr_!=nullptr)
    {
        v_ptr_->setVector(_st.segment(index, v_ptr_->getSize()));
        //   index += v_ptr_->getSize();
    }
}

Eigen::VectorXs FrameBase::getState() const
{
    Eigen::VectorXs state((p_ptr_==nullptr ? 0 : p_ptr_->getSize()) +
                          (o_ptr_==nullptr ? 0 : o_ptr_->getSize())  +
                          (v_ptr_==nullptr ? 0 : v_ptr_->getSize()));

    getState(state);

    return state;
}

void FrameBase::getState(Eigen::VectorXs& state) const
{
    assert(state.size() == ((p_ptr_==nullptr ? 0 : p_ptr_->getSize()) +
                            (o_ptr_==nullptr ? 0 : o_ptr_->getSize())  +
                            (v_ptr_==nullptr ? 0 : v_ptr_->getSize())));

    unsigned int index = 0;
    if (p_ptr_!=nullptr)
    {
        state.head(p_ptr_->getSize()) = p_ptr_->getVector();
        index += p_ptr_->getSize();
    }
    if (o_ptr_!=nullptr)
    {
        state.segment(index, o_ptr_->getSize()) = o_ptr_->getVector();
        index += p_ptr_->getSize();
    }
    if (v_ptr_!=nullptr)
    {
        state.segment(index, v_ptr_->getSize()) = v_ptr_->getVector();
        //   index += v_ptr_->getSize();
    }
}

FrameBasePtr FrameBase::getPreviousFrame() const
{
    //std::cout << "finding previous frame of " << this->node_id_ << std::endl;
    if (getTrajectoryPtr() == nullptr)
        //std::cout << "This Frame is not linked to any trajectory" << std::endl;

    assert(getTrajectoryPtr() != nullptr && "This Frame is not linked to any trajectory");

    //look for the position of this node in the upper list (frame list of trajectory)
    for (auto f_it = getTrajectoryPtr()->getFrameList().rbegin(); f_it != getTrajectoryPtr()->getFrameList().rend(); f_it++ )
    {
        if ( this->node_id_ == (*f_it)->nodeId() )
        {
        	f_it++;
        	if (f_it != getTrajectoryPtr()->getFrameList().rend())
            {
                //std::cout << "previous frame found!" << std::endl;
                return *f_it;
            }
        	else
        	{
        	    //std::cout << "previous frame not found!" << std::endl;
        	    return nullptr;
        	}
        }
    }
    //std::cout << "previous frame not found!" << std::endl;
    return nullptr;
}

FrameBasePtr FrameBase::getNextFrame() const
{
    //std::cout << "finding next frame of " << this->node_id_ << std::endl;
	auto f_it = getTrajectoryPtr()->getFrameList().rbegin();
	f_it++; //starting from second last frame

    //look for the position of this node in the frame list of trajectory
    while (f_it != getTrajectoryPtr()->getFrameList().rend())
    {
        if ( this->node_id_ == (*f_it)->nodeId())
        {
        	f_it--;
			return *f_it;
        }
    	f_it++;
    }
    std::cout << "next frame not found!" << std::endl;
    return nullptr;
}

//void FrameBase::destruct()
//{
//    if (!is_removing_)
//    {
//        if (trajectory_ptr_ != nullptr) // && !up_node_ptr_->isTop())
//            trajectory_ptr_->removeFrame(this);
//        else
//            delete this;
//    }
//}

void FrameBase::setStatus(StateStatus _st)
{
    // TODO: Separate the three fixes and unfixes to the wolfproblem lists
    status_ = _st;
    // State Blocks
    if (status_ == ST_FIXED)
    {
        if (p_ptr_ != nullptr)
        {
            p_ptr_->fix();
            if (getProblem() != nullptr)
                getProblem()->updateStateBlockPtr(p_ptr_);
        }
        if (o_ptr_ != nullptr)
        {
            o_ptr_->fix();
            if (getProblem() != nullptr)
                getProblem()->updateStateBlockPtr(o_ptr_);
        }
        if (v_ptr_ != nullptr)
        {
            v_ptr_->fix();
            if (getProblem() != nullptr)
                getProblem()->updateStateBlockPtr(v_ptr_);
        }
    }
    else if (status_ == ST_ESTIMATED)
    {
        if (p_ptr_ != nullptr)
        {
            p_ptr_->unfix();
            if (getProblem() != nullptr)
                getProblem()->updateStateBlockPtr(p_ptr_);
        }
        if (o_ptr_ != nullptr)
        {
            o_ptr_->unfix();
            if (getProblem() != nullptr)
                getProblem()->updateStateBlockPtr(o_ptr_);
        }
        if (v_ptr_ != nullptr)
        {
            v_ptr_->unfix();
            if (getProblem() != nullptr)
                getProblem()->updateStateBlockPtr(v_ptr_);
        }
    }
}

} // namespace wolf

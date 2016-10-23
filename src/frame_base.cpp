
#include "frame_base.h"
#include "constraint_base.h"
#include "trajectory_base.h"
#include "capture_base.h"
#include "state_block.h"

namespace wolf {

unsigned int FrameBase::frame_id_count_ = 0;

FrameBase::FrameBase(const TimeStamp& _ts, StateBlockPtr _p_ptr, StateBlockPtr _o_ptr, StateBlockPtr _v_ptr) :
            NodeBase("FRAME", "BASE"),
            trajectory_ptr_(),
            state_block_vec_(6), // allow for 6 state blocks by default. Should be enough in all applications.
            frame_id_(++frame_id_count_),
            type_id_(NON_KEY_FRAME),
			status_(ST_ESTIMATED),
            time_stamp_(_ts)
{
    state_block_vec_[0] = _p_ptr;
    state_block_vec_[1] = _o_ptr;
    state_block_vec_[2] = _v_ptr;
    state_block_vec_[3] = nullptr;
    state_block_vec_[4] = nullptr;
    state_block_vec_[5] = nullptr;
    //
    if (isKey())
        std::cout << "constructed +KF" << id() << std::endl;
    else
        std::cout << "constructed  +F" << id() << std::endl;
}

FrameBase::FrameBase(const FrameKeyType & _tp, const TimeStamp& _ts, StateBlockPtr _p_ptr, StateBlockPtr _o_ptr, StateBlockPtr _v_ptr) :
            NodeBase("FRAME", "BASE"),
            trajectory_ptr_(),
            state_block_vec_(6), // allow for 6 state blocks by default. Should be enough in all applications.
            frame_id_(++frame_id_count_),
            type_id_(_tp),
			status_(ST_ESTIMATED),
            time_stamp_(_ts)
{
    state_block_vec_[0] = _p_ptr;
    state_block_vec_[1] = _o_ptr;
    state_block_vec_[2] = _v_ptr;
    state_block_vec_[3] = nullptr;
    state_block_vec_[4] = nullptr;
    state_block_vec_[5] = nullptr;

    if (isKey())
        std::cout << "constructed +KF" << id() << std::endl;
    else
        std::cout << "constructed  +F" << id() << std::endl;
}
                
FrameBase::~FrameBase()
{
    // Remove Frame State Blocks
    removeStateBlocks();

    if (isKey())
        std::cout << "destructed  -KF" << id() << std::endl;
    else
        std::cout << "destructed   -F" << id() << std::endl;
}

void FrameBase::remove()
{
    if (!is_removing_)
    {
        is_removing_ = true;
        FrameBasePtr this_F = shared_from_this(); // keep this alive while removing it
        TrajectoryBasePtr T = trajectory_ptr_.lock();
        if (T)
        {
            T->getFrameList().remove(this_F); // remove from upstream
        }

        while (!capture_list_.empty())
        {
            capture_list_.front()->remove(); // remove downstream
        }
        while (!constrained_by_list_.empty())
        {
            constrained_by_list_.front()->remove(); // remove constrained
        }

        // Remove Frame State Blocks
        std::cout << __FILE__ << ":" << __FUNCTION__ << "():" << __LINE__ << std::endl;

        removeStateBlocks();

        std::cout << "Removed       F" << id() << std::endl;
    }
}

void FrameBase::registerNewStateBlocks()
{
    if (getProblem() != nullptr)
    {
        for (auto sbp : getStateBlockVec())
            if (sbp != nullptr)
                getProblem()->addStateBlock(sbp);
    }
}

void FrameBase::removeStateBlocks()
{
    for (unsigned int i = 0; i < state_block_vec_.size(); i++)
    {
        auto sbp = getStateBlockPtr(i);
        if (sbp != nullptr)
        {
            if (getProblem() != nullptr && type_id_ == KEY_FRAME)
            {
                getProblem()->removeStateBlockPtr(sbp);
            }
            delete sbp;
            setStateBlockPtr(i, nullptr);
        }
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

    assert(_st.size() == ((getPPtr()==nullptr ? 0 : getPPtr()->getSize())  +
                          (getOPtr()==nullptr ? 0 : getOPtr()->getSize())  +
                          (getVPtr()==nullptr ? 0 : getVPtr()->getSize())) &&
                          "In FrameBase::setState wrong state size");

    unsigned int index = 0;
    if (getPPtr()!=nullptr)
    {
        getPPtr()->setVector(_st.head(getPPtr()->getSize()));
        index += getPPtr()->getSize();
    }
    if (getOPtr()!=nullptr)
    {
        getOPtr()->setVector(_st.segment(index, getOPtr()->getSize()));
        index += getOPtr()->getSize();
    }
    if (getVPtr()!=nullptr)
    {
        getVPtr()->setVector(_st.segment(index, getVPtr()->getSize()));
        //   index += getVPtr()->getSize();
    }
}

Eigen::VectorXs FrameBase::getState() const
{
    Eigen::VectorXs state((getPPtr()==nullptr ? 0 : getPPtr()->getSize()) +
                          (getOPtr()==nullptr ? 0 : getOPtr()->getSize())  +
                          (getVPtr()==nullptr ? 0 : getVPtr()->getSize()));

    getState(state);

    return state;
}

void FrameBase::getState(Eigen::VectorXs& state) const
{
    assert(state.size() == ((getPPtr()==nullptr ? 0 : getPPtr()->getSize()) +
                            (getOPtr()==nullptr ? 0 : getOPtr()->getSize())  +
                            (getVPtr()==nullptr ? 0 : getVPtr()->getSize())));

    unsigned int index = 0;
    if (getPPtr()!=nullptr)
    {
        state.head(getPPtr()->getSize()) = getPPtr()->getVector();
        index += getPPtr()->getSize();
    }
    if (getOPtr()!=nullptr)
    {
        state.segment(index, getOPtr()->getSize()) = getOPtr()->getVector();
        index += getOPtr()->getSize();
    }
    if (getVPtr()!=nullptr)
    {
        state.segment(index, getVPtr()->getSize()) = getVPtr()->getVector();
        //   index += getVPtr()->getSize();
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

void FrameBase::setStatus(StateStatus _st)
{
    // TODO: Separate the three fixes and unfixes to the wolfproblem lists
    status_ = _st;
    // State Blocks
    if (status_ == ST_FIXED)
    {
        if (getPPtr() != nullptr)
        {
            getPPtr()->fix();
            if (getProblem() != nullptr)
                getProblem()->updateStateBlockPtr(getPPtr());
        }
        if (getOPtr() != nullptr)
        {
            getOPtr()->fix();
            if (getProblem() != nullptr)
                getProblem()->updateStateBlockPtr(getOPtr());
        }
        if (getVPtr() != nullptr)
        {
            getVPtr()->fix();
            if (getProblem() != nullptr)
                getProblem()->updateStateBlockPtr(getVPtr());
        }
    }
    else if (status_ == ST_ESTIMATED)
    {
        if (getPPtr() != nullptr)
        {
            getPPtr()->unfix();
            if (getProblem() != nullptr)
                getProblem()->updateStateBlockPtr(getPPtr());
        }
        if (getOPtr() != nullptr)
        {
            getOPtr()->unfix();
            if (getProblem() != nullptr)
                getProblem()->updateStateBlockPtr(getOPtr());
        }
        if (getVPtr() != nullptr)
        {
            getVPtr()->unfix();
            if (getProblem() != nullptr)
                getProblem()->updateStateBlockPtr(getVPtr());
        }
    }
}

} // namespace wolf

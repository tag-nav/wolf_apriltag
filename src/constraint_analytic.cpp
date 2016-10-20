#include "constraint_analytic.h"
#include "state_block.h"

namespace wolf {

ConstraintAnalytic::ConstraintAnalytic(ConstraintType _tp, bool _apply_loss_function, ConstraintStatus _status,
                                       StateBlockPtr _state0Ptr, StateBlockPtr _state1Ptr, StateBlockPtr _state2Ptr, StateBlockPtr _state3Ptr, StateBlockPtr _state4Ptr,
                                       StateBlockPtr _state5Ptr, StateBlockPtr _state6Ptr, StateBlockPtr _state7Ptr, StateBlockPtr _state8Ptr, StateBlockPtr _state9Ptr ) :
            ConstraintBase(_tp, _apply_loss_function, _status),
            state_ptr_vector_({_state0Ptr, _state1Ptr, _state2Ptr, _state3Ptr, _state4Ptr,
                               _state5Ptr, _state6Ptr, _state7Ptr, _state8Ptr, _state9Ptr})
{
    resizeVectors();
}

ConstraintAnalytic::ConstraintAnalytic(ConstraintType _tp, FrameBasePtr _frame_ptr, bool _apply_loss_function, ConstraintStatus _status,
                                       StateBlockPtr _state0Ptr, StateBlockPtr _state1Ptr, StateBlockPtr _state2Ptr, StateBlockPtr _state3Ptr, StateBlockPtr _state4Ptr,
                                       StateBlockPtr _state5Ptr, StateBlockPtr _state6Ptr, StateBlockPtr _state7Ptr, StateBlockPtr _state8Ptr, StateBlockPtr _state9Ptr ) :
            ConstraintBase(_tp, _frame_ptr, _apply_loss_function, _status),
            state_ptr_vector_({_state0Ptr, _state1Ptr, _state2Ptr, _state3Ptr, _state4Ptr,
                               _state5Ptr, _state6Ptr, _state7Ptr, _state8Ptr, _state9Ptr})
{
    resizeVectors();
}

ConstraintAnalytic::ConstraintAnalytic(ConstraintType _tp, FeatureBasePtr _feature_ptr, bool _apply_loss_function, ConstraintStatus _status,
                                       StateBlockPtr _state0Ptr, StateBlockPtr _state1Ptr, StateBlockPtr _state2Ptr, StateBlockPtr _state3Ptr, StateBlockPtr _state4Ptr,
                                       StateBlockPtr _state5Ptr, StateBlockPtr _state6Ptr, StateBlockPtr _state7Ptr, StateBlockPtr _state8Ptr, StateBlockPtr _state9Ptr ) :
            ConstraintBase( _tp, _feature_ptr, _apply_loss_function, _status),
            state_ptr_vector_({_state0Ptr, _state1Ptr, _state2Ptr, _state3Ptr, _state4Ptr,
                               _state5Ptr, _state6Ptr, _state7Ptr, _state8Ptr, _state9Ptr})
{
    resizeVectors();
}


ConstraintAnalytic::ConstraintAnalytic(ConstraintType _tp, LandmarkBasePtr _landmark_ptr, bool _apply_loss_function, ConstraintStatus _status,
                                       StateBlockPtr _state0Ptr, StateBlockPtr _state1Ptr, StateBlockPtr _state2Ptr, StateBlockPtr _state3Ptr, StateBlockPtr _state4Ptr,
                                       StateBlockPtr _state5Ptr, StateBlockPtr _state6Ptr, StateBlockPtr _state7Ptr, StateBlockPtr _state8Ptr, StateBlockPtr _state9Ptr ) :
            ConstraintBase( _tp, _landmark_ptr, _apply_loss_function, _status),
            state_ptr_vector_({_state0Ptr, _state1Ptr, _state2Ptr, _state3Ptr, _state4Ptr,
                               _state5Ptr, _state6Ptr, _state7Ptr, _state8Ptr, _state9Ptr})
{
    resizeVectors();
}


ConstraintAnalytic::~ConstraintAnalytic()
{
    //
}


const std::vector<Scalar*> ConstraintAnalytic::getStateBlockPtrVector()
{
    assert(state_ptr_vector_.size() > 0 && state_ptr_vector_.size() <= 10 && "Wrong state vector size in constraint, it should be between 1 and 10");

    switch (state_ptr_vector_.size())
    {
        case 1:
        {
            return std::vector<Scalar*>({state_ptr_vector_[0]->getPtr()});
        }
        case 2:
        {
            return std::vector<Scalar*>({state_ptr_vector_[0]->getPtr(),
                                             state_ptr_vector_[1]->getPtr()});
        }
        case 3:
        {
            return std::vector<Scalar*>({state_ptr_vector_[0]->getPtr(),
                                             state_ptr_vector_[1]->getPtr(),
                                             state_ptr_vector_[2]->getPtr()});
        }
        case 4:
        {
            return std::vector<Scalar*>({state_ptr_vector_[0]->getPtr(),
                                             state_ptr_vector_[1]->getPtr(),
                                             state_ptr_vector_[2]->getPtr(),
                                             state_ptr_vector_[3]->getPtr()});
        }
        case 5:
        {
            return std::vector<Scalar*>({state_ptr_vector_[0]->getPtr(),
                                             state_ptr_vector_[1]->getPtr(),
                                             state_ptr_vector_[2]->getPtr(),
                                             state_ptr_vector_[3]->getPtr(),
                                             state_ptr_vector_[4]->getPtr()});
        }
        case 6:
        {
            return std::vector<Scalar*>({state_ptr_vector_[0]->getPtr(),
                                             state_ptr_vector_[1]->getPtr(),
                                             state_ptr_vector_[2]->getPtr(),
                                             state_ptr_vector_[3]->getPtr(),
                                             state_ptr_vector_[4]->getPtr(),
                                             state_ptr_vector_[5]->getPtr()});
        }
        case 7:
        {
            return std::vector<Scalar*>({state_ptr_vector_[0]->getPtr(),
                                             state_ptr_vector_[1]->getPtr(),
                                             state_ptr_vector_[2]->getPtr(),
                                             state_ptr_vector_[3]->getPtr(),
                                             state_ptr_vector_[4]->getPtr(),
                                             state_ptr_vector_[5]->getPtr(),
                                             state_ptr_vector_[6]->getPtr()});
        }
        case 8:
        {
            return std::vector<Scalar*>({state_ptr_vector_[0]->getPtr(),
                                             state_ptr_vector_[1]->getPtr(),
                                             state_ptr_vector_[2]->getPtr(),
                                             state_ptr_vector_[3]->getPtr(),
                                             state_ptr_vector_[4]->getPtr(),
                                             state_ptr_vector_[5]->getPtr(),
                                             state_ptr_vector_[6]->getPtr(),
                                             state_ptr_vector_[7]->getPtr()});
        }
        case 9:
        {
            return std::vector<Scalar*>({state_ptr_vector_[0]->getPtr(),
                                             state_ptr_vector_[1]->getPtr(),
                                             state_ptr_vector_[2]->getPtr(),
                                             state_ptr_vector_[3]->getPtr(),
                                             state_ptr_vector_[4]->getPtr(),
                                             state_ptr_vector_[5]->getPtr(),
                                             state_ptr_vector_[6]->getPtr(),
                                             state_ptr_vector_[7]->getPtr(),
                                             state_ptr_vector_[8]->getPtr()});
        }
        case 10:
        {
            return std::vector<Scalar*>({state_ptr_vector_[0]->getPtr(),
                                             state_ptr_vector_[1]->getPtr(),
                                             state_ptr_vector_[2]->getPtr(),
                                             state_ptr_vector_[3]->getPtr(),
                                             state_ptr_vector_[4]->getPtr(),
                                             state_ptr_vector_[5]->getPtr(),
                                             state_ptr_vector_[6]->getPtr(),
                                             state_ptr_vector_[7]->getPtr(),
                                             state_ptr_vector_[8]->getPtr(),
                                             state_ptr_vector_[9]->getPtr()});
        }
    }

    return std::vector<Scalar*>(0); //Not going to happen
}

const std::vector<StateBlockPtr> ConstraintAnalytic::getStatePtrVector() const
{
    return state_ptr_vector_;
}

std::vector<unsigned int> ConstraintAnalytic::getStateSizes() const
{
    return state_block_sizes_vector_;
}

JacobianMethod ConstraintAnalytic::getJacobianMethod() const
{
    return JAC_ANALYTIC;
}

void ConstraintAnalytic::resizeVectors()
{
    for (unsigned int ii = 1; ii<state_ptr_vector_.size(); ii++)
    {
        if (state_ptr_vector_.at(ii) != nullptr)
            state_block_sizes_vector_.push_back(state_ptr_vector_.at(ii)->getSize());

        else
        {
            state_ptr_vector_.resize(ii);
            break;
        }
    }
}

} // namespace wolf

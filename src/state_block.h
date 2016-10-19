
#ifndef STATE_BLOCK_H_
#define STATE_BLOCK_H_

// Fwd references
namespace wolf{
class NodeBase;
class LocalParametrizationBase;
}

//Wolf includes
#include "wolf.h"

//std includes
#include <iostream>


namespace wolf {

/** \brief class StateBlock
 *
 * This class implements a state block for general nonlinear optimization. It offers the following functionality:
 *  - A state vector storing the state values.
 *  - A key to indicate whether the state is fixed or not.
 *     - Fixed state blocks are not estimated and treated by the estimator as fixed parameters.
 *     - Non-fixed state blocks are estimated.
 *  - A local parametrization useful for optimizing in the tangent space to the manifold.
 */
class StateBlock
{
    protected:
        NodeBaseWPtr node_ptr_; //< pointer to the wolf Node owning this StateBlock

        bool fixed_; ///< Key to indicate whether the state is fixed or not

        Eigen::VectorXs state_; ///< State vector storing the state values
        LocalParametrizationBase* local_param_ptr_; ///< Local parametrization useful for optimizing in the tangent space to the manifold
        
    public:
        /** \brief Constructor from size
         *
         * \param _size is state size
         * \param _fixed Indicates this state is not estimated and thus acts as a fixed parameter
         * \param _local_param_ptr pointer to the local parametrization for the block
         */
        StateBlock(const unsigned int _size, bool _fixed = false, LocalParametrizationBase* _local_param_ptr = nullptr);

        /** \brief Constructor from vector
         * 
         * \param _state is state vector
         * \param _fixed Indicates this state is not estimated and thus acts as a fixed parameter
         * \param _local_param_ptr pointer to the local parametrization for the block
         **/
        StateBlock(const Eigen::VectorXs _state, bool _fixed = false, LocalParametrizationBase* _local_param_ptr = nullptr);
        
        /** \brief Destructor
         **/
        virtual ~StateBlock();

        /** \brief Returns the pointer to the first element of the state
         **/
        Scalar* getPtr();
        
        /** \brief Returns the state vector
         **/
        const Eigen::VectorXs& getVector() const;

        /** \brief Sets the state vector
         **/
        void setVector(const Eigen::VectorXs& _state);

        /** \brief Returns the state size
         **/
        unsigned int getSize() const;

        /** \brief Returns if the state is fixed (not estimated)
         **/
        bool isFixed() const;

        /** \brief Sets the state as fixed
         **/
        void fix();

        /** \brief Sets the state as estimated
         **/
        void unfix();

        bool hasLocalParametrization();

        LocalParametrizationBase* getLocalParametrizationPtr();

        void setLocalParametrizationPtr(LocalParametrizationBase* _local_param);

        void removeLocalParametrization();

};

} // namespace wolf

// IMPLEMENTATION
#include "local_parametrization_base.h"
namespace wolf {

inline StateBlock::StateBlock(const Eigen::VectorXs _state, bool _fixed, LocalParametrizationBase* _local_param_ptr) :
        node_ptr_(), // nullptr
        fixed_(_fixed),
        state_(_state),
        local_param_ptr_(_local_param_ptr)
{
    std::cout << "constructed           +sb" << std::endl;
}

inline StateBlock::StateBlock(const unsigned int _size, bool _fixed, LocalParametrizationBase* _local_param_ptr) :
        node_ptr_(), // nullptr
        fixed_(_fixed),
        state_(Eigen::VectorXs::Zero(_size)),
        local_param_ptr_(_local_param_ptr)
{
    //
    std::cout << "constructed           +sb" << std::endl;
}

inline StateBlock::~StateBlock()
{
    // We prefer to delete the local_param_ptr_ pointer here in the base class,
    // because sometimes this local_param_ptr_ is set by a set() method in this same base class,
    // thus not in the constructor of any derived class.
    if (local_param_ptr_ != nullptr)
        delete local_param_ptr_;
    std::cout << "destructed            -sb" << std::endl;
}

inline Scalar* StateBlock::getPtr()
{
    return state_.data();
}

inline const Eigen::VectorXs& StateBlock::getVector() const
{
    return state_;
}

inline void StateBlock::setVector(const Eigen::VectorXs& _state)
{
    assert(_state.size() == state_.size());
    state_ = _state;
}

inline unsigned int StateBlock::getSize() const
{
    return state_.size();
}

inline bool StateBlock::isFixed() const
{
    return fixed_;
}

inline void StateBlock::fix()
{
    fixed_ = true;
}

inline void StateBlock::unfix()
{
    fixed_ = false;
}

inline bool StateBlock::hasLocalParametrization()
{
    return (local_param_ptr_ != nullptr);
}

inline LocalParametrizationBase* StateBlock::getLocalParametrizationPtr()
{
    return local_param_ptr_;
}

inline void StateBlock::removeLocalParametrization()
{
	assert(local_param_ptr_ != nullptr && "state block without local parametrization");
	delete local_param_ptr_;
    local_param_ptr_ = nullptr;
}

inline void StateBlock::setLocalParametrizationPtr(LocalParametrizationBase* _local_param)
{
	assert(_local_param != nullptr && "setting a null local parametrization");
    local_param_ptr_ = _local_param;
}

} // namespace wolf

#endif

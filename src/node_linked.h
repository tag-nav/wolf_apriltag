/** \file node_linked.h
 *
 * \author Joan Sola
 *
 * This file defines the basic template class for linked nodes.
 *
 * As a definition of a template class of a significant complexity,
 * this file is organized with member definitions after the class declaration.
 * This has some advantages:
 * - Functions can be inlined.
 * - typedefs within the class can be used straightforwardly in definitions.
 * - Circular references are minimized.
 *
 * and some implications:
 * - After the class declaration, we need to include all headers of the classes acting as the template parameters.
 * - This makes circular references again a risk.
 */

#ifndef NODE_LINKED_H_
#define NODE_LINKED_H_

//wolf includes
#include "node_base.h"
#include "wolf.h"

namespace wolf
{

class Problem;

/** \brief Linked node element in the Wolf Tree
 * 
 * \param UpperType the type of node one level up in the Wolf tree.
 * \param LowerType the type of node one level down in the Wolf tree.
 *
 * Inherit from this class to implement a node element to be placed somewhere in the Wolf Tree.
 * A linked node has seven main data members:
 *  - An unique ID to identify it over the whole Wolf Tree (inherited from NodeBase)
 *  - An enum indicating the node location in the tree (see NodeLocation enum in wolf.h)
 *  - down_node_list_: A list of pointers to derived node objects, specified by the template parameter LowerType.
 *  - up_node_: A pointer to a derived node object, specified by the template parameter UpperType.
 *  - A unique class name, inherited from NodeBase, strictly within this range of possibilities:
 *    - "UNDEFINED"     : used for NodeTerminus
 *    - "PROBLEM"       : for Problem and all derived classes -- beware: Problem is at this time deriving NodeBase.
 *    - "HARDWARE"      : for HardwareBase and all derived classes
 *    - "SENSOR"        : for SensorBase and all derived classes
 *    - "PROCESSOR"     : for ProcessorBase and all derived classes
 *    - "TRAJECTORY"    : for TrajectoryBase and all derived classes
 *    - "FRAME"         : for FrameBase and all derived classes
 *    - "CAPTURE"       : for CaptureBase and all derived classes
 *    - "FEATURE"       : for FeatureBase and all derived classes
 *    - "CONSTRAINT"    : for ConstraintBase and all derived classes
 *    - "MAP"           : for MapBase and all derived classes
 *    - "LANDMARK"      : for LandmarkBase and all derived classes
 *  - A unique type label, inherited from NodeBase, which is a subclass of the above. A few  examples are:
 *    - "CAMERA"        : for the class SensorCamera
 *    - "LASER 2D"      : for the class SensorLaser2D
 *    - "POINT 3D"      : for the class LandmarkPoint3D
 *    - "PROCESSOR LASER 2D" : for the class ProcessorLaser2D
 *    Please refer to each base class derived from NodeLinked (those listed just above) for a list of type labels.
 *  - A name, inherited from NodeBase, defined in each application, which is specific of each object. A few examples follow:
 *    - "Front-left Camera"
 *    - "Left-side LIDAR 2D"
 *    - "Lidar 2D processor"
 */
template<class UpperType, class LowerType>
class NodeLinked : public NodeBase
{
    public:
        typedef UpperType* UpperNodePtr;
        typedef LowerType* LowerNodePtr;

    protected:
        typedef std::list<LowerNodePtr> LowerNodeList;
        typedef typename LowerNodeList::iterator LowerNodeIter;

    protected:
        NodeLocation location_; ///< Indicates whether this node is a TOP, MIDDLE or BOOTOM node
        UpperNodePtr up_node_ptr_; ///< Pointer to upper node
        LowerNodeList down_node_list_; ///< A list of pointers to lower nodes
        bool is_deleting_; ///< This node is being deleted.

    private:
        ProblemPtr problem_ptr_;

    public:

        /** \brief Constructor without specify up node
         */
        NodeLinked(const NodeLocation _loc, const std::string& _class, const std::string& _type = "Base", const std::string& _name = "");

        /** \brief Default destructor (not recommended)
         *
         * Default destructor (please use destruct() instead of delete for guaranteeing the wolf tree integrity)
         */
        virtual ~NodeLinked();

        /** \brief Wolf destructor
         *
         * Wolf destructor (please use it instead of delete for guaranteeing the wolf tree integrity)
         */
        virtual void destruct() final;

        /** \brief Checks if the destructor has been called already
         */
        virtual const bool isDeleting() const;

        /** \brief Checks if node is on the top of Wolf tree
         */
        bool isTop() const;

        /** \brief Checks if node is at the bottom of Wolf tree
         */
        bool isBottom() const;

        /** \brief Sets link to up node
         */
        void linkToUpperNode(UpperNodePtr _pptr);

        /** \brief Clears link to up node
         */
        void unlinkFromUpperNode();

        /** \brief Access the pointer to parent.
         *
         * Access the pointer to parent.
         * Throw if parent nullptr.
         */
        const UpperNodePtr upperNodePtr() const;

        /** \brief Gets a reference to parent.
         *
         * Throw if parent is nullptr.
         */
        const UpperType& upperNode() const;

        /** \brief Adds a down node 
         */
        void addDownNode(LowerNodePtr _ptr);

        /** \brief Append a list of new down nodes
         * The provided list is emptied after this operation
         */
        void addDownNodeList(LowerNodeList& _new_down_node_list);

        /** \brief Inserts a down node in an specific position
         */
        inline void insertDownNode(LowerNodePtr _ptr, LowerNodeIter _place);

        /** \brief Move a down node to an specific position
         */
        inline void moveDownNode(LowerNodePtr _ptr, LowerNodeIter _place);

        /** \brief Gets a reference to down node list
         */
        LowerNodeList& downNodeList() const;

        /** \brief Gets a pointer to down node list
         */
        LowerNodeList* getDownNodeListPtr();

        /** \brief Removes a down node from list, given an iterator
         *
         * @param _iter an iterator to the particular down node in the list that will be removed
         */
        void removeDownNode(const LowerNodeIter& _iter);

        /** \brief Removes a down node from list, given a pointer
         *
         * @param _ptr a pointer to the particular down node in the list that will be removed
         */
        void removeDownNode(const LowerNodePtr _ptr);

        /** \brief Removes a down node from the list, given a node id
         *
         * @param _id node id of the node that will be removed
         */
        void removeDownNode(const unsigned int _id);

        /** \brief Removes a down node from list, given an iterator
         *
         * @param _iter an iterator to the particular down node in the list that will be removed
         */
        void unlinkDownNode(const LowerNodeIter& _iter);

        /** \brief Removes a down node from list, given a pointer
         *
         * @param _ptr a pointer to the particular down node in the list that will be removed
         */
        void unlinkDownNode(const LowerNodePtr _ptr);

        /** \brief Removes a down node from the list, given a node id
         *
         * @param _id node id of the node that will be removed
         */
        void unlinkDownNode(const unsigned int _id);

        /** \brief Gets a pointer to the tree top node - direct method
         **/
        ProblemPtr getProblem();

        /** \brief Gets a pointer to the tree top node - recursive method
         **/
        ProblemPtr getTop();

};


} // namespace wolf

//////////////////////////////////////////
//          IMPLEMENTATION
//////////////////////////////////////////

// Include header files of forward-declared classes derived from NodeLinked -- this avoids loop dependencies
// See this evil ugly solution improved in note 8) of http://www.cplusplus.com/forum/articles/10627/
// This implies including here ALL the base classes in the Wolf tree!
#include "problem.h"
#include "hardware_base.h"
#include "sensor_base.h"
#include "processor_base.h"
#include "trajectory_base.h"
#include "frame_base.h"
#include "capture_base.h"
#include "feature_base.h"
#include "constraint_base.h"
#include "map_base.h"
#include "landmark_base.h"
#include "node_terminus.h"

namespace wolf
{

template<class UpperType, class LowerType>
NodeLinked<UpperType, LowerType>::NodeLinked(const NodeLocation _loc, const std::string& _class, const std::string& _type, const std::string& _name) :
        NodeBase(_class, _type, _name), //
        location_(_loc), //
        up_node_ptr_(nullptr), //
        down_node_list_(), //
        is_deleting_(false),
        problem_ptr_(nullptr)
{
        //
}

template<class UpperType, class LowerType>
NodeLinked<UpperType, LowerType>::~NodeLinked()
{
    //std::cout << "deleting Nodelinked " << node_id_ << " down_node_list_.size() " << down_node_list_.size() << std::endl;
    is_deleting_ = true;

    while (!down_node_list_.empty())
    {
        delete down_node_list_.front();
        down_node_list_.pop_front();
    }
}

template<class UpperType, class LowerType>
void NodeLinked<UpperType, LowerType>::destruct()
{
    //std::cout << "destruct() " << node_id_ << " down_node_list_.size() " << down_node_list_.size() << std::endl;
    if (!is_deleting_)
    {
        if (up_node_ptr_ != nullptr && !up_node_ptr_->isTop())
        {
            //std::cout << "upper node is not WolfProblem " << std::endl;
            up_node_ptr_->removeDownNode((typename UpperType::LowerNodePtr)(this));
        }
        else
        {
            //std::cout << "upper node is WolfProblem or nullptr" << std::endl;
            delete this;
        }
    }
}

template<class UpperType, class LowerType>
inline const bool NodeLinked<UpperType, LowerType>::isDeleting() const
{
    return is_deleting_;
}

template<class UpperType, class LowerType>
inline bool NodeLinked<UpperType, LowerType>::isTop() const
{
    return (location_ == TOP);
}

template<class UpperType, class LowerType>
inline bool NodeLinked<UpperType, LowerType>::isBottom() const
{
    return (location_ == BOTTOM);
}

template<class UpperType, class LowerType>
inline void NodeLinked<UpperType, LowerType>::linkToUpperNode(UpperNodePtr _pptr)
{
    if (isTop())
        up_node_ptr_ = nullptr;
    else
        up_node_ptr_ = _pptr;
}

template<class UpperType, class LowerType>
inline void NodeLinked<UpperType, LowerType>::unlinkFromUpperNode()
{
    up_node_ptr_ = nullptr;
}

template<class UpperType, class LowerType>
inline const typename NodeLinked<UpperType, LowerType>::UpperNodePtr NodeLinked<UpperType, LowerType>::upperNodePtr() const
{
    return up_node_ptr_;
}

template<class UpperType, class LowerType>
inline const UpperType& NodeLinked<UpperType, LowerType>::upperNode() const
{
    assert(up_node_ptr_ != nullptr);
    return *up_node_ptr_;
}

template<class UpperType, class LowerType>
inline void NodeLinked<UpperType, LowerType>::addDownNode(LowerNodePtr _ptr)
{
    assert(!isBottom() && "Trying to add a down node to a bottom node");
    down_node_list_.push_back(_ptr);
    _ptr->linkToUpperNode((typename LowerType::UpperNodePtr)(this));
}

template<class UpperType, class LowerType>
void NodeLinked<UpperType, LowerType>::addDownNodeList(LowerNodeList& _new_down_node_list)
{
    assert(!isBottom() && "Trying to add a down node to a bottom node");
    for (auto new_down_node : _new_down_node_list)
        new_down_node->linkToUpperNode((typename LowerType::UpperNodePtr)(this));
    down_node_list_.splice(down_node_list_.end(), _new_down_node_list);
}

template<class UpperType, class LowerType>
inline void NodeLinked<UpperType, LowerType>::insertDownNode(LowerNodePtr _ptr, LowerNodeIter _place)
{
    assert(!isBottom() && "Trying to insert a down node to a bottom node");
    down_node_list_.insert(_place, _ptr);
    _ptr->linkToUpperNode((typename LowerType::UpperNodePtr)(this));
}

template<class UpperType, class LowerType>
inline void NodeLinked<UpperType, LowerType>::moveDownNode(LowerNodePtr _ptr, LowerNodeIter _place)
{
    assert(!isBottom() && "Trying to move a down node to a bottom node");
    if (*_place != _ptr)
    {
        down_node_list_.remove(_ptr);
        down_node_list_.insert(_place, _ptr);
    }
}

template<class UpperType, class LowerType>
inline typename NodeLinked<UpperType, LowerType>::LowerNodeList& NodeLinked<UpperType, LowerType>::downNodeList() const
{
    return down_node_list_;
}

template<class UpperType, class LowerType>
inline typename NodeLinked<UpperType, LowerType>::LowerNodeList* NodeLinked<UpperType, LowerType>::getDownNodeListPtr()
{
    return &down_node_list_;
}

template<class UpperType, class LowerType>
inline void NodeLinked<UpperType, LowerType>::removeDownNode(const unsigned int _id)
{
    for (auto iter = down_node_list_.begin(); iter != down_node_list_.end(); ++iter)
    {
        if ((*iter)->nodeId() == _id)
        {
            removeDownNode(iter);
            break; //avoid comparison of iter and list.end(), otherwise Valgrind claimed
        }
    }
}

template<class UpperType, class LowerType>
inline void NodeLinked<UpperType, LowerType>::removeDownNode(const LowerNodePtr _ptr)
{
    down_node_list_.remove(_ptr);
    delete _ptr;
}

template<class UpperType, class LowerType>
inline void NodeLinked<UpperType, LowerType>::removeDownNode(const LowerNodeIter& _iter)
{
    //(*_iter)->unlinkFromUpperNode();
    down_node_list_.erase(_iter);
    delete *_iter;
}

template<class UpperType, class LowerType>
inline void NodeLinked<UpperType, LowerType>::unlinkDownNode(const unsigned int _id)
{
    for (auto iter = down_node_list_.begin(); iter != down_node_list_.end(); ++iter)
    {
        if ((*iter)->nodeId() == _id)
        {
            unlinkDownNode(iter);
            break; //avoid comparison of iter and list.end(), otherwise Valgrind claimed
        }
    }
}

template<class UpperType, class LowerType>
inline void NodeLinked<UpperType, LowerType>::unlinkDownNode(const LowerNodePtr _ptr)
{
    _ptr->unlinkFromUpperNode();
    down_node_list_.remove(_ptr);
}

template<class UpperType, class LowerType>
inline void NodeLinked<UpperType, LowerType>::unlinkDownNode(const LowerNodeIter& _iter)
{
    (*_iter)->unlinkFromUpperNode();
    down_node_list_.erase(_iter);
}

template<class UpperType, class LowerType>
ProblemPtr NodeLinked<UpperType, LowerType>::getProblem()
{
    if (problem_ptr_ == nullptr)
        problem_ptr_ = getTop();
    return problem_ptr_;
}

template<class UpperType, class LowerType>
inline ProblemPtr NodeLinked<UpperType, LowerType>::getTop()
{
    if (up_node_ptr_ != nullptr)
        return up_node_ptr_->getTop();

    return nullptr;
}


} // namespace wolf

#endif /* NODE_LINKED_H_ */

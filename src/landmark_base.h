#ifndef LANDMARK_BASE_H_
#define LANDMARK_BASE_H_

// Fwd references
namespace wolf{
class MapBase;
class NodeTerminus;
class StateBlock;
}

//Wolf includes
#include "wolf.h"
#include "node_base.h"
#include "time_stamp.h"

//std includes

// yaml
#include "yaml-cpp/yaml.h"

namespace wolf {


//class LandmarkParamsBase {}; ///< class for landmark parameters. Derive it to define your parameters.

// TODO: add descriptor as a StateBlock -> Could be estimated or not. Aperture could be one case of "descriptor"that can be estimated or not
// TODO: init and end Time stamps

//class LandmarkBase
class LandmarkBase : public NodeBase // NodeConstrained<MapBase, NodeTerminus>
{
    private:
        ProblemPtr problem_ptr_;
        MapBasePtr map_ptr_;
        ConstraintBaseList constrained_by_list_;
        static unsigned int landmark_id_count_;
        
    protected:
        unsigned int landmark_id_; ///< landmark unique id
        LandmarkType type_id_;     ///< type of landmark. (types defined at wolf.h)
        LandmarkStatus status_; ///< status of the landmark. (types defined at wolf.h)
        TimeStamp stamp_;       ///< stamp of the creation of the landmark (and stamp of destruction when status is LANDMARK_OLD)
        StateBlock* p_ptr_;     ///< Position state block pointer
        StateBlock* o_ptr_;     ///< Orientation state block pointer
        Eigen::VectorXs descriptor_;    //TODO: agree? JS: No: It is not general enough as descriptor to be in LmkBase.

    public:

        /** \brief Constructor with type, time stamp and the position state pointer (optional orientation state pointer)
         *
         * Constructor with type, and state pointer
         * \param _tp indicates landmark type.(types defined at wolf.h)
         * \param _p_ptr StateBlock pointer to the position
         * \param _o_ptr StateBlock pointer to the orientation (default: nullptr)
         *
         **/
        LandmarkBase(const LandmarkType & _tp, const std::string& _type, StateBlock* _p_ptr, StateBlock* _o_ptr = nullptr);

        /** \brief Default destructor (not recommended)
         *
         * Default destructor (please use destruct() instead of delete for guaranteeing the wolf tree integrity)
         *
         **/
        virtual ~LandmarkBase();

        void destruct();

        /** \brief Returns landmark_id_, the landmark unique id
         **/
        unsigned int id();
        void setId(unsigned int _id);

        /** \brief Sets the Landmark status
         **/
        void setStatus(LandmarkStatus _st);

        /** \brief Sets the Landmark status to fixed
         **/
        void fix();

        /** \brief Sets the Landmark status to estimated
         **/
        void unfix();

        /** \brief Remove the given constraint from the list. 
         *  If list becomes empty, deletes this object by calling destruct()
         **/
        void removeConstrainedBy(ConstraintBase* _ctr_ptr);

        /** \brief Adds all stateBlocks of the frame to the wolfProblem list of new stateBlocks
         **/
        virtual void registerNewStateBlocks();

        /** \brief Gets the position state block pointer
         **/
        StateBlock* getPPtr() const;

        /** \brief Gets the orientation state block pointer
         **/
        StateBlock* getOPtr() const;

        /** \brief Gets a vector of all state blocks pointers
         **/
        virtual std::vector<StateBlock*> getStateBlockVector() const;

        /** \brief Sets the position state block pointer
         **/
        void setPPtr(StateBlock* _st_ptr);

        /** \brief Sets the orientation state block pointer
         **/
        void setOPtr(StateBlock* _st_ptr);

        /** \brief Sets the descriptor
         **/
        void setDescriptor(const Eigen::VectorXs& _descriptor);

        /** \brief Gets the descriptor
         **/
        const Eigen::VectorXs& getDescriptor() const;        
        
        /** \brief Returns _ii component of descriptor vector
         **/
        Scalar getDescriptor(unsigned int _ii) const;

        /** \brief Return the type of the landmark
         **/
        const LandmarkType getTypeId() const;

        virtual YAML::Node saveToYaml() const;

        void addConstrainedBy(ConstraintBase* _ctr_ptr)
        {
            constrained_by_list_.push_back(_ctr_ptr);
        }
        unsigned int getHits() const
        {
            return constrained_by_list_.size();
        }
        ConstraintBaseList* getConstrainedByListPtr()
        {
            return &constrained_by_list_;
        }


        void setMapPtr(MapBasePtr _map_ptr){map_ptr_ = _map_ptr;}
        Problem* getProblem(){return problem_ptr_;}
        void setProblem(Problem* _prob_ptr){problem_ptr_ = _prob_ptr;}


};

}

#include "map_base.h"

namespace wolf{

inline unsigned int LandmarkBase::id()
{
    return landmark_id_;
}

inline void LandmarkBase::setId(unsigned int _id)
{
    landmark_id_ = _id;
    if (_id > landmark_id_count_)
        landmark_id_count_ = _id;
}

inline void LandmarkBase::fix()
{
    //std::cout << "Fixing frame " << nodeId() << std::endl;
    this->setStatus(LANDMARK_FIXED);
}

inline void LandmarkBase::unfix()
{
    //std::cout << "Unfixing frame " << nodeId() << std::endl;
    this->setStatus(LANDMARK_ESTIMATED);
}

inline void LandmarkBase::removeConstrainedBy(ConstraintBase* _ctr_ptr)
{
    constrained_by_list_.remove(_ctr_ptr);
//    NodeConstrained::removeConstrainedBy(_ctr_ptr);
    if (constrained_by_list_.empty())
//    if (getConstrainedByListPtr()->empty())
        this->destruct();
}

inline StateBlock* LandmarkBase::getPPtr() const
{
    return p_ptr_;
}

inline StateBlock* LandmarkBase::getOPtr() const
{
    return o_ptr_;
}

inline std::vector<StateBlock*> LandmarkBase::getStateBlockVector() const
{
    if (p_ptr_ == nullptr && o_ptr_ == nullptr)
        return std::vector<StateBlock*>(0);

    if (p_ptr_ == nullptr)
        return std::vector<StateBlock*>( {o_ptr_});

    if (o_ptr_ == nullptr)
        return std::vector<StateBlock*>( {p_ptr_});

    return std::vector<StateBlock*>( {p_ptr_, o_ptr_});
}

inline void LandmarkBase::setPPtr(StateBlock* _st_ptr)
{
    p_ptr_ = _st_ptr;
}

inline void LandmarkBase::setOPtr(StateBlock* _st_ptr)
{
    o_ptr_ = _st_ptr;
}

inline void LandmarkBase::setDescriptor(const Eigen::VectorXs& _descriptor)
{
    descriptor_ = _descriptor;
}

inline Scalar LandmarkBase::getDescriptor(unsigned int _ii) const
{
    assert(_ii < descriptor_.size() && "LandmarkBase::getDescriptor: bad index");
    return descriptor_(_ii);
}

inline const Eigen::VectorXs& LandmarkBase::getDescriptor() const
{
    return descriptor_;
}

inline void LandmarkBase::destruct()
{
    // TODO implement something
    if (!is_deleting_)
    {
        if (map_ptr_ != nullptr) // && !up_node_ptr_->isTop())
        {
            //std::cout << "upper node is not WolfProblem " << std::endl;
            map_ptr_->removeLandmark(this);
        }
        else
        {
            //std::cout << "upper node is WolfProblem or nullptr" << std::endl;
            delete this;
        }
    }
}

inline const LandmarkType LandmarkBase::getTypeId() const
{
    return type_id_;
}

} // namespace wolf
#endif

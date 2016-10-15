#ifndef SENSOR_BASE_H_
#define SENSOR_BASE_H_

// Fwd refs
namespace wolf{
class HardwareBase;
class ProcessorBase;
class StateBlock;
}

//Wolf includes
#include "wolf.h"
#include "node_base.h"

//std includes

namespace wolf {


/** \brief base struct for intrinsic sensor parameters
 *
 * Derive from this struct to create structs of sensor intrinsic parameters.
 */
struct IntrinsicsBase
{
        std::string type;
        std::string name;
};

class SensorBase : public NodeBase, public std::enable_shared_from_this<SensorBase>
{
    private:
        HardwareBasePtr hardware_ptr_;
        ProcessorBaseList processor_list_;

        static unsigned int sensor_id_count_; ///< Object counter (acts as simple ID factory)

    protected:
        unsigned int sensor_id_;   // sensor ID
        SensorType type_id_;       // the type of sensor. See wolf.h for a list of all sensor types.
        StateBlock* p_ptr_;		// sensor position state block pointer
        StateBlock* o_ptr_; 	// sensor orientation state block pointer

        /** \brief intrinsic parameters.
         * Use it if desired. By using this StateBlock, WOLF will be able to auto-calibrate these parameters.
         * To do so, just unfix() it. After the calibration process, you can fix() it again if desired.
         *
         * (Note: Many other intrinsic parameters can be stored as members of the classes derived from this.
         * We recommend you use a struct for this purpose if the number of intrinsic parameters is large.)
         */
        StateBlock* intrinsic_ptr_;

        bool extrinsic_dynamic_;// extrinsic parameters vary with time? If so, they will be taken from the Capture nodes. TODO: Not Yet Implemented.

        Eigen::VectorXs noise_std_; // std of sensor noise
        Eigen::MatrixXs noise_cov_; // cov matrix of noise

    public:

        /** \brief Constructor with noise size
         *
         * Constructor with parameter vector
         * \param _tp Type of the sensor  (types defined at wolf.h)
         * \param _p_ptr StateBlock pointer to the sensor position
         * \param _o_ptr StateBlock pointer to the sensor orientation
         * \param _intr_ptr StateBlock pointer to the sensor intrinsic params that might be estimated (if unfixed).
         * \param _noise_size dimension of the noise term
         * \param _extr_dyn Flag indicating if extrinsics are dynamic (moving) or static (not moving)
         *
         **/
        SensorBase(const SensorType & _tp, const std::string& _type, StateBlock* _p_ptr, StateBlock* _o_ptr, StateBlock* _intr_ptr, const unsigned int _noise_size, const bool _extr_dyn = false);

        /** \brief Constructor with noise std vector
         *
         * Constructor with parameter vector
         * \param _tp Type of the sensor  (types defined at wolf.h)
         * \param _p_ptr StateBlock pointer to the sensor position
         * \param _o_ptr StateBlock pointer to the sensor orientation
         * \param _intr_ptr StateBlock pointer to the sensor intrinsic params that might be estimated (if unfixed).
         * \param _noise_std standard deviations of the noise term
         * \param _extr_dyn Flag indicating if extrinsics are dynamic (moving) or static (not moving)
         *
         **/
        SensorBase(const SensorType & _tp, const std::string& _type, StateBlock* _p_ptr, StateBlock* _o_ptr, StateBlock* _intr_ptr, const Eigen::VectorXs & _noise_std, const bool _extr_dyn = false);


        /** \brief Default destructor (not recommended)
         *
         * Default destructor (please use destruct() instead of delete for guaranteeing the wolf tree integrity)
         *
         **/
        virtual ~SensorBase();
//        void destruct();
        void remove();

        unsigned int id();

        SensorType typeId();

        ProcessorBasePtr addProcessor(ProcessorBasePtr _proc_ptr);
        void removeProcessor(ProcessorBasePtr _prc_ptr);

        ProcessorBaseList* getProcessorListPtr();

        StateBlock* getPPtr() const;

        StateBlock* getOPtr() const;

        StateBlock* getIntrinsicPtr() const;

        void fix();

        void unfix();

        /** \brief Adds all stateBlocks of the sensor to the wolfProblem list of new stateBlocks
         **/
        virtual void registerNewStateBlocks();

        /**
         * Check if sensor is dynamic
         */
        bool isExtrinsicDynamic();

        void setNoise(const Eigen::VectorXs & _noise_std);

        Eigen::VectorXs getNoiseStd();

        Eigen::MatrixXs getNoiseCov();

        ProblemPtr getProblem();

};

}

#include "processor_base.h"
#include "hardware_base.h"

namespace wolf{

inline wolf::ProblemPtr SensorBase::getProblem()
{
    if (problem_ptr_ == nullptr && hardware_ptr_ != nullptr)
        problem_ptr_ = hardware_ptr_->getProblem();
    return problem_ptr_;
}

inline void SensorBase::removeProcessor(ProcessorBasePtr _prc_ptr)
{
    processor_list_.remove(_prc_ptr);
    delete _prc_ptr;
}

inline unsigned int SensorBase::id()
{
    return sensor_id_;
}

inline wolf::SensorType SensorBase::typeId()
{
    return type_id_;
}

inline ProcessorBasePtr SensorBase::addProcessor(ProcessorBasePtr _proc_ptr)
{
    processor_list_.push_back(_proc_ptr);
    //    _proc_ptr->setSensorPtr(this); // TODO remove line
    _proc_ptr->setSensorPtr(shared_from_this());
    _proc_ptr->setProblem(getProblem());
    return _proc_ptr;
}

inline ProcessorBaseList* SensorBase::getProcessorListPtr()
{
    return & processor_list_;
}

inline StateBlock* SensorBase::getPPtr() const
{
    return p_ptr_;
}

inline StateBlock* SensorBase::getOPtr() const
{
    return o_ptr_;
}

inline StateBlock* SensorBase::getIntrinsicPtr() const
{
    return intrinsic_ptr_;
}

inline bool SensorBase::isExtrinsicDynamic()
{
    return extrinsic_dynamic_;
}

inline Eigen::VectorXs SensorBase::getNoiseStd()
{
    return noise_std_;
}

//inline void SensorBase::destruct()
//{
//    if (!is_removing_)
//    {
//        if (hardware_ptr_ != nullptr)
//            //            hardware_ptr_->removeSensor(this); // TODO remove line
//            hardware_ptr_->removeSensor(shared_from_this());
//        else
//            delete this;
//    }
//}

inline Eigen::MatrixXs SensorBase::getNoiseCov()
{
    return noise_cov_;
}

} // namespace wolf

#endif

// wolf includes
#include "base/problem.h"
#include "base/hardware_base.h"
#include "base/trajectory_base.h"
#include "base/map_base.h"
#include "base/sensor/sensor_base.h"
#include "base/processor/processor_motion.h"
#include "base/processor/processor_tracker.h"
#include "base/capture/capture_pose.h"
#include "base/factor/factor_base.h"
#include "base/sensor/sensor_factory.h"
#include "base/processor/processor_factory.h"
#include "base/state_block.h"


// IRI libs includes

// C++ includes
#include <algorithm>

namespace wolf
{

// unnamed namespace used for helper functions local to this file.
namespace
{
std::string uppercase(std::string s) {for (auto & c: s) c = std::toupper(c); return s;}
}

Problem::Problem(const std::string& _frame_structure) :
        hardware_ptr_(std::make_shared<HardwareBase>()),
        trajectory_ptr_(std::make_shared<TrajectoryBase>(_frame_structure)),
        map_ptr_(std::make_shared<MapBase>()),
        processor_motion_ptr_(),
        prior_is_set_(false)
{
    if (_frame_structure == "PO 2D")
    {
        state_size_ = 3;
        state_cov_size_ = 3;
        dim_ = 2;
    }

    else if (_frame_structure == "PO 3D")
    {
        state_size_ = 7;
        state_cov_size_ = 6;
        dim_ = 3;
    }
    else if (_frame_structure == "POV 3D")
    {
        state_size_ = 10;
        state_cov_size_ = 9;
        dim_ = 3;
    }
    else std::runtime_error(
            "Problem::Problem(): Unknown frame structure. Add appropriate frame structure to the switch statement.");

}

void Problem::setup()
{
    hardware_ptr_  -> setProblem(shared_from_this());
    trajectory_ptr_-> setProblem(shared_from_this());
    map_ptr_       -> setProblem(shared_from_this());
}

ProblemPtr Problem::create(const std::string& _frame_structure)
{
    ProblemPtr p(new Problem(_frame_structure)); // We use `new` and not `make_shared` since the Problem constructor is private and cannot be passed to `make_shared`.
    p->setup();
    return p->shared_from_this();
}

Problem::~Problem()
{
    //    WOLF_DEBUG("destructed -P");
}

void Problem::addSensor(SensorBasePtr _sen_ptr)
{
    getHardware()->addSensor(_sen_ptr);
}

SensorBasePtr Problem::installSensor(const std::string& _sen_type, //
                                     const std::string& _unique_sensor_name, //
                                     const Eigen::VectorXs& _extrinsics, //
                                     IntrinsicsBasePtr _intrinsics)
{
    SensorBasePtr sen_ptr = SensorFactory::get().create(uppercase(_sen_type), _unique_sensor_name, _extrinsics, _intrinsics);
    addSensor(sen_ptr);
    return sen_ptr;
}

SensorBasePtr Problem::installSensor(const std::string& _sen_type, //
                                     const std::string& _unique_sensor_name, //
                                     const Eigen::VectorXs& _extrinsics, //
                                     const std::string& _intrinsics_filename)
{

    if (_intrinsics_filename != "")
    {
        assert(file_exists(_intrinsics_filename) && "Cannot install sensor: intrinsics' YAML file does not exist.");
        IntrinsicsBasePtr intr_ptr = IntrinsicsFactory::get().create(_sen_type, _intrinsics_filename);
        return installSensor(_sen_type, _unique_sensor_name, _extrinsics, intr_ptr);
    }
    else
        return installSensor(_sen_type, _unique_sensor_name, _extrinsics, IntrinsicsBasePtr());

}

ProcessorBasePtr Problem::installProcessor(const std::string& _prc_type, //
                                           const std::string& _unique_processor_name, //
                                           SensorBasePtr _corresponding_sensor_ptr, //
                                           ProcessorParamsBasePtr _prc_params)
{
    if (_corresponding_sensor_ptr == nullptr)
    {
      WOLF_ERROR("Cannot install processor '", _unique_processor_name,
                 "' since the associated sensor does not exist !");
      return ProcessorBasePtr();
    }

    ProcessorBasePtr prc_ptr = ProcessorFactory::get().create(uppercase(_prc_type), _unique_processor_name, _prc_params, _corresponding_sensor_ptr);
    prc_ptr->configure(_corresponding_sensor_ptr);
    _corresponding_sensor_ptr->addProcessor(prc_ptr);

    // setting the origin in all processor motion if origin already setted
    if (prc_ptr->isMotion() && prior_is_set_)
        (std::static_pointer_cast<ProcessorMotion>(prc_ptr))->setOrigin(getLastKeyFrame());

    // setting the main processor motion
    if (prc_ptr->isMotion() && processor_motion_ptr_ == nullptr)
        processor_motion_ptr_ = std::static_pointer_cast<ProcessorMotion>(prc_ptr);

    return prc_ptr;
}

ProcessorBasePtr Problem::installProcessor(const std::string& _prc_type, //
                                           const std::string& _unique_processor_name, //
                                           const std::string& _corresponding_sensor_name, //
                                           const std::string& _params_filename)
{
    SensorBasePtr sen_ptr = getSensor(_corresponding_sensor_name);
    if (sen_ptr == nullptr)
        throw std::runtime_error("Sensor not found. Cannot bind Processor.");
    if (_params_filename == "")
        return installProcessor(_prc_type, _unique_processor_name, sen_ptr, nullptr);
    else
    {
        assert(file_exists(_params_filename) && "Cannot install processor: parameters' YAML file does not exist.");
        ProcessorParamsBasePtr prc_params = ProcessorParamsFactory::get().create(_prc_type, _params_filename);
        return installProcessor(_prc_type, _unique_processor_name, sen_ptr, prc_params);
    }
}

SensorBasePtr Problem::getSensor(const std::string& _sensor_name)
{
    auto sen_it = std::find_if(getHardware()->getSensorList().begin(),
                               getHardware()->getSensorList().end(),
                               [&](SensorBasePtr sb)
                               {
                                   return sb->getName() == _sensor_name;
                               }); // lambda function for the find_if

    if (sen_it == getHardware()->getSensorList().end())
        return nullptr;

    return (*sen_it);
}

ProcessorMotionPtr Problem::setProcessorMotion(const std::string& _processor_name)
{
    for (auto sen : getHardware()->getSensorList()) // loop all sensors
    {
        auto prc_it = std::find_if(sen->getProcessorList().begin(), // find processor by its name
                                   sen->getProcessorList().end(),
                                   [&](ProcessorBasePtr prc)
                                   {
                                       return prc->getName() == _processor_name;
                                   }); // lambda function for the find_if

        if (prc_it != sen->getProcessorList().end())  // found something!
        {
            if ((*prc_it)->isMotion()) // found, and it's motion!
            {
                std::cout << "Found processor '" << _processor_name << "', of type Motion, and set as the main motion processor." << std::endl;
                processor_motion_ptr_ = std::static_pointer_cast<ProcessorMotion>(*prc_it);
                return processor_motion_ptr_;
            }
            else // found, but it's not motion!
            {
                std::cout << "Found processor '" << _processor_name << "', but not of type Motion!" << std::endl;
                return nullptr;
            }
        }
    }
    // nothing found!
    std::cout << "Processor '" << _processor_name << "' not found!" << std::endl;
    return nullptr;
}

void Problem::setProcessorMotion(ProcessorMotionPtr _processor_motion_ptr)
{
    processor_motion_ptr_ = _processor_motion_ptr;
}

void Problem::clearProcessorMotion()
{
    processor_motion_ptr_.reset();
}

FrameBasePtr Problem::emplaceFrame(const std::string& _frame_structure, //
                                   FrameType _frame_key_type, //
                                   const Eigen::VectorXs& _frame_state, //
                                   const TimeStamp& _time_stamp)
{
    FrameBasePtr frm = FrameFactory::get().create(_frame_structure, _frame_key_type, _time_stamp, _frame_state);
    trajectory_ptr_->addFrame(frm);
    return frm;
}

FrameBasePtr Problem::emplaceFrame(const std::string& _frame_structure, //
                                   FrameType _frame_key_type, //
                                   const TimeStamp& _time_stamp)
{
    return emplaceFrame(_frame_structure, _frame_key_type, getState(_time_stamp), _time_stamp);
}

FrameBasePtr Problem::emplaceFrame(FrameType _frame_key_type, //
                                   const Eigen::VectorXs& _frame_state, //
                                   const TimeStamp& _time_stamp)
{
    return emplaceFrame(trajectory_ptr_->getFrameStructure(), _frame_key_type, _frame_state, _time_stamp);
}

FrameBasePtr Problem::emplaceFrame(FrameType _frame_key_type, //
                                   const TimeStamp& _time_stamp)
{
    return emplaceFrame(trajectory_ptr_->getFrameStructure(), _frame_key_type, _time_stamp);
}

Eigen::VectorXs Problem::getCurrentState()
{
    Eigen::VectorXs state(getFrameStructureSize());
    getCurrentState(state);
    return state;
}

void Problem::getCurrentState(Eigen::VectorXs& state)
{
    assert(state.size() == getFrameStructureSize() && "Problem::getCurrentState: bad state size");

    if (processor_motion_ptr_ != nullptr)
        processor_motion_ptr_->getCurrentState(state);
    else if (trajectory_ptr_->getLastKeyFrame() != nullptr)
        trajectory_ptr_->getLastKeyFrame()->getState(state);
    else
        state = zeroState();
}

void Problem::getCurrentStateAndStamp(Eigen::VectorXs& state, TimeStamp& ts)
{
    assert(state.size() == getFrameStructureSize() && "Problem::getCurrentState: bad state size");

    if (processor_motion_ptr_ != nullptr)
    {
        processor_motion_ptr_->getCurrentState(state);
        processor_motion_ptr_->getCurrentTimeStamp(ts);
    }
    else if (trajectory_ptr_->getLastKeyFrame() != nullptr)
    {
        trajectory_ptr_->getLastKeyFrame()->getTimeStamp(ts);
        trajectory_ptr_->getLastKeyFrame()->getState(state);
    }
    else
        state = zeroState();
}

void Problem::getState(const TimeStamp& _ts, Eigen::VectorXs& state)
{
    assert(state.size() == getFrameStructureSize() && "Problem::getStateAtTimeStamp: bad state size");

    // try to get the state from processor_motion if any, otherwise...
    if (processor_motion_ptr_ == nullptr || !processor_motion_ptr_->getState(_ts, state))
    {
        FrameBasePtr closest_frame = trajectory_ptr_->closestKeyFrameToTimeStamp(_ts);
        if (closest_frame != nullptr)
            closest_frame->getState(state);
        else
            state = zeroState();
    }
}

Eigen::VectorXs Problem::getState(const TimeStamp& _ts)
{
    Eigen::VectorXs state(getFrameStructureSize());
    getState(_ts, state);
    return state;
}

SizeEigen Problem::getFrameStructureSize() const
{
    return state_size_;
}

void Problem::getFrameStructureSize(SizeEigen& _x_size, SizeEigen& _cov_size) const
{
    _x_size   = state_size_;
    _cov_size = state_cov_size_;
}

SizeEigen Problem::getDim() const
{
    return dim_;
}

Eigen::VectorXs Problem::zeroState()
{
    Eigen::VectorXs state = Eigen::VectorXs::Zero(getFrameStructureSize());

    // Set the quaternion identity for 3D states. Set only the real part to 1:
    if (trajectory_ptr_->getFrameStructure() == "PO 3D" ||
        trajectory_ptr_->getFrameStructure() == "POV 3D")
        state(6) = 1.0;

    return state;
}

bool Problem::permitKeyFrame(ProcessorBasePtr _processor_ptr)
{
    // This should implement a black list of processors that have forbidden keyframe creation
    // This decision is made at problem level, not at processor configuration level.
    // If you want to configure a processor for not creating keyframes, see Processor::voting_active_ and its accessors.

    // Currently allowing all processors to vote:
    return true;
}

void Problem::keyFrameCallback(FrameBasePtr _keyframe_ptr, ProcessorBasePtr _processor_ptr, const Scalar& _time_tolerance)
{
    if (_processor_ptr)
    {
        WOLF_DEBUG((_processor_ptr->isMotion() ? "PM " : "PT "), _processor_ptr->getName(), ": KF", _keyframe_ptr->id(), " Callback emitted with ts = ", _keyframe_ptr->getTimeStamp());
    }
    else
    {
        WOLF_DEBUG("External callback: KF", _keyframe_ptr->id(), " Callback emitted with ts = ", _keyframe_ptr->getTimeStamp());
    }

    for (auto sensor : hardware_ptr_->getSensorList())
        for (auto processor : sensor->getProcessorList())
            if (processor && (processor != _processor_ptr) )
                processor->keyFrameCallback(_keyframe_ptr, _time_tolerance);
}

LandmarkBasePtr Problem::addLandmark(LandmarkBasePtr _lmk_ptr)
{
    getMap()->addLandmark(_lmk_ptr);
    return _lmk_ptr;
}

void Problem::addLandmarkList(LandmarkBasePtrList& _lmk_list)
{
    getMap()->addLandmarkList(_lmk_list);
}

StateBlockPtr Problem::addStateBlock(StateBlockPtr _state_ptr)
{
    //std::cout << "Problem::addStateBlockPtr " << _state_ptr.get() << std::endl;
    if(std::find(state_block_list_.begin(),state_block_list_.end(),_state_ptr) != state_block_list_.end())
    {
        WOLF_WARN("Adding a state block that has already been added");
        return _state_ptr;
    }

    // add the state unit to the list
    state_block_list_.push_back(_state_ptr);

    // Add add notification
    // Check if there is already a notification for this state block
    auto notification_it = state_block_notification_map_.find(_state_ptr);
    if (notification_it != state_block_notification_map_.end() && notification_it->second == ADD)
    {
        WOLF_WARN("There is already an ADD notification of this state block");
    }
    else
        state_block_notification_map_[_state_ptr] = ADD;

    return _state_ptr;
}

void Problem::removeStateBlock(StateBlockPtr _state_ptr)
{
    //std::cout << "Problem::removeStateBlockPtr " << _state_ptr.get() << std::endl;
    //assert(std::find(state_block_list_.begin(),state_block_list_.end(),_state_ptr) != state_block_list_.end() && "Removing a state_block that hasn't been added or already removed");
    if(std::find(state_block_list_.begin(),state_block_list_.end(),_state_ptr) == state_block_list_.end())
    {
        WOLF_WARN("Removing a state_block that hasn't been added or already removed");
        return;
    }

    // add the state unit to the list
    state_block_list_.remove(_state_ptr);

    // Check if there is already a notification for this state block
    auto notification_it = state_block_notification_map_.find(_state_ptr);
    if (notification_it != state_block_notification_map_.end())
    {
        if (notification_it->second == REMOVE)
        {
            WOLF_WARN("There is already an REMOVE notification of this state block");
        }
        // Remove ADD notification
        else
        {
            state_block_notification_map_.erase(notification_it);
        }
    }
    // Add REMOVE notification
    else
        state_block_notification_map_[_state_ptr] = REMOVE;
}

FactorBasePtr Problem::addFactor(FactorBasePtr _factor_ptr)
{
    //std::cout << "Problem::addFactorPtr " << _factor_ptr->id() << std::endl;

    // Add ADD notification
    // Check if there is already a notification for this state block
    auto notification_it = factor_notification_map_.find(_factor_ptr);
    if (notification_it != factor_notification_map_.end() && notification_it->second == ADD)
    {
        WOLF_WARN("There is already an ADD notification of this factor");
    }
    // Add ADD notification (override in case of REMOVE)
    else
        factor_notification_map_[_factor_ptr] = ADD;

    return _factor_ptr;
}

void Problem::removeFactor(FactorBasePtr _factor_ptr)
{
    //std::cout << "Problem::removeFactorPtr " << _factor_ptr->id() << std::endl;

    // Check if there is already a notification for this state block
    auto notification_it = factor_notification_map_.find(_factor_ptr);
    if (notification_it != factor_notification_map_.end())
    {
        if (notification_it->second == REMOVE)
        {
            WOLF_WARN("There is already an REMOVE notification of this state block");
        }
        // Remove ADD notification
        else
            factor_notification_map_.erase(notification_it);
    }
    // Add REMOVE notification
    else
        factor_notification_map_[_factor_ptr] = REMOVE;
}

void Problem::clearCovariance()
{
    covariances_.clear();
}

void Problem::addCovarianceBlock(StateBlockPtr _state1, StateBlockPtr _state2, const Eigen::MatrixXs& _cov)
{
    assert(_state1->getSize() == (unsigned int ) _cov.rows() && "wrong covariance block size");
    assert(_state2->getSize() == (unsigned int ) _cov.cols() && "wrong covariance block size");

    covariances_[std::pair<StateBlockPtr, StateBlockPtr>(_state1, _state2)] = _cov;
}

void Problem::addCovarianceBlock(StateBlockPtr _state1, const Eigen::MatrixXs& _cov)
{
    assert(_state1->getSize() == (unsigned int ) _cov.rows() && "wrong covariance block size");
    assert(_state1->getSize() == (unsigned int ) _cov.cols() && "wrong covariance block size");

    covariances_[std::make_pair(_state1, _state1)] = _cov;
}

bool Problem::getCovarianceBlock(StateBlockPtr _state1, StateBlockPtr _state2, Eigen::MatrixXs& _cov, const int _row,
                                 const int _col)
{
    //std::cout << "entire cov to be filled:" << std::endl << _cov << std::endl;
    //std::cout << "_row " << _row << std::endl;
    //std::cout << "_col " << _col << std::endl;
    //std::cout << "_state1 size: " << _state1->getSize() << std::endl;
    //std::cout << "_state2 size: " << _state2->getSize() << std::endl;
    //std::cout << "part of cov to be filled:" << std::endl <<  _cov.block(_row, _col, _state1->getSize(), _state2->getSize()) << std::endl;
    //if (covariances_.find(std::pair<StateBlockPtr, StateBlockPtr>(_state1, _state2)) != covariances_.end())
    //    std::cout << "stored cov" << std::endl << covariances_[std::pair<StateBlockPtr, StateBlockPtr>(_state1, _state2)] << std::endl;
    //else if (covariances_.find(std::pair<StateBlockPtr, StateBlockPtr>(_state2, _state1)) != covariances_.end())
    //    std::cout << "stored cov" << std::endl << covariances_[std::pair<StateBlockPtr, StateBlockPtr>(_state2, _state1)].transpose() << std::endl;

    assert(_row + _state1->getSize() <= _cov.rows() && _col + _state2->getSize() <= _cov.cols() && "Problem::getCovarianceBlock: Bad matrix covariance size!");

    if (covariances_.find(std::pair<StateBlockPtr, StateBlockPtr>(_state1, _state2)) != covariances_.end())
        _cov.block(_row, _col, _state1->getSize(), _state2->getSize()) =
                covariances_[std::pair<StateBlockPtr, StateBlockPtr>(_state1, _state2)];
    else if (covariances_.find(std::pair<StateBlockPtr, StateBlockPtr>(_state2, _state1)) != covariances_.end())
       _cov.block(_row, _col, _state1->getSize(), _state2->getSize()) =
                covariances_[std::pair<StateBlockPtr, StateBlockPtr>(_state2, _state1)].transpose();
    else
    {
      WOLF_DEBUG("Could not find the requested covariance block.");
      return false;
    }

    return true;
}

bool Problem::getCovarianceBlock(std::map<StateBlockPtr, unsigned int> _sb_2_idx, Eigen::MatrixXs& _cov)
{
    // fill covariance
    for (auto it1 = _sb_2_idx.begin(); it1 != _sb_2_idx.end(); it1++)
        for (auto it2 = it1; it2 != _sb_2_idx.end(); it2++)
        {
            StateBlockPtr sb1 = it1->first;
            StateBlockPtr sb2 = it2->first;
            std::pair<StateBlockPtr, StateBlockPtr> pair_12(sb1, sb2);
            std::pair<StateBlockPtr, StateBlockPtr> pair_21(sb2, sb1);

            // search st1 & st2
            if (covariances_.find(pair_12) != covariances_.end())
            {
                assert(_sb_2_idx[sb1] + sb1->getSize() <= _cov.rows() &&
                       _sb_2_idx[sb2] + sb2->getSize() <= _cov.cols() && "Problem::getCovarianceBlock: Bad matrix covariance size!");
                assert(_sb_2_idx[sb2] + sb2->getSize() <= _cov.rows() &&
                       _sb_2_idx[sb1] + sb1->getSize() <= _cov.cols() && "Problem::getCovarianceBlock: Bad matrix covariance size!");

                _cov.block(_sb_2_idx[sb1], _sb_2_idx[sb2], sb1->getSize(), sb2->getSize()) = covariances_[pair_12];
                _cov.block(_sb_2_idx[sb2], _sb_2_idx[sb1], sb2->getSize(), sb1->getSize()) = covariances_[pair_12].transpose();
            }
            else if (covariances_.find(pair_21) != covariances_.end())
            {
                assert(_sb_2_idx[sb1] + sb1->getSize() <= _cov.rows() &&
                       _sb_2_idx[sb2] + sb2->getSize() <= _cov.cols() && "Problem::getCovarianceBlock: Bad matrix covariance size!");
                assert(_sb_2_idx[sb2] + sb2->getSize() <= _cov.rows() &&
                       _sb_2_idx[sb1] + sb1->getSize() <= _cov.cols() && "Problem::getCovarianceBlock: Bad matrix covariance size!");

                _cov.block(_sb_2_idx[sb1], _sb_2_idx[sb2], sb1->getSize(), sb2->getSize()) = covariances_[pair_21].transpose();
                _cov.block(_sb_2_idx[sb2], _sb_2_idx[sb1], sb2->getSize(), sb1->getSize()) = covariances_[pair_21];
            }
            else
                return false;
        }

    return true;
}

bool Problem::getCovarianceBlock(StateBlockPtr _state, Eigen::MatrixXs& _cov, const int _row_and_col)
{
    return getCovarianceBlock(_state, _state, _cov, _row_and_col, _row_and_col);
}

bool Problem::getFrameCovariance(FrameBaseConstPtr _frame_ptr, Eigen::MatrixXs& _covariance)
{
    bool success(true);
    int i = 0, j = 0;

    const auto& state_bloc_vec = _frame_ptr->getStateBlockVec();

    for (const auto& sb_i : state_bloc_vec)
    {
        if (sb_i)
        {
            j = 0;
            for (const auto& sb_j : state_bloc_vec)
            {
                if (sb_j)
                {
                    success = success && getCovarianceBlock(sb_i, sb_j, _covariance, i, j);
                    j += sb_j->getSize();
                }
            }
            i += sb_i->getSize();
        }
    }
    return success;
}

Eigen::MatrixXs Problem::getFrameCovariance(FrameBaseConstPtr _frame_ptr)
{
    SizeEigen sz = 0;
    for (const auto& sb : _frame_ptr->getStateBlockVec())
        if (sb)
            sz += sb->getSize();

    Eigen::MatrixXs covariance(sz, sz);

    getFrameCovariance(_frame_ptr, covariance);
    return covariance;
}

Eigen::MatrixXs Problem::getLastKeyFrameCovariance()
{
    FrameBasePtr frm = getLastKeyFrame();
    return getFrameCovariance(frm);
}

bool Problem::getLandmarkCovariance(LandmarkBaseConstPtr _landmark_ptr, Eigen::MatrixXs& _covariance)
{
    bool success(true);
    int i = 0, j = 0;

    const auto& state_bloc_vec = _landmark_ptr->getStateBlockVec();

    for (const auto& sb_i : state_bloc_vec)
    {
        if (sb_i)
        {
            j = 0;
            for (const auto& sb_j : state_bloc_vec)
            {
                if (sb_j)
                {
                    success = success && getCovarianceBlock(sb_i, sb_j, _covariance, i, j);
                    j += sb_j->getSize();
                }
            }
            i += sb_i->getSize();
        }
    }
    return success;
}

Eigen::MatrixXs Problem::getLandmarkCovariance(LandmarkBaseConstPtr _landmark_ptr)
{
    SizeEigen sz = 0;
    for (const auto& sb : _landmark_ptr->getStateBlockVec())
        if (sb)
            sz += sb->getSize();

    Eigen::MatrixXs covariance(sz, sz);

    getLandmarkCovariance(_landmark_ptr, covariance);
    return covariance;
}

MapBasePtr Problem::getMap()
{
    return map_ptr_;
}

TrajectoryBasePtr Problem::getTrajectory()
{
    return trajectory_ptr_;
}

HardwareBasePtr Problem::getHardware()
{
    return hardware_ptr_;
}

FrameBasePtr Problem::getLastFrame()
{
    return trajectory_ptr_->getLastFrame();
}

FrameBasePtr Problem::getLastKeyFrame()
{
    return trajectory_ptr_->getLastKeyFrame();
}

StateBlockPtrList& Problem::getStateBlockPtrList()
{
    return state_block_list_;
}

FrameBasePtr Problem::setPrior(const Eigen::VectorXs& _prior_state, const Eigen::MatrixXs& _prior_cov, const TimeStamp& _ts, const Scalar _time_tolerance)
{
    if ( ! prior_is_set_ )
    {
        // Create origin frame
        FrameBasePtr origin_keyframe = emplaceFrame(KEY_FRAME, _prior_state, _ts);

        // create origin capture with the given state as data
        // Capture fix only takes 3D position and Quaternion orientation
        CapturePosePtr init_capture;
        if (trajectory_ptr_->getFrameStructure() == "POV 3D")
            init_capture = std::make_shared<CapturePose>(_ts, nullptr, _prior_state.head(7), _prior_cov.topLeftCorner(6,6));
        else
            init_capture = std::make_shared<CapturePose>(_ts, nullptr, _prior_state, _prior_cov);

        origin_keyframe->addCapture(init_capture);

        // emplace feature and factor
        init_capture->emplaceFeatureAndFactor();

        WOLF_DEBUG("Set prior callback: KF", origin_keyframe->id(), " Callback emitted with ts = ", origin_keyframe->getTimeStamp());

        // Notify all motion processors about the prior KF
        for (auto sensor : hardware_ptr_->getSensorList())
            for (auto processor : sensor->getProcessorList())
                if (processor->isMotion())
                    // Motion processors will set its origin at the KF
                    (std::static_pointer_cast<ProcessorMotion>(processor))->setOrigin(origin_keyframe);

        prior_is_set_ = true;

        // Notify all other processors about the origin KF --> they will join it or not depending on their received data
        for (auto sensor : hardware_ptr_->getSensorList())
            for (auto processor : sensor->getProcessorList())
                if ( !processor->isMotion() )
                    processor->keyFrameCallback(origin_keyframe, _time_tolerance);

        return origin_keyframe;
    }
    else
        throw std::runtime_error("Origin already set!");
}

void Problem::loadMap(const std::string& _filename_dot_yaml)
{
    getMap()->load(_filename_dot_yaml);
}

void Problem::saveMap(const std::string& _filename_dot_yaml, const std::string& _map_name)
{
    getMap()->save(_filename_dot_yaml, _map_name);
}

void Problem::print(int depth, bool constr_by, bool metric, bool state_blocks)
{
    using std::cout;
    using std::endl;

    cout << endl;
    cout << "P: wolf tree status ---------------------" << endl;
    cout << "Hardware" << ((depth < 1) ? ("   -- " + std::to_string(getHardware()->getSensorList().size()) + "S") : "")  << endl;
    if (depth >= 1)
    {
        // Sensors =======================================================================================
        for (auto S : getHardware()->getSensorList())
        {
            cout << "  S" << S->id() << " " << S->getType();
            if (!metric && !state_blocks) cout << (S->isExtrinsicDynamic() ? " [Dyn," : " [Sta,") << (S->isIntrinsicDynamic() ? "Dyn]" : "Sta]");
            if (depth < 2)
                cout << " -- " << S->getProcessorList().size() << "p";
            cout << endl;
            if (metric && state_blocks)
            {
                for (unsigned int i = 0; i < S->getStateBlockVec().size(); i++)
                {
                    if (i==0) cout << "    Extr " << (S->isExtrinsicDynamic() ? "[Dyn]" : "[Sta]") << " = [";
                    if (i==2) cout << "    Intr " << (S->isIntrinsicDynamic() ? "[Dyn]" : "[Sta]") << " = [";
                    auto sb = S->getStateBlock(i);
                    if (sb)
                    {
                        cout << (sb->isFixed() ? " Fix( " : " Est( ") << sb->getState().transpose() << " )";
                    }
                    if (i==1) cout << " ]" << endl;
                }
                if (S->getStateBlockVec().size() > 2) cout << " ]" << endl;
            }
            else if (metric)
            {
                cout << "    Extr " << (S->isExtrinsicDynamic() ? "[Dyn]" : "[Sta]") << "= ( ";
                if (S->getP())
                    cout << S->getP()->getState().transpose();
                if (S->getO())
                    cout << " " << S->getO()->getState().transpose();
                cout  << " )";
                if (S->getIntrinsic())
                    cout << "    Intr " << (S->isIntrinsicDynamic() ? "[Dyn]" : "[Sta]") << "= ( " << S->getIntrinsic()->getState().transpose() << " )";
                cout << endl;
            }
            else if (state_blocks)
            {
                cout << "    sb:" << (S->isExtrinsicDynamic() ? "[Dyn," : "[Sta,") << (S->isIntrinsicDynamic() ? "Dyn]" : "Sta]");
                for (auto sb : S->getStateBlockVec())
                    if (sb != nullptr)
                        cout << " " << (sb->isFixed() ? "Fix" : "Est");
                cout << endl;
            }
            if (depth >= 2)
            {
                // Processors =======================================================================================
                for (auto p : S->getProcessorList())
                {
                    if (p->isMotion())
                    {
                        std::cout << "    pm" << p->id() << " " << p->getType() << endl;
                        ProcessorMotionPtr pm = std::static_pointer_cast<ProcessorMotion>(p);
                        if (pm->getOrigin())
                            cout << "      o: C" << pm->getOrigin()->id() << " - " << (pm->getOrigin()->getFrame()->isKey() ? "  KF" : "  F")
                            << pm->getOrigin()->getFrame()->id() << endl;
                        if (pm->getLast())
                            cout << "      l: C" << pm->getLast()->id() << " - " << (pm->getLast()->getFrame()->isKey() ? "  KF" : "  F")
                            << pm->getLast()->getFrame()->id() << endl;
                        if (pm->getIncoming())
                            cout << "      i: C" << pm->getIncoming()->id() << endl;
                    }
                    else
                    {
                        cout << "    pt" << p->id() << " " << p->getType() << endl;
                        ProcessorTrackerPtr pt = std::dynamic_pointer_cast<ProcessorTracker>(p);
                        if (pt)
                        {
//                            ProcessorTrackerFeatureTrifocalPtr ptt = std::dynamic_pointer_cast<ProcessorTrackerFeatureTrifocal>(pt);
//                            if (ptt)
//                            {
//                                if (ptt->getPrevOrigin())
//                                    cout << "      p: C" << ptt->getPrevOrigin()->id() << " - " << (ptt->getPrevOrigin()->getFrame()->isKey() ? "  KF" : "  F")
//                                    << ptt->getPrevOrigin()->getFrame()->id() << endl;
//                            }
                            if (pt->getOrigin())
                                cout << "      o: C" << pt->getOrigin()->id() << " - " << (pt->getOrigin()->getFrame()->isKey() ? "  KF" : "  F")
                                << pt->getOrigin()->getFrame()->id() << endl;
                            if (pt->getLast())
                                cout << "      l: C" << pt->getLast()->id() << " - " << (pt->getLast()->getFrame()->isKey() ? "  KF" : "  F")
                                << pt->getLast()->getFrame()->id() << endl;
                            if (pt->getIncoming())
                                cout << "      i: C" << pt->getIncoming()->id() << endl;
                        }
                    }
                } // for p
            }
        } // for S
    }
    cout << "Trajectory" << ((depth < 1) ? (" -- " + std::to_string(getTrajectory()->getFrameList().size()) + "F") : "")  << endl;
    if (depth >= 1)
    {
        // Frames =======================================================================================
        for (auto F : getTrajectory()->getFrameList())
        {
            cout << (F->isKey() ? "  KF" : "  F") << F->id() << ((depth < 2) ? " -- " + std::to_string(F->getCaptureList().size()) + "C  " : "");
            if (constr_by)
            {
                cout << "  <-- ";
                for (auto cby : F->getConstrainedByList())
                    cout << "c" << cby->id() << " \t";
            }
            cout << endl;
            if (metric)
            {
                cout << (F->isFixed() ? "    Fix" : "    Est") << ", ts=" << std::setprecision(5)
                        << F->getTimeStamp().get();
                cout << ",\t x = ( " << std::setprecision(2) << F->getState().transpose() << " )";
                cout << endl;
            }
            if (state_blocks)
            {
                cout << "    sb:";
                for (auto sb : F->getStateBlockVec())
                    if (sb != nullptr)
                        cout << " " << (sb->isFixed() ? "Fix" : "Est");
                cout << endl;
            }
            if (depth >= 2)
            {
                // Captures =======================================================================================
                for (auto C : F->getCaptureList())
                {
                    cout << "    C" << (C->isMotion() ? "M" : "") << C->id() << " " << C->getType();
                    
                    if(C->getSensor() != nullptr)
                    {
                        cout << " -> S" << C->getSensor()->id();
                        cout << (C->getSensor()->isExtrinsicDynamic() ? " [Dyn, ": " [Sta, ");
                        cout << (C->getSensor()->isIntrinsicDynamic() ? "Dyn]" : "Sta]");
                    }
                    else
                        cout << " -> S-";
                    if (C->isMotion())
                    {
                        auto CM = std::static_pointer_cast<CaptureMotion>(C);
                        if (CM->getOriginFrame())
                            cout << " -> OF" << CM->getOriginFrame()->id();
                    }

                    cout << ((depth < 3) ? " -- " + std::to_string(C->getFeatureList().size()) + "f" : "");
                    if (constr_by)
                    {
                        cout << "  <-- ";
                        for (auto cby : C->getConstrainedByList())
                            cout << "c" << cby->id() << " \t";
                    }
                    cout << endl;

                    if (state_blocks)
                        for(auto sb : C->getStateBlockVec())
                            if(sb != nullptr)
                            {
                                cout << "      sb: ";
                                cout << (sb->isFixed() ? "Fix" : "Est");
                                if (metric)
                                    cout << std::setprecision(2) << " (" << sb->getState().transpose() << " )";
                                cout << endl;
                            }

                    if (C->isMotion() )
                    {
                        CaptureMotionPtr CM = std::dynamic_pointer_cast<CaptureMotion>(C);
                        cout << "      buffer size  :  " << CM->getBuffer().get().size() << endl;
                        if ( metric && ! CM->getBuffer().get().empty())
                        {
                            cout << "      delta preint : (" << CM->getDeltaPreint().transpose() << ")" << endl;
                            if (CM->hasCalibration())
                            {
                                cout << "      calib preint : (" << CM->getCalibrationPreint().transpose() << ")" << endl;
                                cout << "      jacob preint : (" << CM->getJacobianCalib().row(0) << ")" << endl;
                                cout << "      calib current: (" << CM->getCalibration().transpose() << ")" << endl;
                                cout << "      delta correct: (" << CM->getDeltaCorrected(CM->getCalibration()).transpose() << ")" << endl;
                            }
                        }
                    }

                    if (depth >= 3)
                    {
                        // Features =======================================================================================
                        for (auto f : C->getFeatureList())
                        {
                            cout << "      f" << f->id() << " trk" << f->trackId() << " " << f->getType() << ((depth < 4) ? " -- " + std::to_string(f->getFactorList().size()) + "c  " : "");
                            if (constr_by)
                            {
                                cout << "  <--\t";
                                for (auto cby : f->getConstrainedByList())
                                    cout << "c" << cby->id() << " \t";
                            }
                            cout << endl;
                            if (metric)
                                cout << "        m = ( " << std::setprecision(2) << f->getMeasurement().transpose()
                                        << " )" << endl;
                            if (depth >= 4)
                            {
                                // Factors =======================================================================================
                                for (auto c : f->getFactorList())
                                {
                                    cout << "        c" << c->id() << " " << c->getType() << " -->";
                                    if (c->getFrameOther() == nullptr && c->getCaptureOther() == nullptr && c->getFeatureOther() == nullptr && c->getLandmarkOther() == nullptr)
                                        cout << " A";
                                    if (c->getFrameOther() != nullptr)
                                        cout << " F" << c->getFrameOther()->id();
                                    if (c->getCaptureOther() != nullptr)
                                        cout << " C" << c->getCaptureOther()->id();
                                    if (c->getFeatureOther() != nullptr)
                                        cout << " f" << c->getFeatureOther()->id();
                                    if (c->getLandmarkOther() != nullptr)
                                        cout << " L" << c->getLandmarkOther()->id();
                                    cout << endl;
                                } // for c
                            }
                        } // for f
                    }
                } // for C
            }
        } // for F
    }
    cout << "Map" << ((depth < 1) ? ("        -- " + std::to_string(getMap()->getLandmarkList().size()) + "L") : "") << endl;
    if (depth >= 1)
    {
        // Landmarks =======================================================================================
        for (auto L : getMap()->getLandmarkList())
        {
            cout << "  L" << L->id() << " " << L->getType();
            if (constr_by)
            {
                cout << "\t<-- ";
                for (auto cby : L->getConstrainedByList())
                    cout << "c" << cby->id() << " \t";
            }
            cout << endl;
            if (metric)
            {
                cout << (L->isFixed() ? "    Fix" : "    Est");
                cout << ",\t x = ( " << std::setprecision(2) << L->getState().transpose() << " )";
                cout << endl;
            }
            if (state_blocks)
            {
                cout << "    sb:";
                for (auto sb : L->getStateBlockVec())
                    if (sb != nullptr)
                        cout << (sb->isFixed() ? " Fix" : " Est");
                cout << endl;
            }
        } // for L
    }
    cout << "-----------------------------------------" << endl;
    cout << endl;
}

FrameBasePtr Problem::closestKeyFrameToTimeStamp(const TimeStamp& _ts)
{
    return trajectory_ptr_->closestKeyFrameToTimeStamp(_ts);
}

bool Problem::check(int verbose_level)
{
    using std::cout;
    using std::endl;

    bool is_consistent = true; // true if all checks passed; false if any check fails.

    if (verbose_level) cout << endl;
    if (verbose_level) cout << "Wolf tree integrity ---------------------" << endl;
    auto P_raw = this;
    if (verbose_level > 0)
    {
        cout << "P @ " << P_raw << endl;
    }
    // ------------------------
    //     HARDWARE branch
    // ------------------------
    auto H = hardware_ptr_;
    if (verbose_level > 0)
    {
        cout << "H @ " << H.get() << endl;
    }
    // check pointer to Problem
    is_consistent = is_consistent && (H->getProblem().get() == P_raw);

    // Sensors =======================================================================================
    for (auto S : H->getSensorList())
    {
        if (verbose_level > 0)
        {
            cout << "  S" << S->id() << " @ " << S.get() << endl;
            cout << "    -> P @ " << S->getProblem().get() << endl;
            cout << "    -> H @ " << S->getHardware().get() << endl;
            for (auto sb : S->getStateBlockVec())
            {
                cout <<  "    sb @ " << sb.get();
                if (sb)
                {
                    auto lp = sb->getLocalParametrization();
                    if (lp)
                        cout <<  " (lp @ " << lp.get() << ")";
                }
                cout << endl;
            }
        }
        // check problem and hardware pointers
        is_consistent = is_consistent && (S->getProblem().get() == P_raw);
        is_consistent = is_consistent && (S->getHardware() == H);

        // Processors =======================================================================================
        for (auto p : S->getProcessorList())
        {
            if (verbose_level > 0)
            {
                cout << "    p" << p->id() << " @ " << p.get() << " -> S" << p->getSensor()->id() << endl;
                cout << "      -> P  @ " << p->getProblem().get() << endl;
                cout << "      -> S" << p->getSensor()->id() << " @ " << p->getSensor().get() << endl;
            }
            // check problem and sensor pointers
            is_consistent = is_consistent && (p->getProblem().get() == P_raw);
            is_consistent = is_consistent && (p->getSensor() == S);
        }
    }
    // ------------------------
    //    TRAJECTORY branch
    // ------------------------
    auto T = trajectory_ptr_;
    if (verbose_level > 0)
    {
        cout << "T @ " << T.get() << endl;
    }
    // check pointer to Problem
    is_consistent = is_consistent && (T->getProblem().get() == P_raw);

    // Frames =======================================================================================
    for (auto F : T->getFrameList())
    {
        if (verbose_level > 0)
        {
            cout << (F->isKey() ? "  KF" : "  F") << F->id() << " @ " << F.get() << endl;
            cout << "    -> P @ " << F->getProblem().get() << endl;
            cout << "    -> T @ " << F->getTrajectory().get() << endl;
            for (auto sb : F->getStateBlockVec())
            {
                cout <<  "    sb @ " << sb.get();
                if (sb)
                {
                    auto lp = sb->getLocalParametrization();
                    if (lp)
                        cout <<  " (lp @ " << lp.get() << ")";
                }
                cout << endl;
            }
        }
        // check problem and trajectory pointers
        is_consistent = is_consistent && (F->getProblem().get() == P_raw);
        is_consistent = is_consistent && (F->getTrajectory() == T);
        for (auto cby : F->getConstrainedByList())
        {
            if (verbose_level > 0)
            {
                cout << "    <- c" << cby->id() << " -> F" << cby->getFrameOther()->id() << endl;
            }
            // check constrained_by pointer to this frame
            is_consistent = is_consistent && (cby->getFrameOther() == F);
            for (auto sb : cby->getStateBlockPtrVector())
            {
                if (verbose_level > 0)
                {
                    cout << "      sb @ " << sb.get();
                    if (sb)
                    {
                        auto lp = sb->getLocalParametrization();
                        if (lp)
                            cout <<  " (lp @ " << lp.get() << ")";
                    }
                    cout << endl;
                }
            }
        }

        // Captures =======================================================================================
        for (auto C : F->getCaptureList())
        {
            if (verbose_level > 0)
            {
                cout << "    C" << C->id() << " @ " << C.get() << " -> S";
                if (C->getSensor()) cout << C->getSensor()->id();
                else cout << "-";
                cout << endl;
                cout << "      -> P  @ " << C->getProblem().get() << endl;
                cout << "      -> F" << C->getFrame()->id() << " @ " << C->getFrame().get() << endl;
                for (auto sb : C->getStateBlockVec())
                {
                    cout <<  "      sb @ " << sb.get();
                    if (sb)
                    {
                        auto lp = sb->getLocalParametrization();
                        if (lp)
                            cout <<  " (lp @ " << lp.get() << ")";
                    }
                    cout << endl;
                }
            }
            // check problem and frame pointers
            is_consistent = is_consistent && (C->getProblem().get() == P_raw);
            is_consistent = is_consistent && (C->getFrame() == F);

            // Features =======================================================================================
            for (auto f : C->getFeatureList())
            {
                if (verbose_level > 0)
                {
                    cout << "      f" << f->id() << " @ " << f.get() << endl;
                    cout << "        -> P  @ " << f->getProblem().get() << endl;
                    cout << "        -> C" << f->getCapture()->id() << " @ " << f->getCapture().get()
                            << endl;
                }
                // check problem and capture pointers
                is_consistent = is_consistent && (f->getProblem().get() == P_raw);
                is_consistent = is_consistent && (f->getCapture() == C);

                for (auto cby : f->getConstrainedByList())
                {
                    if (verbose_level > 0)
                    {
                        cout << "     <- c" << cby->id() << " -> f" << cby->getFeatureOther()->id() << endl;
                    }
                    // check constrained_by pointer to this feature
                    is_consistent = is_consistent && (cby->getFeatureOther() == f);
                }

                // Factors =======================================================================================
                for (auto c : f->getFactorList())
                {
                    if (verbose_level > 0)
                        cout << "        c" << c->id() << " @ " << c.get();

                    auto Fo = c->getFrameOther();
                    auto Co = c->getCaptureOther();
                    auto fo = c->getFeatureOther();
                    auto Lo = c->getLandmarkOther();

                    if ( !Fo && !Co && !fo && !Lo )    // case ABSOLUTE:
                    {
                        if (verbose_level > 0)
                            cout << " --> Abs." << endl;
                    }

                    // find constrained_by pointer in constrained frame
                    if ( Fo )  // case FRAME:
                    {
                        if (verbose_level > 0)
                            cout << " --> F" << Fo->id() << " <- ";
                        bool found = false;
                        for (auto cby : Fo->getConstrainedByList())
                        {
                            if (verbose_level > 0)
                                cout << " c" << cby->id();
                            found = found || (c == cby);
                        }
                        if (verbose_level > 0)
                            cout << endl;
                        // check constrained_by pointer in constrained frame
                        is_consistent = is_consistent && found;
                    }

                    // find constrained_by pointer in constrained capture
                    if ( Co )  // case CAPTURE:
                    {
                        if (verbose_level > 0)
                            cout << " --> C" << Co->id() << " <- ";
                        bool found = false;
                        for (auto cby : Co->getConstrainedByList())
                        {
                            if (verbose_level > 0)
                                cout << " c" << cby->id();
                            found = found || (c == cby);
                        }
                        if (verbose_level > 0)
                            cout << endl;
                        // check constrained_by pointer in constrained frame
                        is_consistent = is_consistent && found;
                    }

                    // find constrained_by pointer in constrained feature
                    if ( fo )   // case FEATURE:
                    {
                        if (verbose_level > 0)
                            cout << " --> f" << fo->id() << " <- ";
                        bool found = false;
                        for (auto cby : fo->getConstrainedByList())
                        {
                            if (verbose_level > 0)
                                cout << " c" << cby->id();
                            found = found || (c == cby);
                        }
                        if (verbose_level > 0)
                            cout << endl;
                        // check constrained_by pointer in constrained feature
                        is_consistent = is_consistent && found;
                    }

                    // find constrained_by pointer in constrained landmark
                    if ( Lo )      // case LANDMARK:
                    {
                        if (verbose_level > 0)
                            cout << " --> L" << Lo->id() << " <- ";
                        bool found = false;
                        for (auto cby : Lo->getConstrainedByList())
                        {
                            if (verbose_level > 0)
                                cout << " c" << cby->id();
                            found = found || (c == cby);
                        }
                        if (verbose_level > 0)
                            cout << endl;
                        // check constrained_by pointer in constrained landmark
                        is_consistent = is_consistent && found;
                    }
                    if (verbose_level > 0)
                    {
                        cout << "          -> P  @ " << c->getProblem().get() << endl;
                        cout << "          -> f" << c->getFeature()->id() << " @ " << c->getFeature().get() << endl;
                    }
                    // check problem and feature pointers
                    is_consistent = is_consistent && (c->getProblem().get() == P_raw);
                    is_consistent = is_consistent && (c->getFeature() == f);

                    // find state block pointers in all constrained nodes
                    SensorBasePtr S = c->getFeature()->getCapture()->getSensor(); // get own sensor to check sb
                    for (auto sb : c->getStateBlockPtrVector())
                    {
                        bool found = false;
                        if (verbose_level > 0)
                        {
                            cout <<  "          sb @ " << sb.get();
                            if (sb)
                            {
                                auto lp = sb->getLocalParametrization();
                                if (lp)
                                    cout <<  " (lp @ " << lp.get() << ")";
                            }
                        }
                        // find in own Frame
                        found = found || (std::find(F->getStateBlockVec().begin(), F->getStateBlockVec().end(), sb) != F->getStateBlockVec().end());
                        // find in own Capture
                        found = found || (std::find(C->getStateBlockVec().begin(), C->getStateBlockVec().end(), sb) != C->getStateBlockVec().end());
                        // find in own Sensor
                        if (S)
                            found = found || (std::find(S->getStateBlockVec().begin(), S->getStateBlockVec().end(), sb) != S->getStateBlockVec().end());
                        // find in constrained Frame
                        if (Fo)
                            found = found || (std::find(Fo->getStateBlockVec().begin(), Fo->getStateBlockVec().end(), sb) != Fo->getStateBlockVec().end());
                        // find in constrained Capture
                        if (Co)
                            found = found || (std::find(Co->getStateBlockVec().begin(), Co->getStateBlockVec().end(), sb) != Co->getStateBlockVec().end());
                        // find in constrained Feature
                        if (fo)
                        {
                            // find in constrained feature's Frame
                            FrameBasePtr foF = fo->getFrame();
                            found = found || (std::find(foF->getStateBlockVec().begin(), foF->getStateBlockVec().end(), sb) != foF->getStateBlockVec().end());
                            // find in constrained feature's Capture
                            CaptureBasePtr foC = fo->getCapture();
                            found = found || (std::find(foC->getStateBlockVec().begin(), foC->getStateBlockVec().end(), sb) != foC->getStateBlockVec().end());
                            // find in constrained feature's Sensor
                            SensorBasePtr foS = fo->getCapture()->getSensor();
                            found = found || (std::find(foS->getStateBlockVec().begin(), foS->getStateBlockVec().end(), sb) != foS->getStateBlockVec().end());
                        }
                        // find in constrained landmark
                        if (Lo)
                            found = found || (std::find(Lo->getStateBlockVec().begin(), Lo->getStateBlockVec().end(), sb) != Lo->getStateBlockVec().end());
                        if (verbose_level > 0)
                        {
                            if (found)
                                cout << " found";
                            else
                                cout << " NOT FOUND !";
                            cout << endl;
                        }
                        // check that all state block pointers were found
                        is_consistent = is_consistent && found;
                    }
                }
            }
        }
    }
    // ------------------------
    //       MAP branch
    // ------------------------
    auto M = map_ptr_;
    if (verbose_level > 0)
        cout << "M @ " << M.get() << endl;
    // check pointer to Problem
    is_consistent = is_consistent && (M->getProblem().get() == P_raw);

    // Landmarks =======================================================================================
    for (auto L : M->getLandmarkList())
    {
        if (verbose_level > 0)
        {
            cout << "  L" << L->id() << " @ " << L.get() << endl;
            cout << "  -> P @ " << L->getProblem().get() << endl;
            cout << "  -> M @ " << L->getMap().get() << endl;
            for (auto sb : L->getStateBlockVec())
            {
                cout <<  "  sb @ " << sb.get();
                if (sb)
                {
                    auto lp = sb->getLocalParametrization();
                    if (lp)
                        cout <<  " (lp @ " << lp.get() << ")";
                }
                cout << endl;
            }
        }
        // check problem and map pointers
        is_consistent = is_consistent && (L->getProblem().get() == P_raw);
        is_consistent = is_consistent && (L->getMap() == M);
        for (auto cby : L->getConstrainedByList())
        {
            if (verbose_level > 0)
                cout << "      <- c" << cby->id() << " -> L" << cby->getLandmarkOther()->id() << endl;
            // check constrained_by pointer to this landmark
            is_consistent = is_consistent && (cby->getLandmarkOther() && L->id() == cby->getLandmarkOther()->id());
            for (auto sb : cby->getStateBlockPtrVector())
            {
                if (verbose_level > 0)
                {
                    cout << "      sb @ " << sb.get();
                    if (sb)
                    {
                        auto lp = sb->getLocalParametrization();
                        if (lp)
                            cout <<  " (lp @ " << lp.get() << ")";
                    }
                    cout << endl;
                }
            }
        }
    }

    if (verbose_level) cout << "--------------------------- Wolf tree " << (is_consistent ? " OK" : "Not OK !!") << endl;
    if (verbose_level) cout << endl;

    return is_consistent;
}

void Problem::print(const std::string& depth, bool constr_by, bool metric, bool state_blocks)
{
    if (depth.compare("T") == 0)
        print(0, constr_by, metric, state_blocks);
    else if (depth.compare("F") == 0)
        print(1, constr_by, metric, state_blocks);
    else if (depth.compare("C") == 0)
        print(2, constr_by, metric, state_blocks);
    else if (depth.compare("f") == 0)
        print(3, constr_by, metric, state_blocks);
    else if (depth.compare("c") == 0)
        print(4, constr_by, metric, state_blocks);
    else
        print(0, constr_by, metric, state_blocks);
}

} // namespace wolf

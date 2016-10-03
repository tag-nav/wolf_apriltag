#include "processor_base.h"
#include "capture_base.h"
#include "frame_base.h"

namespace wolf {

unsigned int ProcessorBase::processor_id_count_ = 0;

ProcessorBase::ProcessorBase(ProcessorType _tp, const std::string& _type, const Scalar& _time_tolerance) :
        NodeBase("PROCESSOR"),
        problem_ptr_(nullptr),
        sensor_ptr_(nullptr),
        processor_id_(++processor_id_count_),
        type_id_(_tp),
        time_tolerance_(_time_tolerance)
{
    //
}

ProcessorBase::~ProcessorBase()
{
}

bool ProcessorBase::permittedKeyFrame()
{
    return getProblem()->permitKeyFrame(this);
}

void ProcessorBase::makeFrame(CaptureBasePtr _capture_ptr, FrameKeyType _type)
{

    // We need to create the new free Frame to hold what will become the last Capture
    FrameBasePtr new_frame_ptr = getProblem()->createFrame(_type, _capture_ptr->getTimeStamp());

    new_frame_ptr->addCapture(_capture_ptr); // Add incoming Capture to the new Frame

    if (_type == KEY_FRAME)
        // Keyframe callback in order to let the other processors to establish their constraints
        getProblem()->keyFrameCallback(_capture_ptr->getFramePtr(), this, time_tolerance_);
}

} // namespace wolf

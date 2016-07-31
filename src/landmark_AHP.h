#ifndef LANDMARK_AHP_H
#define LANDMARK_AHP_H

//Wolf includes
#include "landmark_base.h"
#include "yaml/yaml_conversion.h"

//OpenCV includes
#include "opencv2/features2d/features2d.hpp"

namespace wolf {

/* Landmark - Anchored Homogeneous Point*/
class LandmarkAHP : public LandmarkBase
{
    protected:
        cv::Mat descriptor_;
        FrameBase* anchor_frame_;
        SensorBase* anchor_sensor_;

    public:
        LandmarkAHP(Eigen::Vector4s _position, FrameBase* _frame, SensorBase* _anchor_sensor, cv::Mat _2D_descriptor);

        virtual ~LandmarkAHP();

        const cv::Mat& getDescriptor() const;
        void setDescriptor(const cv::Mat& _descriptor);

        const FrameBase*  getAnchorFrame () const;
        const SensorBase* getAnchorSensor() const;

        void setAnchorFrame  (FrameBase*  _anchor_frame );
        void setAnchorSensor (SensorBase* _anchor_sensor);
        void setAnchor       (FrameBase*  _anchor_frame , SensorBase* _anchor_sensor);

        /** \brief Creator for Factory<LandmarkBase, YAML::Node>
         * Caution: This creator does not set the landmark's anchor frame and sensor.
         * These need to be set afterwards.
         */
        static LandmarkBase* create(const YAML::Node& _node);
};

inline const cv::Mat& LandmarkAHP::getDescriptor() const
{
    return descriptor_;
}

inline void LandmarkAHP::setDescriptor(const cv::Mat& _descriptor)
{
    descriptor_ = _descriptor;
}

inline const FrameBase* LandmarkAHP::getAnchorFrame() const
{
    return anchor_frame_;
}

inline void LandmarkAHP::setAnchorFrame(FrameBase* _anchor_frame)
{
    anchor_frame_ = _anchor_frame;
}

inline const SensorBase* LandmarkAHP::getAnchorSensor() const
{
    return anchor_sensor_;
}

inline void LandmarkAHP::setAnchorSensor(SensorBase* _anchor_sensor)
{
    anchor_sensor_ = _anchor_sensor;
}

inline void LandmarkAHP::setAnchor(FrameBase* _anchor_frame, SensorBase* _anchor_sensor)
{
    anchor_frame_  = _anchor_frame;
    anchor_sensor_ = _anchor_sensor;
}

inline wolf::LandmarkBase* LandmarkAHP::create(const YAML::Node& _node)
{
    Eigen::VectorXs pos = _node["position"].as<Eigen::VectorXs>();
    cv::Mat desc(_node["descriptor"].as<std::vector<unsigned int> >());
    return new LandmarkAHP(pos, nullptr, nullptr, desc);
}

} // namespace wolf

#endif // LANDMARK_AHP_H

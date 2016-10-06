/**
 * \file sensor_factory.cpp
 *
 *  Created on: Apr 25, 2016
 *      \author: jsola
 */

#include "sensor_factory.h"

#include <iostream>
#include <iomanip>

namespace wolf
{

bool SensorFactory::registerCreator(const std::string& _sensor_type, CreateSensorCallback createFn)
{
    bool reg = callbacks_.insert(CallbackMap::value_type(_sensor_type, createFn)).second;
    if (reg)
        std::cout << std::setw(22) << std::left << "SensorFactory" << " : registered " << _sensor_type << std::endl;
    else
        std::cout << "SensorFactory     : sensor " << _sensor_type << " already registered. Skipping. " << std::endl;
    return reg;
}

bool SensorFactory::unregisterCreator(const std::string& _sensor_type)
{
    return callbacks_.erase(_sensor_type) == 1;
}

SensorBasePtr SensorFactory::create(const std::string& _sensor_type, const std::string& _name, const Eigen::VectorXs& _extrinsics, const IntrinsicsBasePtr _intrinsics)
{
    CallbackMap::const_iterator creator_callback_it = callbacks_.find(_sensor_type);
    if (creator_callback_it == callbacks_.end())
    {
        // not found
        throw std::runtime_error("Unknown Sensor type");
    }
    // Invoke the creation function
    return (creator_callback_it->second)(_name, _extrinsics, _intrinsics);
}

// Singleton ---------------------------------------------------
// This class is a singleton. The code below guarantees this.

SensorFactory& SensorFactory::get() // Unique point of access;
{
    static SensorFactory instance_;
    return instance_;
}

} /* namespace wolf */

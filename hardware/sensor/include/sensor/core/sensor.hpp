#ifndef SENSOR_CORE_SENSOR_HPP_
#define SENSOR_CORE_SENSOR_HPP_

#include <cstdint>

#include "sensor/core/sensor_interface.hpp"

namespace sensor_interface {

class Sensor {
public:
    explicit Sensor(uint32_t period)
    : period_(period) {}

    virtual ~Sensor() = default;

    virtual void initialize() = 0;
    
    virtual void update() = 0;

    virtual void shutdown() = 0;

    uint32_t period() const { return period_; }

protected:
    const uint32_t period_;
};

} // namespace sensor_interface

#endif // SENSOR_CORE_SENSOR_HPP_
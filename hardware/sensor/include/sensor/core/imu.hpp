#ifndef SENSOR_CORE_IMU_HPP_
#define SENSOR_CORE_IMU_HPP_

#include <string>

#include "sensor/core/sensor.hpp"

namespace sensor_interface {

class Imu : public Sensor {
public:
    explicit Imu(uint32_t period, const std::string& device, uint32_t baudrate)
    : Sensor(period), device_(device), baudrate_(baudrate) {}

    virtual ~Imu() = default;

    virtual void read(imu_data_t& data) = 0;

protected:
    const std::string device_;

    const uint32_t baudrate_;
};

}

#endif // SENSOR_CORE_IMU_HPP_
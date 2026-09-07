#ifndef SENSOR_CORE_SENSOR_INTERFACE_HPP_
#define SENSOR_CORE_SENSOR_INTERFACE_HPP_

namespace sensor_interface {

struct imu_data_t {
    double orientation[4]{};
    double angular_velocity[3]{};
    double linear_acceleration[3]{};
    double temperature{};
};

} // namespace sensor_interface

#endif // SENSOR_CORE_SENSOR_INTERFACE_HPP_
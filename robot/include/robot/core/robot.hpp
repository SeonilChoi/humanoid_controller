#ifndef ROBOT_CORE_ROBOT_HPP_
#define ROBOT_CORE_ROBOT_HPP_

#include <string>

#include "robot/core/kinematics.hpp"
#include "motor/motor_manager.hpp"
#include "sensor/sensor_manager.hpp"

namespace robot {

class Robot {
public:
    explicit Robot(const std::string& config_file)
    : config_file_(config_file) {}

    virtual ~Robot() = default;

    virtual void start() = 0;

    virtual void stop() = 0;

    virtual void observation() = 0;

    virtual void control() = 0;

protected:
    motor_manager::MotorManager motor_manager_;

    sensor_manager::SensorManager sensor_manager_;

    kinematics::Kinematics kinematics_;

    const std::string config_file_;
};

} // namespace robot

#endif // ROBOT_CORE_ROBOT_HPP_
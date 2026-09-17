#ifndef MOTOR_CORE_MOTOR_INTERFACE_HPP_
#define MOTOR_CORE_MOTOR_INTERFACE_HPP_

#include <cstdint>

namespace motor_interface {

static constexpr uint8_t MAX_MOTORS = 12;

struct motor_command_t {
    double position{};
    double velocity{};
    double torque{};
    double kp{};
    double kd{};
};

struct motor_state_t {
    double position{};
    double velocity{};
    double torque{};
    double temperature{};
    uint8_t error{};
};

} // namespace motor_interface

#endif // MOTOR_CORE_MOTOR_INTERFACE_HPP_
#ifndef MOTOR_MOTOR_INTERFACE_HPP_
#define MOTOR_MOTOR_INTERFACE_HPP_

#include <cstdint>

namespace motor_interface {

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
};

} // namespace motor_interface

#endif // MOTOR_MOTOR_INTERFACE_HPP_
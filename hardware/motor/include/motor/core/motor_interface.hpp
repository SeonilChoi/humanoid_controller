#ifndef MOTOR_CORE_MOTOR_INTERFACE_HPP_
#define MOTOR_CORE_MOTOR_INTERFACE_HPP_

#include <cstdint>

namespace motor_interface {

static constexpr uint8_t MAX_MOTORS = 12;

struct motor_route_t {
    uint8_t motor_index; // Index of the motor in the global motor array
    uint8_t motor_id;    // ID of the motor, unique within its master
};

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
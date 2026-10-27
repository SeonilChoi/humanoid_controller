#ifndef MOTOR_CORE_MOTOR_INTERFACE_HPP_
#define MOTOR_CORE_MOTOR_INTERFACE_HPP_

#include <cstdint>

namespace motor_interface {

static constexpr uint8_t MAX_MOTORS = 16;  // ARTI-D1 14

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

// PD gains a program sends as motor_command_t kp / kd while it controls the motor (motor.yaml 'kp' / 'kd'),
// in DRIVER units: Unitree rotor side, CubeMars MIT output side, Dynamixel Position P / D Gain register values.
struct motor_gains_t {
    double kp{};
    double kd{};
};

struct motor_homing_t {
    int8_t direction{};
    double offset{};
    double velocity{};
    double kp{};
    double kd{};
    double zero_kp{};
    double zero_kd{};
    double hold_kp{};
    double hold_kd{};
    double torque_threshold{};
    double velocity_threshold{};
};

} // namespace motor_interface

#endif // MOTOR_CORE_MOTOR_INTERFACE_HPP_
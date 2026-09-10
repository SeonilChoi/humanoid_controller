#ifndef JOY_CORE_JOY_INTERFACE_HPP_
#define JOY_CORE_JOY_INTERFACE_HPP_

#include <cstdint>

namespace joy_interface {

struct joy_data_t {
    bool square{};
    bool circle{};
    bool triangle{};
    bool cross{};

    bool l1{};
    bool r1{};
    bool l2{};
    bool r2{};

    bool select{};
    bool start{};
    bool center{};

    float dpad_x{};
    float dpad_y{};

    float stick_lx{};
    float stick_ly{};
    float stick_rx{};
    float stick_ry{};

    float l2_analog{};
    float r2_analog{};
};

} // namespace joy_interface

#endif // JOY_CORE_JOY_INTERFACE_HPP_
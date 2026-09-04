#ifndef MOTOR_CORE_MOTOR_MASTER_HPP_
#define MOTOR_CORE_MOTOR_MASTER_HPP_

#include <memory>
#include <cstddef>
#include <cstdint>
#include <unordered_map>

#include "motor/core/motor_driver.hpp"

namespace motor_interface {

class MotorMaster {
public:
    explicit MotorMaster(const uint32_t period)
    : period_(period) {}

    virtual ~MotorMaster() = default;

    virtual void add_motor(uint8_t id, double gear_ratio, double zero_offset, uint32_t pulse_per_revolution) = 0;

    virtual void initialize() = 0;

    virtual void shutdown() = 0;

    virtual void update(uint8_t id, const motor_command_t& command, motor_state_t& status) = 0;

    uint32_t period() const { return period_; }

protected:
    const uint32_t period_;

    std::unordered_map<uint8_t, std::unique_ptr<MotorDriver>> drivers_;
};

} // namespace motor_interface

#endif // MOTOR_CORE_MOTOR_MASTER_HPP_
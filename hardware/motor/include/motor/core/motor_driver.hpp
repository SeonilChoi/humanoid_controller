#ifndef MOTOR_CORE_MOTOR_DRIVER_HPP_
#define MOTOR_CORE_MOTOR_DRIVER_HPP_

namespace motor_interface {

class MotorDriver {
public:
    explicit MotorDriver(double gear_ratio, double zero_offset)
    : gear_ratio_(gear_ratio), zero_offset_(zero_offset) {}

    virtual ~MotorDriver() = default;

    virtual std::size_t encode(const motor_command_t& command, uint8_t* buffer, std::size_t size) = 0;

    virtual void decode(const uint8_t* buffer, std::size_t size, motor_state_t& state) = 0;

protected:
    const double gear_ratio_;

    const double zero_offset_;
};

} // namespace motor_interface

#endif // MOTOR_CORE_MOTOR_DRIVER_HPP_
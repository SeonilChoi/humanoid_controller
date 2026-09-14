#ifndef MOTOR_DRIVER_DYNAMIXEL_DRIVER_HPP_
#define MOTOR_DRIVER_DYNAMIXEL_DRIVER_HPP_

#include <cstdint>
#include <cstddef>
#include <stdexcept>

#include "motor/core/motor_driver.hpp"

namespace dynamixel {

constexpr std::size_t TX_DATA_SIZE = 4;
constexpr std::size_t RX_DATA_SIZE = 21;

constexpr uint16_t ADDR_TORQUE_ENABLE = 64;
constexpr uint16_t ADDR_GOAL_POSITION = 116;
constexpr uint16_t ADDR_PRESENT_CURRENT = 126;

constexpr uint8_t TORQUE_ENABLE = 1;
constexpr uint8_t TORQUE_DISABLE = 0;

constexpr double XM430_VELOCITY_UNIT = 0.229; // RPM
constexpr double XM430_CURRENT_UNIT = 0.00269; // A

constexpr double PI = 3.14159265358979323846;

class DynamixelDriver : public motor_interface::MotorDriver {
public:
    DynamixelDriver(uint8_t id, double gear_ratio, double zero_offset, uint32_t pulse_per_revolution)
    : motor_interface::MotorDriver(id, gear_ratio, zero_offset, pulse_per_revolution) {
        if (id > 252) throw std::runtime_error("[DynamixelDriver] Motor ID must be 0-252.");
    }

    ~DynamixelDriver() override = default;

    std::size_t encode(const motor_interface::motor_command_t& command, uint8_t* buffer, std::size_t size) override {
        if (buffer == nullptr) throw std::runtime_error("[DynamixelDriver::encode] Null buffer.");
        if (size != TX_DATA_SIZE) throw std::runtime_error("[DynamixelDriver::encode] Invalid buffer size.");

        write_s32_little_endian(buffer, position(command.position));

        (void)command.velocity;
        (void)command.torque;
        (void)command.kp;
        (void)command.kd;
        
        return TX_DATA_SIZE;
    }

    void decode(const uint8_t* buffer, std::size_t size, motor_interface::motor_state_t& status) override {
        if (buffer == nullptr) throw std::runtime_error("[DynamixelDriver::decode] Null buffer.");
        if (size != RX_DATA_SIZE) throw std::runtime_error("[DynamixelDriver::decode] Invalid buffer size.");

        const int16_t current_raw = read_s16_little_endian(buffer + 0);
        const int32_t velocity_raw = read_s32_little_endian(buffer + 2);
        const int32_t position_raw = read_s32_little_endian(buffer + 6);
        const uint8_t temperature_raw = buffer[20];

        status.position = position(position_raw);
        status.velocity = velocity(velocity_raw);
        status.torque = torque(current_raw);
        status.temperature = static_cast<double>(temperature_raw);
    }

    std::size_t enable(uint8_t* buffer, std::size_t size) override {
        if (buffer == nullptr) throw std::runtime_error("[DynamixelDriver::enable] Null buffer.");
        if (size < 1) throw std::runtime_error("[DynamixelDriver::enable] Invalid buffer size.");

        buffer[0] = TORQUE_ENABLE;
        return 1;
    }

    std::size_t disable(uint8_t* buffer, std::size_t size) override {
        if (buffer == nullptr) throw std::runtime_error("[DynamixelDriver::disable] Null buffer.");
        if (size < 1) throw std::runtime_error("[DynamixelDriver::disable] Invalid buffer size.");

        buffer[0] = TORQUE_DISABLE;
        return 1;
    }

private:
    int32_t position(const double& value) {
        const double motor = (value + zero_offset_) * gear_ratio_;
        return static_cast<int32_t>(motor / (PI * 2.0) * pulse_per_revolution_);
    }
    double position(const int32_t& value) {
        const double motor = static_cast<double>(value) / pulse_per_revolution_ * (PI * 2.0);
        return motor / gear_ratio_ - zero_offset_;
    }
    double velocity(const int32_t& value) {
        const double rpm = static_cast<double>(value) * XM430_VELOCITY_UNIT;
        const double motor = rpm * (PI * 2.0) / 60.0;
        return motor / gear_ratio_;
    }
    double torque(const int16_t& value) {
        const double current = static_cast<double>(value) * XM430_CURRENT_UNIT;
        return current * gear_ratio_;
    }
};

} // namespace dynamixel

#endif // MOTOR_DRIVER_DYNAMIXEL_DRIVER_HPP_
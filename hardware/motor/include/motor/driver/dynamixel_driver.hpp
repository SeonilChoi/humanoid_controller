#ifndef MOTOR_DRIVER_DYNAMIXEL_DRIVER_HPP_
#define MOTOR_DRIVER_DYNAMIXEL_DRIVER_HPP_

#include <cstdint>
#include <cstddef>
#include <stdexcept>

#include "motor/core/motor_driver.hpp"

namespace dynamixel {

constexpr std::size_t TX_DATA_SIZE = 4;
constexpr std::size_t GAIN_DATA_SIZE = 6;
constexpr std::size_t RX_DATA_SIZE = 21;

constexpr uint16_t ADDR_TORQUE_ENABLE = 64;
constexpr uint16_t ADDR_POSITION_D_GAIN = 80;   // D (80), I (82), P (84): 2 bytes each, written together
constexpr uint16_t ADDR_GOAL_POSITION = 116;
constexpr uint16_t ADDR_PRESENT_CURRENT = 126;

constexpr uint8_t TORQUE_ENABLE = 1;
constexpr uint8_t TORQUE_DISABLE = 0;

constexpr double XM430_VELOCITY_UNIT = 0.229; // RPM
constexpr double XM430_CURRENT_UNIT = 0.00269; // A

constexpr double PI = 3.14159265358979323846;

class DynamixelDriver : public motor_interface::MotorDriver {
public:
    DynamixelDriver(uint8_t id, double gear_ratio, int8_t axis, uint32_t pulse_per_revolution, double min, double max)
    : motor_interface::MotorDriver(id, gear_ratio, axis, pulse_per_revolution, min, max) {
        if (id > 252) throw std::runtime_error("[DynamixelDriver] Motor ID must be 0-252.");
    }

    ~DynamixelDriver() override = default;

    std::size_t encode(const motor_interface::motor_command_t& command, uint8_t* buffer, std::size_t size) override {
        if (buffer == nullptr) throw std::runtime_error("[DynamixelDriver::encode] Null buffer.");
        if (size != TX_DATA_SIZE) throw std::runtime_error("[DynamixelDriver::encode] Invalid buffer size.");

        write_s32_little_endian(buffer, position(command.position));

        // kp / kd go to the servo's Position P / D Gain registers (DynamixelMaster writes them when they change).
        (void)command.velocity;
        (void)command.torque;
        
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

    // Position D / I / P Gain registers (80 / 82 / 84) from motor_command_t kd / kp (register values, I = 0).
    std::size_t encode_gains(const motor_interface::motor_command_t& command, uint8_t* buffer, std::size_t size) {
        if (buffer == nullptr) throw std::runtime_error("[DynamixelDriver::encode_gains] Null buffer.");
        if (size != GAIN_DATA_SIZE) throw std::runtime_error("[DynamixelDriver::encode_gains] Invalid buffer size.");

        const auto gain = [](double value) { return static_cast<uint16_t>(clamp(value, 0.0, 16383.0) + 0.5); };
        const uint16_t d = gain(command.kd);
        const uint16_t p = gain(command.kp);
        buffer[0] = static_cast<uint8_t>(d);
        buffer[1] = static_cast<uint8_t>(d >> 8);
        buffer[2] = 0;
        buffer[3] = 0;
        buffer[4] = static_cast<uint8_t>(p);
        buffer[5] = static_cast<uint8_t>(p >> 8);
        return GAIN_DATA_SIZE;
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
        double motor;
        if (zero_offset_ == -10.0) {
            motor = value * axis_ * gear_ratio_;
        } else {
            motor = (clamp(value, min_, max_) * axis_ + zero_offset_) * gear_ratio_;
        }
        return static_cast<int32_t>(motor / (PI * 2.0) * pulse_per_revolution_);
    }
    double position(const int32_t& value) {
        const double motor = static_cast<double>(value) / pulse_per_revolution_ * (PI * 2.0);
        if (zero_offset_ == -10.0) {
            return motor / gear_ratio_ * axis_;
        } else {
            // inverse of position(double): motor / gear = joint * axis + zero_offset (also right for axis -1)
            return (motor / gear_ratio_ - zero_offset_) * axis_;
        }
    }
    double velocity(const int32_t& value) {
        const double rpm = static_cast<double>(value) * XM430_VELOCITY_UNIT;
        const double motor = rpm * (PI * 2.0) / 60.0;
        return motor / gear_ratio_ * axis_;
    }
    double torque(const int16_t& value) {
        const double current = static_cast<double>(value) * XM430_CURRENT_UNIT;
        return current * gear_ratio_ * axis_;
    }
};

} // namespace dynamixel

#endif // MOTOR_DRIVER_DYNAMIXEL_DRIVER_HPP_
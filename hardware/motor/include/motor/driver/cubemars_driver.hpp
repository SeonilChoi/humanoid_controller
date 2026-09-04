#ifndef MOTOR_DRIVER_CUBEMARS_DRIVER_HPP_
#define MOTOR_DRIVER_CUBEMARS_DRIVER_HPP_

#include <cmath>
#include <cstdint>
#include <cstddef>
#include <stdexcept>

#include "motor/core/motor_driver.hpp"

namespace cubemars {

constexpr std::size_t TX_PACKET_SIZE = 8;
constexpr std::size_t RX_PACKET_SIZE = 8;

constexpr double POSITION_MIN = -12.5;
constexpr double POSITION_MAX =  12.5;

constexpr double VELOCITY_MIN = -6.0;
constexpr double VELOCITY_MAX =  6.0;

constexpr double TORQUE_MIN = -34.0;
constexpr double TORQUE_MAX =  34.0;

constexpr double KP_MIN = 0.0;
constexpr double KP_MAX = 500.0;

constexpr double KD_MIN = 0.0;
constexpr double KD_MAX = 5.0;

class CubemarsDriver : public motor_interface::MotorDriver {
public:
    CubemarsDriver(uint8_t id, double gear_ratio, double zero_offset, uint32_t pulse_per_revolution)
    : motor_interface::MotorDriver(id, gear_ratio, zero_offset, pulse_per_revolution) {}

    virtual ~CubemarsDriver() = default;

    std::size_t encode(const motor_interface::motor_command_t& command, uint8_t* buffer, std::size_t size) override {
        if (buffer == nullptr) throw std::runtime_error("[CubemarsDriver::encode] Null buffer.");

        if (size < TX_PACKET_SIZE) throw std::runtime_error("[CubemarsDriver::encode] Invalid buffer size.");

        const uint16_t position_raw = static_cast<uint16_t>(position(command.position));
        const uint16_t velocity_raw = static_cast<uint16_t>(velocity(command.velocity));
        const uint16_t torque_raw = static_cast<uint16_t>(torque(command.torque));
        const uint16_t kp_raw = static_cast<uint16_t>(kp(command.kp));
        const uint16_t kd_raw = static_cast<uint16_t>(kd(command.kd));

        write_u16_big_endian(buffer, position_raw);
        buffer[2] = static_cast<uint8_t>(velocity_raw >> 4);
        buffer[3] = static_cast<uint8_t>(((velocity_raw & 0x0F) << 4) | ((kp_raw >> 8) & 0x0F));
        buffer[4] = static_cast<uint8_t>(kp_raw);
        buffer[5] = static_cast<uint8_t>(kd_raw >> 4);
        buffer[6] = static_cast<uint8_t>(((kd_raw & 0x0F) << 4) | ((torque_raw >> 8) & 0x0F));
        buffer[7] = static_cast<uint8_t>(torque_raw);

        return TX_PACKET_SIZE;
    }

    void decode(const uint8_t* buffer, std::size_t size, motor_interface::motor_state_t& status) override {
        if (buffer == nullptr) throw std::runtime_error("[CubemarsDriver::decode] Null buffer.");

        if (size != RX_PACKET_SIZE) throw std::runtime_error("[CubemarsDriver::decode] Invalid buffer size.");

        const uint8_t received_id = buffer[0];
        if (received_id != id_) throw std::runtime_error("[CubemarsDriver::decode] Motor ID mismatch.");

        const uint16_t position_raw = read_u16_big_endian(buffer + 1);
        const uint16_t velocity_raw = (static_cast<uint16_t>(buffer[3]) << 4) | (static_cast<uint16_t>(buffer[4]) >> 4);
        const uint16_t torque_raw = ((static_cast<uint16_t>(buffer[4]) & 0x0F) << 8) |
                                    static_cast<uint16_t>(buffer[5]);
        const uint8_t temperature_raw = buffer[6];
        const uint8_t error = buffer[7];

        status.position = position(position_raw);
        status.velocity = velocity(velocity_raw);
        status.torque = torque(torque_raw);
        status.temperature = static_cast<double>(temperature_raw) - 40.0;
        status.error = error;
    }

    std::size_t enable(uint8_t* buffer, std::size_t size) {
        if (size < TX_PACKET_SIZE) throw std::runtime_error("[CubemarsDriver::enable] Invalied buffer size.");

        for (int i = 0; i < TX_PACKET_SIZE-1; ++i) buffer[i] = 0xFF;
        buffer[TX_PACKET_SIZE-1] = 0xFC;

        return TX_PACKET_SIZE;
    }

    std::size_t disable(uint8_t* buffer, std::size_t size) {
        if (size < TX_PACKET_SIZE) throw std::runtime_error("[CubemarsDriver::disable] Invalied buffer size.");

        for (int i = 0; i < TX_PACKET_SIZE-1; ++i) buffer[i] = 0xFF;
        buffer[TX_PACKET_SIZE-1] = 0xFD;

        return TX_PACKET_SIZE;
    }

private:
    static uint16_t float_to_uint(double value, double min, double max, uint8_t bits) {
        value = clamp(value, min, max);

        const double span = max - min;
        const uint32_t max_raw = (1U << bits) - 1U;

        return static_cast<uint16_t>((value - min) * static_cast<double>(max_raw) / span);
    }

    static double uint_to_float(uint16_t value, double min, double max, uint8_t bits) {
        const double span = max - min;

        const uint32_t max_raw = (1U << bits) - 1U;

        return static_cast<double>(value) * span / static_cast<double>(max_raw) + min;
    }

    uint16_t position(const double& value) {
        const double position = value + zero_offset_;
        return float_to_uint(position, POSITION_MIN, POSITION_MAX, 16);
    }

    uint16_t velocity(const double& value) {
        return float_to_uint(value, VELOCITY_MIN, VELOCITY_MAX, 12);
    }

    uint16_t torque(const double& value) {
        return float_to_uint(value, TORQUE_MIN, TORQUE_MAX, 12);
    }

    uint16_t kp(const double& value) {
        return float_to_uint(value, KP_MIN, KP_MAX, 12);
    }

    uint16_t kd(const double& value) {
        return float_to_uint(value, KD_MIN, KD_MAX, 12);
    }

    double position(const uint16_t& value) {
        const double position = uint_to_float(value, POSITION_MIN, POSITION_MAX, 16);
        return position - zero_offset_;
    }

    double velocity(const uint16_t& value) {
        return uint_to_float(value, VELOCITY_MIN, VELOCITY_MAX, 12);
    }

    double torque(const uint16_t& value) {
        return uint_to_float(value, TORQUE_MIN, TORQUE_MAX, 12);
    }
};

} // namespace cubemars

#endif // MOTOR_DRIVER_CUBEMARS_DRIVER_HPP_
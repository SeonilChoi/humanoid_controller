#ifndef MOTOR_DRIVER_UNITREE_DRIVER_HPP_
#define MOTOR_DRIVER_UNITREE_DRIVER_HPP_

#include <cmath>
#include <cstdint>
#include <cstddef>
#include <stdexcept>

#include "motor/core/motor_driver.hpp"

namespace unitree {

constexpr std::size_t TX_PACKET_SIZE = 17;
constexpr std::size_t RX_PACKET_SIZE = 16;

constexpr uint8_t MODE_FOC = 1;
constexpr uint8_t TIMEOUT_ENABLE = 0;

constexpr double PI = 3.14159265358979323846;

class UnitreeDriver : public motor_interface::MotorDriver {
public:
    UnitreeDriver(uint8_t id, double gear_ratio, double zero_offset, uint32_t pulse_per_revolution)
    : motor_interface::MotorDriver(id, gear_ratio, zero_offset, pulse_per_revolution) {
        if (id > 14) throw std::runtime_error("[UnitreeDriver::UnitreeDriver] Motor ID must be 0-14.");
    }

    ~UnitreeDriver() override = default;

    std::size_t encode(const motor_interface::motor_command_t& command, uint8_t* buffer, std::size_t size) override {
        if (buffer == nullptr) throw std::runtime_error("[UnitreeDriver::encode] Null buffer.");
                
        if (size < TX_PACKET_SIZE) throw std::runtime_error("[UnitreeDriver::encode] Invalid buffer size.");
        
        buffer[0] = 0xFE;
        buffer[1] = 0xEE;
        buffer[2] = (id_ & 0x0F) | ((MODE_FOC & 0x07) << 4) | ((TIMEOUT_ENABLE & 0x01) << 7);
        
        int16_t torque_raw = torque(command.torque);
        int16_t velocity_raw = velocity(command.velocity);
        int32_t position_raw = position(command.position);
        int16_t kp_raw = kp(command.kp);
        int16_t kd_raw = kd(command.kd);

        write_s16_little_endian(buffer + 3, torque_raw);
        write_s16_little_endian(buffer + 5, velocity_raw);
        write_s32_little_endian(buffer + 7, position_raw);
        write_s16_little_endian(buffer + 11, kp_raw);
        write_s16_little_endian(buffer + 13, kd_raw);

        const uint16_t crc = crc_ccitt(0, buffer, 15);

        write_u16_little_endian(buffer + 15, crc);
        
        return TX_PACKET_SIZE;
    }

    void decode(const uint8_t* buffer, std::size_t size, motor_interface::motor_state_t& status) override {
        if (buffer == nullptr) throw std::runtime_error("[UnitreeDriver::decode] Null buffer.");

        if (size != RX_PACKET_SIZE) throw std::runtime_error("[UnitreeDriver::decode] Invalid buffer size.");

        if (buffer[0] != 0xFD || buffer[1] != 0xEE) throw std::runtime_error("[UnitreeDriver::decode] Invalid packet header.");

        const uint16_t received_crc = read_u16_little_endian(buffer + 14);
        const uint16_t calculated_crc = crc_ccitt(0, buffer, 14);
        if (received_crc != calculated_crc) throw std::runtime_error("[UnitreeDriver::decode] CRC mismatch.");

        const uint8_t received_id = buffer[2] & 0x0F;
        if (received_id != id_) throw std::runtime_error("[UnitreeDriver::decode] motor ID mismatch.");

        const uint8_t mode = (buffer[2] >> 4) & 0x07;
        const bool timeout = (buffer[2] >> 7) & 0x01;

        const int16_t torque_raw = read_s16_little_endian(buffer + 3);
        const int16_t velocity_raw = read_s16_little_endian(buffer + 5);
        const int32_t position_raw = read_s32_little_endian(buffer + 7);
        const int8_t temperature = static_cast<int8_t>(buffer[11]);
        
        const uint16_t error_foot_end_force = read_u16_little_endian(buffer + 12);
        const uint8_t error = static_cast<uint8_t>(error_foot_end_force & 0x07);
        const uint16_t foot_end_force = static_cast<uint16_t>((error_foot_end_force >> 3) & 0x0FFF);

        status.position = position(position_raw);
        status.velocity = velocity(velocity_raw);
        status.torque = torque(torque_raw);
        status.temperature = temperature;
        status.error = error;

        (void)mode;
        (void)foot_end_force;
        (void)timeout;
    }

private:
    static uint16_t crc_ccitt(uint16_t crc, const uint8_t* buffer, std::size_t size) {
        while (size--) {
            crc ^= *buffer++;

            for (uint8_t i = 0; i < 8; ++i) {
                if (crc & 1) {
                    crc = static_cast<uint16_t>((crc >> 1) ^ 0x8408);
                } else {
                    crc = static_cast<uint16_t>(crc >> 1);
                }
            }
        }

        return crc;
    }

    int32_t position(const double& value) override {
        double position = (value + zero_offset_) * gear_ratio_;
        return static_cast<int32_t>(position / (PI * 2) * pulse_per_revolution_);
    }

    int16_t velocity(const double& value) override {
        double velocity = value * gear_ratio_;
        return static_cast<int16_t>(clamp(velocity / (PI * 2) * 256.0, -pulse_per_revolution_, pulse_per_revolution_));
    }

    int16_t torque(const double& value) override {
        double torque = value / gear_ratio_;
        return static_cast<int16_t>(clamp(torque * 256.0, -pulse_per_revolution_, pulse_per_revolution_));
    }

    double position(const int32_t& value) override {
        double position = static_cast<double>(value) / pulse_per_revolution_ * (PI * 2);
        return position / gear_ratio_ - zero_offset_;
    }

    double velocity(const int16_t& value) override {
        double velocity = static_cast<double>(value) / 256.0 * (PI * 2);
        return velocity / gear_ratio_;
    }

    double torque(const int16_t& value) override {
        const double torque = static_cast<double>(value) / 256.0;
        return torque * gear_ratio_;
    }

    int16_t kp(const double& value) {
        return static_cast<int16_t>(clamp(value / 25.6 * pulse_per_revolution_, 0.0, pulse_per_revolution_));
    }

    int16_t kd(const double& value) {
        return static_cast<int16_t>(clamp(value / 25.6 * pulse_per_revolution_, 0.0, pulse_per_revolution_));
    }
};

}

#endif // MOTOR_DRIVER_UNITREE_DRIVER_HPP_
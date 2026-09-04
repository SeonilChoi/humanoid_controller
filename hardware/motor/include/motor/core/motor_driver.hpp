#ifndef MOTOR_CORE_MOTOR_DRIVER_HPP_
#define MOTOR_CORE_MOTOR_DRIVER_HPP_

#include <cstdint>

namespace motor_interface {

class MotorDriver {
public:
    explicit MotorDriver(uint8_t id, double gear_ratio, double zero_offset, uint32_t pulse_per_revolution)
    : id_(id),
      gear_ratio_(gear_ratio),
      zero_offset_(zero_offset),
      pulse_per_revolution_(pulse_per_revolution) {}

    virtual ~MotorDriver() = default;

    virtual std::size_t encode(const motor_command_t& command, uint8_t* buffer, std::size_t size) = 0;

    virtual void decode(const uint8_t* buffer, std::size_t size, motor_state_t& state) = 0;

    virtual std::size_t enable(uint8_t* buffer, std::size_t size) = 0;

    virtual std::size_t disable(uint8_t* buffer, std::size_t size) = 0;

protected:
    static double clamp(const double& value, const double& min, const double& max) {
        if (value < min) return min;
        if (value > max) return max;
        return value;
    }

    static void write_u16_little_endian(uint8_t* data, uint16_t value) {
        data[0] = static_cast<uint8_t>(value);
        data[1] = static_cast<uint8_t>(value >> 8);
    }

    static void write_s16_little_endian(uint8_t* data, int16_t value) {
        write_u16_little_endian(data, static_cast<uint16_t>(value));
    }

    static void write_u32_little_endian(uint8_t *data, uint32_t value) {
        data[0] = static_cast<uint8_t>(value);
        data[1] = static_cast<uint8_t>(value >> 8);
        data[2] = static_cast<uint8_t>(value >> 16);
        data[3] = static_cast<uint8_t>(value >> 24);
    }

    static void write_s32_little_endian(uint8_t *data, int32_t value) {
        write_u32_little_endian(data, static_cast<uint32_t>(value));
    }

    static uint16_t read_u16_little_endian(const uint8_t* data) {
        return static_cast<uint16_t>(data[0]) | (static_cast<uint16_t>(data[1]) << 8);
    }

    static int16_t read_s16_little_endian(const uint8_t* data) {
        return static_cast<int16_t>(read_u16_little_endian(data));
    }

    static uint32_t read_u32_little_endian(const uint8_t* data) {
        return static_cast<uint32_t>(data[0]) |
               (static_cast<uint32_t>(data[1]) << 8) |
               (static_cast<uint32_t>(data[2]) << 16) |
               (static_cast<uint32_t>(data[3]) << 24);
    }

    static int32_t read_s32_little_endian(const uint8_t* data) {
        return static_cast<int32_t>(read_u32_little_endian(data));
    }

    static void write_u16_big_endian(uint8_t* data, uint16_t value) {
        data[0] = static_cast<uint8_t>(value >> 8);
        data[1] = static_cast<uint8_t>(value);
    }

    static void write_s16_big_endian(uint8_t* data, int16_t value) {
        write_u16_big_endian(data, static_cast<uint16_t>(value));
    }

    static void write_u32_big_endian(uint8_t *data, uint32_t value) {
        data[0] = static_cast<uint8_t>(value >> 24);
        data[1] = static_cast<uint8_t>(value >> 16);
        data[2] = static_cast<uint8_t>(value >> 8);
        data[3] = static_cast<uint8_t>(value);
    }

    static void write_s32_big_endian(uint8_t *data, int32_t value) {
        write_u32_big_endian(data, static_cast<uint32_t>(value));
    }

    static uint16_t read_u16_big_endian(const uint8_t* data) {
        return (static_cast<uint16_t>(data[0]) << 8) | static_cast<uint16_t>(data[1]);
    }

    static int16_t read_s16_big_endian(const uint8_t* data) {
        return static_cast<int16_t>(read_u16_big_endian(data));
    }

    static uint32_t read_u32_big_endian(const uint8_t* data) {
        return (static_cast<uint32_t>(data[0]) << 24) |
               (static_cast<uint32_t>(data[1]) << 16) |
               (static_cast<uint32_t>(data[2]) << 8) |
               static_cast<uint32_t>(data[3]);
    }

    static int32_t read_s32_big_endian(const uint8_t* data) {
        return static_cast<int32_t>(read_u32_big_endian(data));
    }

    const uint8_t id_;

    const double gear_ratio_;

    const double zero_offset_;

    const uint32_t pulse_per_revolution_;
};

} // namespace motor_interface

#endif // MOTOR_CORE_MOTOR_DRIVER_HPP_
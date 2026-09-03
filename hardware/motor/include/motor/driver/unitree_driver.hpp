#ifndef MOTOR_DRIVER_UNITREE_DRIVER_HPP_
#define MOTOR_DRIVER_UNITREE_DRIVER_HPP_

namespace unitree {

class UnitreeDriver : public motor_interface::MotorDriver {
public:
    UnitreeDriver(double gear_ratio, double zero_offset)
    : motor_interface::MotorDriver(gear_ratio, zero_offset) {}

    virtual ~UnitreeDriver() = default;

    std::size_t encode(const motor_interface::motor_command_t& command, uint8_t* buffer, std::size_t size) override {
        if (size < 17) throw std::runtime_error("[UnitreeDriver::encode] Invalid buffer size.");

        return 17;
    }

    void decode(const uint8_t* buffer, std::size_t size, motor_interface::motor_state_t& status) override {
        if (size != 16) throw std::runtime_error("[UnitreeDriver::decode] Invalid buffer size.");
    }
};

}

#endif // MOTOR_DRIVER_UNITREE_DRIVER_HPP_
#ifndef MOTOR_MOTOR_MANAGER_HPP_
#define MOTOR_MOTOR_MANAGER_HPP_

#include <string>

#include "motor/motor_interface.hpp"

namespace motor_manager {

class MotorManager {
public:
    explicit MotorManager(const std::string& config_file);

    virtual ~MotorManager() = default;

    void run();

    void write(const motor_interface::motor_command_t* command);

    void read(motor_interface::motor_state_t* status);
};

} // namespace motor_manager
#endif
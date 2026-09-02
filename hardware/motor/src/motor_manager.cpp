#include <iostream>

#include "motor/motor_manager.hpp"

motor_manager::MotorManager::MotorManager(const std::string& config_file) {
    std::cout << config_file << std::endl;
}

void motor_manager::MotorManager::run() {
    std::cout << "MotorManager is running..." << std::endl;
}

void motor_manager::MotorManager::write(const motor_interface::motor_command_t* command) {
    std::cout << command[0].position << std::endl;
}

void motor_manager::MotorManager::read(motor_interface::motor_state_t* status) {
    status[0].position = 1.0;
}
#include <string>
#include <thread>
#include <chrono>
#include <iostream>

#include "motor/core/motor_interface.hpp"
#include "motor/motor_manager.hpp"

int main(int argc, char* argv[])
{
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <config_file>" << std::endl;
        return 1;
    }

    const std::string config_file = argv[1];

    motor_manager::MotorManager motor_manager(config_file);

    motor_manager.start();

    motor_interface::motor_command_t command[2]{};
    command[0].position = 1.57;
    command[0].velocity = 0.0;
    command[0].torque = 0.0;
    command[0].kp = 0.1;
    command[0].kd = 0.1;

    command[1].position = 1.57;
    command[1].velocity = 0.0;
    command[1].torque = 0.0;
    command[1].kp = 0.1;
    command[1].kd = 0.1;

    motor_interface::motor_state_t status[2]{};

    for (int i = 0; i < 1000; i++) {
        motor_manager.write(command);

        motor_manager.read(status);
        std::cout << "Motor ID 0 " << status[0].position << std::endl;
        std::cout << "Motor ID 1 " << status[1].position << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    motor_manager.stop();
    
    return 0;
}
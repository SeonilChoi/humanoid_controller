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

    motor_interface::motor_command_t command[6]{};

    command[0].position = 1.5;
    command[0].velocity = 0.0;
    command[0].torque = 0.0;
    command[0].kp = 0.2;
    command[0].kd = 0.1;

    command[1].position = 0.0;
    command[1].velocity = 0.0;
    command[1].torque = 0.0;
    command[1].kp = 0.2;
    command[1].kd = 0.1;

    command[2].position = 1.5;
    command[2].velocity = 0.0;
    command[2].torque = 0.0;
    command[2].kp = 1.0;
    command[2].kd = 0.5;

    command[3].position = 1.5;
    command[3].velocity = 0.0;
    command[3].torque = 0.0;
    command[3].kp = 1.0;
    command[3].kd = 0.5;

    command[4].position = 0.0;
    command[4].velocity = 0.0;
    command[4].torque = 0.0;
    command[4].kp = 0.2;
    command[4].kd = 0.1;

    command[5].position = 1.5;
    command[5].velocity = 0.0;
    command[5].torque = 0.0;
    command[5].kp = 0.2;
    command[5].kd = 0.1;

    motor_interface::motor_state_t status[6]{};

    for (int i = 0; i < 1000; i++) {
        motor_manager.write(command);

        motor_manager.read(status);
        std::cout << "Motor ID 0 " << status[0].position << std::endl;
        std::cout << "Motor ID 1 " << status[1].position << std::endl;
        std::cout << "Motor ID 2 " << status[2].position << std::endl;
        std::cout << "Motor ID 3 " << status[3].position << std::endl;
        std::cout << "Motor ID 4 " << status[4].position << std::endl;
        std::cout << "Motor ID 5 " << status[5].position << std::endl;

        if (i % 100 == 0) {
            double tmp = command[1].position;
            command[1].position = command[0].position;
            command[3].position = command[0].position;
            command[5].position = command[0].position;

            command[0].position = tmp;
            command[2].position = tmp;
            command[4].position = tmp;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    motor_manager.stop();
    
    return 0;
}


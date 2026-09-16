#include <thread>
#include <chrono>
#include <iostream>

#include "motor/motor_manager.hpp"
#include "motor/core/motor_interface.hpp"

int main(int argc, char* argv[])
{
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << "<config_file>" << std::endl;
        return 1;
    }

    motor_manager::MotorManager motor_manager(argv[1]);

    int n_motors = motor_manager.number_of_motors();

    motor_interface::motor_command_t command[motor_interface::MAX_MOTORS]{};
    for (int i = 0; i < n_motors; ++i) {
        command[i].kp = 0.5;
        command[i].kd = 0.2;
    }

    motor_interface::motor_state_t status[motor_interface::MAX_MOTORS]{};

    motor_manager.start();

    double position = 0.0;
    
    while (True) {
        std::cout << "Enter position(rad): "
        std::cin >> position;
        std::cout << std::endl;

        if (position == -99.0) break;

        for (int i = 0; i < n_motors; ++i) {
            command[i].position = position;
        }
        motor_manager.write(command);

        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        motor_manager.read(status);
        for (int i = 0; i < n_motors; ++i) {
            std::cout << "Motor Index: " << i << " Position: " << status[i].position << std::endl;
        }
    }

    motor_manager.stop();

    return 0;
}
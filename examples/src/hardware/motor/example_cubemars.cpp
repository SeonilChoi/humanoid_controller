#include <thread>
#include <chrono>
#include <csignal>
#include <iostream>

#include "motor/motor_manager.hpp"
#include "motor/core/motor_interface.hpp"

namespace {
volatile std::sig_atomic_t running = 1;
void on_sigint(int) { running = 0; }
}

int main(int argc, char* argv[])
{
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <config_file>" << " <goal_position>" << std::endl;
        return 1;
    }

    std::signal(SIGINT, on_sigint);

    motor_manager::MotorManager motor_manager(argv[1]);
    double goal_position = std::stod(argv[2]);

    int n_motors = motor_manager.number_of_motors();
    motor_interface::motor_command_t command[motor_interface::MAX_MOTORS]{};
    for (int i = 0; i < n_motors; ++i) {
        command[i].position = goal_position;
        command[i].kp = 10.0;
        command[i].kd = 5.0;
    }
    motor_interface::motor_state_t status[motor_interface::MAX_MOTORS]{};
    

    motor_manager.start();
    motor_manager.write(command);

    while (running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        motor_manager.read(status);
        for (int i = 0; i < n_motors; ++i) {
            std::cout << "Motor Index: " << i << " Position: " << status[i].position << std::endl;
        }
        std::cout << std::endl;

    }

    motor_manager.stop();

    return 0;
}
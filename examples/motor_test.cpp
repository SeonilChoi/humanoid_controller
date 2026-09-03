#include <string>
#include <thread>
#include <chrono>
#include <iostream>

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

    std::this_thread::sleep_for(std::chrono::seconds(5));

    motor_manager.stop();
    
    return 0;
}
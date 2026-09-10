#include <thread>
#include <chrono>
#include <iostream>

#include "joy/core/joy_interface.hpp"
#include "joy/joy_handler.hpp"

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <config file>" << std::endl;
        return 1;
    }

    joy_handler::JoyHandler joy_handler(argv[1]);
    joy_handler.start();

    while (true) {
        joy_interface::joy_data_t data;
        joy_handler.read(data);
        std::cout << "Cross: " << data.cross << std::endl;
        std::cout << "Circle: " << data.circle << std::endl;
        std::cout << "Triangle: " << data.triangle << std::endl;
        std::cout << "Square: " << data.square << std::endl;
        std::cout << "L1: " << data.l1 << std::endl;
        std::cout << "R1: " << data.r1 << std::endl;
        std::cout << "L2: " << data.l2 << std::endl;
        std::cout << "R2: " << data.r2 << std::endl;
        std::cout << "Select: " << data.select << std::endl;
        std::cout << "Start: " << data.start << std::endl;
        std::cout << "Center: " << data.center << std::endl;
        std::cout << "Stick Lx: " << data.stick_lx << std::endl;
        std::cout << "Stick Ly: " << data.stick_ly << std::endl;
        std::cout << "Stick Rx: " << data.stick_rx << std::endl;
        std::cout << "Stick Ry: " << data.stick_ry << std::endl;
        std::cout << "L2 Analog: " << data.l2_analog << std::endl;
        std::cout << "R2 Analog: " << data.r2_analog << std::endl;
        std::cout << "Dpad X: " << data.dpad_x << std::endl;
        std::cout << "Dpad Y: " << data.dpad_y << std::endl;
        std::cout << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    joy_handler.stop();

    return 0;
}
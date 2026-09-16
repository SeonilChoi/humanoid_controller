#include <thread>
#include <chrono>
#include <iostream>

#include "robot/robots/ARTI_H1.hpp"

int main(int argc, char* argv[])
{
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <config_file>" << std::endl;
        return 1;
    }

    const std::string config_file = argv[1];

    arti::ArtiH1 robot(config_file);

    robot.start();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    robot.control();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    for (int i = 0; i < 1000; ++i) {
        robot.observation();
        
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    
        robot.control();
    }

    robot.stop();
    return 0;
}
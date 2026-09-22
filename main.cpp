#include <thread>
#include <chrono>
#include <csignal>
#include <iostream>

#include "robot/robots/ARTI_H1.hpp"

namespace {
volatile std::sig_atomic_t running = 1;
void on_sigint(int) { running = 0; }
}

int main(int argc, char* argv[])
{
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <config_file>" << std::endl;
        return 1;
    }

    std::signal(SIGINT, on_sigint);

    const std::string config_file = argv[1];

    robot::ArtiH1 robot(config_file);

    robot.start();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    robot.initialize();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    while (running) {
        robot.observation();
        
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    
        robot.control();
    }

    robot.stop();
    return 0;
}
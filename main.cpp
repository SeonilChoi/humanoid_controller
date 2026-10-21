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

    try {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        robot.initialize();
        std::this_thread::sleep_for(std::chrono::milliseconds(10000));

        constexpr auto kControlPeriod = std::chrono::microseconds(1'000'000 / 60);
        auto next_wakeup = std::chrono::steady_clock::now();

        while (running) {
            robot.observation();
            robot.control();

            next_wakeup += kControlPeriod;
            const auto now = std::chrono::steady_clock::now();
            if (next_wakeup < now) {
                next_wakeup = now;
            } else {
                std::this_thread::sleep_until(next_wakeup);
            }
        }
    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        robot.stop();
        return 1;
    }

    robot.stop();
    return 0;
}
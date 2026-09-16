#include <thread>
#include <chrono>
#include <csignal>
#include <iostream>

#include "sensor/sensor_manager.hpp"
#include "sensor/core/imu.hpp"
#include "sensor/core/sensor_interface.hpp"

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

    sensor_manager::SensorManager sensor_manager(argv[1]);

    sensor_manager.start();

    auto& imu = sensor_manager.sensor<sensor_interface::Imu>(0);

    sensor_interface::imu_data_t data{};

    while (running) {
        imu.read(data);

        std::cout << "Xsens MTi orientation: "
                  << data.orientation[0] << " "
                  << data.orientation[1] << " "
                  << data.orientation[2] << " "
                  << data.orientation[3] << std::endl;

        std::cout << "Xsens MTi angular velocity: "
                  << data.angular_velocity[0] << " "
                  << data.angular_velocity[1] << " "
                  << data.angular_velocity[2] << std::endl;

        std::cout << "Xsens MTi linear acceleration: "
                  << data.linear_acceleration[0] << " "
                  << data.linear_acceleration[1] << " "
                  << data.linear_acceleration[2] << std::endl;

        std::cout << "Xsens MTi temperature: "
                  << data.temperature << std::endl;

        std::cout << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    sensor_manager.stop();

    return 0;
}
#include <string>
#include <thread>
#include <chrono>
#include <iostream>

#include "sensor/core/sensor_interface.hpp"
#include "sensor/core/imu.hpp"
#include "sensor/sensor_manager.hpp"

int main(int argc, char* argv[])
{
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <config_file>" << std::endl;
        return 1;
    }

    const std::string config_file = argv[1];

    sensor_manager::SensorManager sensor_manager(config_file);

    sensor_manager.start();

    auto& imu = sensor_manager.sensor<sensor_interface::Imu>(0);

    sensor_interface::imu_data_t data{};

    for (int i = 0; i < 1000; i++) {
        imu.read(data);

        std::cout << "Xsens MTi orientation: "
                  << data.orientation[0] << " "
                  << data.orientation[1] << " "
                  << data.orientation[2] << " "
                  << data.orientation[3] << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    sensor_manager.stop();

    return 0;
}
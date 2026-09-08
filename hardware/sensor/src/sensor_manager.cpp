#include <chrono>
#include <thread>
#include <iostream>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

#include "sensor/sensor_manager.hpp"
#include "sensor/imu/xsens_mti.hpp"

sensor_manager::SensorManager::SensorManager(const std::string& config_file) {
    load(config_file);
}

sensor_manager::SensorManager::~SensorManager() {
    stop();
}

void sensor_manager::SensorManager::start() {
    if (running_.load()) return;

    running_.store(true);

    for (auto& [index, sensor] : sensors_) {
        threads_.emplace(index, std::thread(&SensorManager::run, this, index));
    }
}

void sensor_manager::SensorManager::stop() {
    if (!running_.load() && threads_.empty()) return;

    running_.store(false);

    for (auto& [index, thread] : threads_) {
        if (thread.joinable()) {
            thread.join();
        }
    }

    threads_.clear();

    for (auto& [index, sensor] : sensors_) {
        try {
            sensor->shutdown();
        } catch (const std::exception& e) {
            std::cerr << "[SensorManager::stop] " << e.what() << std::endl;
        }
    }
}

void sensor_manager::SensorManager::load(const std::string& config_file) {
    YAML::Node root = YAML::LoadFile(config_file);
    if (!root) throw std::runtime_error("[SensorManager::load] Failed to load configuration file: " + config_file);

    YAML::Node sensors = root["sensors"];
    if (!sensors || !sensors.IsSequence()) throw std::runtime_error("[SensorManager::load] Invalied sensors configuration");

    for (const auto& s : sensors) {
        const unsigned int idx = s["index"].as<unsigned int>();
	const uint8_t sensor_index = static_cast<uint8_t>(idx);

        if (sensors_.find(sensor_index) != sensors_.end()) {
            throw std::runtime_error("[SensorManager::load] Duplicate sensor index found: " + std::to_string(sensor_index));
        }

        const std::string sensor = s["sensor"].as<std::string>();
        const std::string type = s["type"].as<std::string>();
        const std::string device = s["device"].as<std::string>();
        const uint32_t baudrate = s["baudrate"].as<uint32_t>();
        const uint32_t period = s["period"].as<uint32_t>();

        if (sensor == "imu") {
            if (type == "xsens_mti") {
                sensors_[sensor_index] = std::make_unique<xsens_mti::XsensMti>(period, device, baudrate);
	    }
        }

        sensors_[sensor_index]->initialize();
    }
}

void sensor_manager::SensorManager::run(const uint8_t index) {
    try{
        auto& sensor = *sensors_.at(index);

        const auto cycle = std::chrono::microseconds(sensor.period());
        auto next_wakeup = std::chrono::steady_clock::now();

        while (running_.load()) {
            next_wakeup += cycle;
            std::this_thread::sleep_until(next_wakeup);

            sensor.update();

            const auto now = std::chrono::steady_clock::now();
            if (next_wakeup < now - cycle) next_wakeup = now;
        }

    } catch (const std::exception& e) {
        std::cerr << "[SensorManager::run] " << e.what() << std::endl;
        running_.store(false);
    }
}

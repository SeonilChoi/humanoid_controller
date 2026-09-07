#ifndef SENSOR_SENSOR_MANAGER_HPP_
#define SENSOR_SENSOR_MANAGER_HPP_

#include <string>
#include <atomic>
#include <memory>
#include <thread>
#include <cstdint>
#include <stdexcept>
#include <unordered_map>

#include "sensor/core/sensor.hpp"

namespace sensor_manager {

class SensorManager {
public:
    explicit SensorManager(const std::string& config_file);

    virtual ~SensorManager();

    void start();

    void stop();

    template<typename T>
    T& sensor(uint8_t id) {
        auto it = sensors_.find(id);

        if (it == sensors_.end()) throw std::runtime_error("[SensorManager::sensor] Sensor not found.");

        auto* sensor = dynamic_cast<T*>(it->second.get());

        if (sensor == nullptr) throw std::runtime_error("[SensorManager::sensor] Sensor null.");

        return *sensor;
    }

private:
    void load(const std::string& config_file);

    void run(uint8_t id);

    std::atomic<bool> running_{false};

    std::unordered_map<uint8_t, std::unique_ptr<sensor_interface::Sensor>> sensors_;

    std::unordered_map<uint8_t, std::thread> threads_;
};

} // namespace sensor_manager

#endif // SENSOR_SENSOR_MANAGER_HPP_
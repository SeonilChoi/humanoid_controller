#include <iostream>
#include <string>
#include <cstdint>
#include <stdexcept>
#include <thread>
#include <chrono>

#include <yaml-cpp/yaml.h>

#include "motor/motor_manager.hpp"
#include "motor/master/unitree_master.hpp"
#include "motor/master/cubemars_master.hpp"

motor_manager::MotorManager::MotorManager(const std::string& config_file) {
    load(config_file);
}

void motor_manager::MotorManager::start() {
    running_.store(true);

    for (auto& [id, master] : masters_) {
        threads_.emplace(id, std::thread(&MotorManager::run, this, id));
    }
}

void motor_manager::MotorManager::stop() {
    running_.store(false);

    for (auto& [id, thread] : threads_) {
        if (thread.joinable()) {
            thread.join();
        }
    }
    threads_.clear();

    for (auto& [id, master] : masters_) {
        try {
            master->shutdown();
        } catch (const std::exception& e) {
            std::cerr << "[MotorManager::stop] " << e.what() << std::endl;
        }
    }
}

void motor_manager::MotorManager::write(const motor_interface::motor_command_t* command) {
    std::lock_guard<std::mutex> lock(mutex_);
    for (uint8_t i = 0; i < number_of_motors_; ++i) {
        command_[i] = command[i];
    }
}

void motor_manager::MotorManager::read(motor_interface::motor_state_t* status) {
    std::lock_guard<std::mutex> lock(mutex_);
    for (uint8_t i = 0; i < number_of_motors_; ++i) {
        status[i] = status_[i];
    }
}

void motor_manager::MotorManager::load(const std::string& config_file) {
    YAML::Node root = YAML::LoadFile(config_file);
    if (!root) throw std::runtime_error("[MotorManager::load] Failed to load configuration file: " + config_file);

    YAML::Node masters = root["masters"];
    if (!masters || !masters.IsSequence()) throw std::runtime_error("[MotorManager::load] Invalid masters configuration.");

    for (const auto& m : masters) {
        uint8_t master_id = m["id"].as<uint8_t>();
        uint32_t period = m["period"].as<uint32_t>();

        if (masters_.find(master_id) != masters_.end()) {
            throw std::runtime_error("Duplicate master ID found: " + std::to_string(master_id));
        }

        std::string type = m["type"].as<std::string>();
        if (type == "unitree") {
            std::string device = m["device"].as<std::string>();
            masters_[master_id] = std::make_unique<unitree::UnitreeMaster>(period, device);
        } else if (type == "cubemars") {
            std::string device = m["device"].as<std::string>();
            masters_[master_id] = std::make_unique<cubemars::CubemarsMaster>(period, device);
        }

        YAML::Node motors = m["motors"];
        if (!motors || !motors.IsSequence()) {
            throw std::runtime_error("Invalid motors configuration.");
        }

        for (const auto& motor : motors) {
            uint8_t motor_index = motor["index"].as<uint8_t>();
            uint8_t motor_id = motor["id"].as<uint8_t>();
            double gear_ratio = motor["gear_ratio"].as<double>();
            double zero_offset = motor["zero_offset"].as<double>();
            uint32_t pulse_per_revolution = motor["pulse_per_revolution"].as<uint32_t>();

            masters_[master_id]->add_motor(motor_id, gear_ratio, zero_offset, pulse_per_revolution);
            
            routes_[master_id].push_back({motor_index, motor_id});

            number_of_motors_++;
        }

        masters_[master_id]->initialize();
    }
}

void motor_manager::MotorManager::run(uint8_t id) {

    try{
    auto& master = *masters_.at(id);
    const auto& routes = routes_.at(id);

    const auto cycle = std::chrono::microseconds(master.period());
    auto next_wakeup = std::chrono::steady_clock::now();

    while (running_.load()) {
        next_wakeup += cycle;
        std::this_thread::sleep_until(next_wakeup);

        for (const auto& route : routes) {
            const uint8_t index = route.motor_index;
            const uint8_t motor_id = route.motor_id;

            motor_interface::motor_command_t command{};
            motor_interface::motor_state_t status{};
            
            {
                std::lock_guard<std::mutex> lock(mutex_);
                command = command_[index];
            }

            master.update(motor_id, command, status);
        
            {
                std::lock_guard<std::mutex> lock(mutex_);
                status_[index] = status;
            }
        }

        const auto now = std::chrono::steady_clock::now();
        if (next_wakeup < now - cycle) next_wakeup = now;
    }
    }
    catch (const std::exception& e) {
        std::cerr << "[MotorManager::run] " << e.what() << std::endl;
        running_.store(false);
    }
}
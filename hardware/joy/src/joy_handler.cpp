#include <chrono>
#include <iostream>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

#include "joy/joy_handler.hpp"
#include "joy/playstation/dualsense.hpp"

joy_handler::JoyHandler::~JoyHandler() {
    stop();
}

void joy_handler::JoyHandler::initialize() {
    joy_->initialize();
}

void joy_handler::JoyHandler::start() {
    if (running_.load()) return;

    running_.store(true);

    thread_ = std::thread(&JoyHandler::run, this);
}

void joy_handler::JoyHandler::stop() {
    if (!running_.load() && !thread_.joinable()) return;

    running_.store(false);

    if (thread_.joinable()) {
        thread_.join();
    }

    try {
        joy_->shutdown();
    } catch (const std::exception& e) {
        std::cerr << "[JoyHandler::stop] " << e.what() << std::endl;
    }
}

void joy_handler::JoyHandler::read(joy_interface::joy_data_t& data) {
    joy_->read(data);
}

void joy_handler::JoyHandler::load(const std::string& config_file) {
    YAML::Node config = YAML::LoadFile(config_file);

    const std::string type = config["type"].as<std::string>();

    const std::string device = config["device"].as<std::string>();

    const uint32_t period = config["period"].as<uint32_t>();

    const std::string axis_layout = config["axis_layout"].as<std::string>();
    
    if (type == "dualsense") {
        joy_ = std::make_unique<playstation::DualSense>(device, period, axis_layout);
    }
}

void joy_handler::JoyHandler::run() {
    try {
        const auto cycle = std::chrono::microseconds(joy_->period());
        auto next_wakeup = std::chrono::steady_clock::now();

        while (running_.load()) {
            next_wakeup += cycle;
            std::this_thread::sleep_until(next_wakeup);
            
            joy_->update();

            const auto now = std::chrono::steady_clock::now();
            if (next_wakeup < now - cycle) next_wakeup = now;
        }

    } catch (const std::exception& e) {
        std::cerr << "[JoyHandler::run] " << e.what() << std::endl;
        running_.store(false);
    }
}
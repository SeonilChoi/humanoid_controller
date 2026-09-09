#include <yaml-cpp/yaml.h>

#include "robot/core/robot.hpp"

robot::Robot::Robot(const std::string& config_file)
: config_file_(config_file) {
    load(config_file);

    motor_manager_ = std::make_unique<motor_manager::MotorManager>(motor_manager_config_file_);

    sensor_manager_ = std::make_unique<sensor_manager::SensorManager>(sensor_manager_config_file_);

    kinematics_ = std::make_unique<kinematics::Kinematics>(urdf_file_);
}

void robot::Robot::load(const std::string& config_file) {
    const YAML::Node root = YAML::LoadFile(config_file);

    if (!root || !root.IsMap()) {
        throw std::runtime_error("[Robot::load] Invalid config file.");
    }

    name_ = root["name"].as<std::string>();

    motor_manager_config_file_ = root["motor_manager_config_file"].as<std::string>();

    sensor_manager_config_file_ = root["sensor_manager_config_file"].as<std::string>();

    urdf_file_ = root["urdf_file"].as<std::string>();

    const YAML::Node motors = root["motors"];

    if (!motors || !motors.IsSequence()) {
        throw std::runtime_error("[Robot::load] Invalid motors.");
    }

    for (const auto& motor : motors) {
        motor_config_t config;

        config.name = motor["name"].as<std::string>();
        config.index = motor["index"].as<uint8_t>();

        motors_.push_back(config);
    }

    const YAML::Node sensors = root["sensors"];

    if (!sensors || !sensors.IsSequence()) {
        throw std::runtime_error("[Robot::load] Invalid sensors.");
    }

    for (const auto& sensor : sensors) {
        sensor_config_t config;

        config.index = sensor["index"].as<uint8_t>();
        config.name = sensor["name"].as<std::string>();
        config.frame = sensor["frame"].as<std::string>();
        config.translation = sensor["translation"].as<std::vector<double>>();
        config.orientation = sensor["orientation"].as<std::vector<double>>();
        
        sensors_.push_back(config);
    }

    const YAML::Node foots = root["foots"];

    if (!foots || !foots.IsSequence()) {
        throw std::runtime_error("[Robot::load] Invalid foots.");
    }

    for (const auto& foot : foots) {
        foot_config_t config;

        config.name = foot["name"].as<std::string>();
        config.offset = foot["offset"].as<std::vector<double>>();

        foots_.push_back(config);
    }
}

void robot::Robot::start() {
    motor_manager_->start();
    sensor_manager_->start();
}

void robot::Robot::stop() {
    motor_manager_->stop();
    sensor_manager_->stop();
}
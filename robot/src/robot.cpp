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

    name_ = root["name"].as<std::string>();

    motor_manager_config_file_ = root["motor_manager_config_file"].as<std::string>();

    sensor_manager_config_file_ = root["sensor_manager_config_file"].as<std::string>();

    urdf_file_ = root["urdf_file"].as<std::string>();
}

void robot::Robot::start() {
    motor_manager_->start();
    sensor_manager_->start();
}

void robot::Robot::stop() {
    motor_manager_->stop();
    sensor_manager_->stop();
}
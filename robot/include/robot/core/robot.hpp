#ifndef ROBOT_CORE_ROBOT_HPP_
#define ROBOT_CORE_ROBOT_HPP_

#include <memory>
#include <string>
#include <vector>
#include <cstdint>

#include <yaml-cpp/yaml.h>

#include "motor/motor_manager.hpp"
#include "sensor/sensor_manager.hpp"
#include "joy/joy_handler.hpp"

#include "motor/core/motor_interface.hpp"
#include "robot/core/kinematics.hpp"
#include "robot/core/controller.hpp"

namespace robot {

struct motor_config_t {
    uint8_t index;
    std::string name;
};

struct sensor_config_t {
    uint8_t index;
    std::string name;
    std::string frame;
    std::vector<double> translation;
    std::vector<double> orientation;
};

struct foot_config_t {
    std::string name;
    std::vector<double> offset;
};

class Robot {
public:
    explicit Robot(const std::string& config_file)
    : config_file_(config_file) {
        load(config_file);

        motor_manager_ = std::make_unique<motor_manager::MotorManager>(motor_manager_config_file_);

        sensor_manager_ = std::make_unique<sensor_manager::SensorManager>(sensor_manager_config_file_);

        joy_handler_ = std::make_unique<joy_handler::JoyHandler>(joy_handler_config_file_);

        kinematics_ = std::make_unique<kinematics::Kinematics>(urdf_file_);
    }

    virtual ~Robot() {
        controller_->shutdown();
    }

    void start() {
        motor_manager_->start();
        sensor_manager_->start();
        joy_handler_->start();
    } 

    void stop() {
        motor_manager_->stop();
        sensor_manager_->stop();
        joy_handler_->stop();
    }

    virtual void initialize() = 0;

    virtual void observation() = 0;

    virtual void control() = 0;

protected:
    void load(const std::string& config_file) {
        const YAML::Node root = YAML::LoadFile(config_file);

        if (!root || !root.IsMap()) {
            throw std::runtime_error("[Robot::load] Invalid config file.");
        }
    
        name_ = root["name"].as<std::string>();
    
        motor_manager_config_file_ = root["motor_manager_config_file"].as<std::string>();
    
        sensor_manager_config_file_ = root["sensor_manager_config_file"].as<std::string>();
    
        joy_handler_config_file_ = root["joy_handler_config_file"].as<std::string>();
    
        urdf_file_ = root["urdf_file"].as<std::string>();
    
        model_file_ = root["model_file"].as<std::string>();
    
        const YAML::Node motors = root["motors"];
    
        if (!motors || !motors.IsSequence()) {
            throw std::runtime_error("[Robot::load] Invalid motors.");
        }
    
        for (const auto& motor : motors) {
            motor_config_t config;
    
            config.index = static_cast<uint8_t>(motor["index"].as<unsigned int>());
            config.name = motor["name"].as<std::string>();
    
            motors_.push_back(config);
        }
    
        const YAML::Node sensors = root["sensors"];
    
        if (!sensors || !sensors.IsSequence()) {
            throw std::runtime_error("[Robot::load] Invalid sensors.");
        }
    
        for (const auto& sensor : sensors) {
            sensor_config_t config;
    
            config.index = static_cast<uint8_t>(sensor["index"].as<unsigned int>());
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

    std::unique_ptr<motor_manager::MotorManager> motor_manager_;

    std::unique_ptr<sensor_manager::SensorManager> sensor_manager_;

    std::unique_ptr<joy_handler::JoyHandler> joy_handler_;

    std::unique_ptr<kinematics::Kinematics> kinematics_;

    std::unique_ptr<controller::Controller> controller_;

    std::string name_;

    std::string motor_manager_config_file_;

    std::string sensor_manager_config_file_;

    std::string joy_handler_config_file_;

    std::string urdf_file_;

    std::string model_file_;

    std::vector<motor_config_t> motors_;

    std::vector<sensor_config_t> sensors_;

    std::vector<foot_config_t> foots_;

    std::vector<double> observation_;

    std::vector<double> action_;

    motor_interface::motor_command_t motor_command_[motor_interface::MAX_MOTORS]{};

private:
    const std::string config_file_;
};

} // namespace robot

#endif // ROBOT_CORE_ROBOT_HPP_
#ifndef ROBOT_CORE_ROBOT_HPP_
#define ROBOT_CORE_ROBOT_HPP_

#include <memory>
#include <string>
#include <vector>

#include "motor/motor_manager.hpp"
#include "sensor/sensor_manager.hpp"
#include "robot/core/kinematics.hpp"

namespace robot {

class Robot {
public:
    explicit Robot(const std::string& config_file);

    virtual ~Robot() = default;

    virtual void start();

    virtual void stop();

    virtual const std::vector<double>& observation() = 0;

    virtual void control() = 0;

protected:
    void load(const std::string& config_file);    

    std::unique_ptr<motor_manager::MotorManager> motor_manager_;

    std::unique_ptr<sensor_manager::SensorManager> sensor_manager_;

    std::unique_ptr<kinematics::Kinematics> kinematics_;

    std::vector<double> observation_;

    std::string name_;

    std::string motor_manager_config_file_;

    std::string sensor_manager_config_file_;

    std::string urdf_file_;

private:
    const std::string config_file_;
};

} // namespace robot

#endif // ROBOT_CORE_ROBOT_HPP_
#ifndef MOTOR_MOTOR_MANAGER_HPP_
#define MOTOR_MOTOR_MANAGER_HPP_

#include <mutex>
#include <string>
#include <cstdint>
#include <memory>
#include <thread>
#include <atomic>
#include <vector>
#include <unordered_map>

#include "motor/core/motor_interface.hpp"
#include "motor/core/motor_master.hpp"

namespace motor_manager {

class MotorManager {
public:
    explicit MotorManager(const std::string& config_file);

    virtual ~MotorManager() = default;

    void start();

    void stop();

    void write(const motor_interface::motor_command_t* command);

    void read(motor_interface::motor_state_t* status);

private:
    void load(const std::string& config_file);

    void run(uint8_t id);

    std::unordered_map<uint8_t, std::unique_ptr<motor_interface::MotorMaster>> masters_;

    std::unordered_map<uint8_t, std::thread> threads_;

    std::unordered_map<uint8_t, std::vector<motor_interface::motor_route_t>> routes_;

    std::mutex mutex_;

    motor_interface::motor_command_t command_[motor_interface::MAX_MOTORS]{};

    motor_interface::motor_state_t status_[motor_interface::MAX_MOTORS]{};

    std::atomic<bool> running_{false};

    uint8_t number_of_motors_{0};
};

} // namespace motor_manager
#endif
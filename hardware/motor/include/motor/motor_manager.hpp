#ifndef MOTOR_MOTOR_MANAGER_HPP_
#define MOTOR_MOTOR_MANAGER_HPP_

#include <mutex>
#include <string>
#include <cstdint>
#include <memory>
#include <thread>
#include <atomic>
#include <unordered_map>

#include "motor/core/motor_interface.hpp"
#include "motor/core/motor_master.hpp"

namespace motor_manager {

class MotorManager {
public:
    explicit MotorManager(const std::string& config_file);

    virtual ~MotorManager();

    void start();

    void stop();

    void set_zero_offset();

    void set_homing_offset(uint8_t index);

    void write(const motor_interface::motor_command_t* command);

    void read(motor_interface::motor_state_t* status);

    int number_of_motors() { return static_cast<int>(number_of_motors_); }

    bool is_running() const { return running_.load(); }

    // link fault: a master update threw (serial write / timeout) in its last cycle, or a motor missed too many replies
    // in a row. The motor threads keep running and retrying (the robot holds its pose, see ArtiH1::control).
    bool comm_fault() const;

    const motor_interface::motor_homing_t& homing(uint8_t index) const { return homing_[index]; }

    const motor_interface::motor_gains_t& gains(uint8_t index) const { return gains_[index]; }

private:
    void load(const std::string& config_file);

    void run(uint8_t id);

    std::unordered_map<uint8_t, std::unique_ptr<motor_interface::MotorMaster>> masters_;

    std::unordered_map<uint8_t, std::thread> threads_;

    std::mutex mutex_;

    motor_interface::motor_command_t command_[motor_interface::MAX_MOTORS]{};

    motor_interface::motor_state_t status_[motor_interface::MAX_MOTORS]{};

    motor_interface::motor_homing_t homing_[motor_interface::MAX_MOTORS]{};

    motor_interface::motor_gains_t gains_[motor_interface::MAX_MOTORS]{};

    uint8_t master_ids_[motor_interface::MAX_MOTORS]{};

    std::atomic<bool> running_{false};

    std::atomic<bool> comm_error_{false};

    // steady_clock ticks of the last link error (thrown update or a lost motor); comm_fault() stays true for
    // COMM_FAULT_HOLD_TIME after it so a link that drops every few cycles reads as one fault, not a flicker
    mutable std::atomic<std::int64_t> last_comm_error_{0};

    std::atomic<bool> stopped_{false};

    uint8_t number_of_motors_{0};
};

} // namespace motor_manager
#endif
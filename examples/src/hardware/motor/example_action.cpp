#include <algorithm>
#include <array>
#include <thread>
#include <chrono>
#include <csignal>
#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "motor/motor_manager.hpp"
#include "motor/core/motor_interface.hpp"

namespace {
volatile std::sig_atomic_t running = 1;
void on_sigint(int) { running = 0; }

constexpr const char* kMotorNames[] = {
    "left_hip_yaw",
    "right_hip_yaw",
    "left_hip_roll",
    "right_hip_roll",
    "left_hip_pitch",
    "right_hip_pitch",
    "left_knee",
    "right_knee",
    "left_ankle_pitch",
    "right_ankle_pitch",
    "left_ankle_roll",
    "right_ankle_roll",
};

// motor.yaml kp / kd of each motor (driver units), the same motor_command_ gains ARTI-H1 walks with.
void apply_gains(const motor_manager::MotorManager& motor_manager, motor_interface::motor_command_t* command, int n_motors) {
    for (int i = 0; i < n_motors; ++i) {
        command[i].kp = motor_manager.gains(static_cast<uint8_t>(i)).kp;
        command[i].kd = motor_manager.gains(static_cast<uint8_t>(i)).kd;
    }
}
}

int main(int argc, char* argv[])
{
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <config_file> <action_file>" << std::endl;
        return 1;
    }

    std::signal(SIGINT, on_sigint);

    const YAML::Node logs = YAML::LoadFile(argv[2]);
    const YAML::Node actions = logs["joint_targets"];
    if (!actions || !actions.IsSequence()) {
        std::cerr << "Invalid action file: 'actions' sequence is missing." << std::endl;
        return 1;
    }

    std::array<int, motor_interface::MAX_MOTORS> action_to_motor{};
    for (int i = 0; i < motor_interface::MAX_MOTORS; ++i) {
        action_to_motor[i] = i;
    }

    const YAML::Node action_names = logs["action_names"];
    if (action_names && action_names.IsSequence()) {
        std::unordered_map<std::string, int> motor_index;
        for (int i = 0; i < static_cast<int>(sizeof(kMotorNames) / sizeof(kMotorNames[0])); ++i) {
            motor_index.emplace(kMotorNames[i], i);
        }

        const int n_names = static_cast<int>(action_names.size());
        for (int i = 0; i < n_names && i < motor_interface::MAX_MOTORS; ++i) {
            const std::string name = action_names[i].as<std::string>();
            const auto it = motor_index.find(name);
            if (it == motor_index.end()) {
                std::cerr << "Unknown action name: " << name << std::endl;
                return 1;
            }
            action_to_motor[i] = it->second;
        }
    }

    std::vector<std::array<double, motor_interface::MAX_MOTORS>> positions;
    positions.reserve(actions.size());
    for (const auto& step : actions) {
        if (!step.IsSequence()) {
            std::cerr << "Invalid action step." << std::endl;
            return 1;
        }

        std::array<double, motor_interface::MAX_MOTORS> position{};
        const int n = std::min(
            static_cast<int>(step.size()),
            static_cast<int>(motor_interface::MAX_MOTORS));
        for (int i = 0; i < n; ++i) {
            position[static_cast<std::size_t>(action_to_motor[i])] = step[i].as<double>();
        }
        positions.push_back(position);
    }

    const double dt = (logs["dt"] && logs["dt"].as<double>() > 0.0)
        ? logs["dt"].as<double>()
        : (1.0 / 60.0);
    const auto period = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::duration<double>(dt));

    const std::size_t action_steps = positions.size();
    if (!positions.empty()) {
        const auto last = positions.back();
        const int return_steps = std::max(1, static_cast<int>(1.0 / dt));
        for (int s = 1; s <= return_steps; ++s) {
            const double alpha = static_cast<double>(s) / static_cast<double>(return_steps);
            std::array<double, motor_interface::MAX_MOTORS> position{};
            for (std::size_t i = 0; i < position.size(); ++i) {
                position[i] = last[i] * (1.0 - alpha);
            }
            positions.push_back(position);
        }
    }

    std::cout << "Loaded " << action_steps << " steps, then return to 0 over 1 s, dt="
              << dt << " s" << std::endl;

    motor_manager::MotorManager motor_manager(argv[1]);
    motor_manager.start();

    const int n_motors = motor_manager.number_of_motors();
    motor_interface::motor_command_t command[motor_interface::MAX_MOTORS]{};
    for (int i = 0; i < n_motors; ++i) {
        command[i].position = 0.0;
        command[i].kp = 0.0;
        command[i].kd = 0.0;
    }
    motor_manager.write(command);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    motor_manager.set_zero_offset();
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    apply_gains(motor_manager, command, n_motors);

    for (int i = 0; i < n_motors; ++i) {
        command[i].position = 0.0;
    }
    motor_manager.write(command);
    std::this_thread::sleep_for(std::chrono::milliseconds(8000));

    // The log starts at the policy default pose, not at 0: move there over 2 s instead of jumping.
    if (!positions.empty()) {
        const int ramp_steps = std::max(1, static_cast<int>(2.0 / dt));
        for (int s = 1; s <= ramp_steps && running; ++s) {
            const double alpha = static_cast<double>(s) / static_cast<double>(ramp_steps);
            for (int i = 0; i < n_motors; ++i) {
                command[i].position = positions.front()[static_cast<std::size_t>(i)] * alpha;
            }
            motor_manager.write(command);
            std::this_thread::sleep_for(period);
        }
    }

    std::size_t step = 0;
    auto next_wakeup = std::chrono::steady_clock::now();

    while (running) {
        if (!positions.empty()) {
            const std::size_t index = std::min(step, positions.size() - 1);
            for (int i = 0; i < n_motors; ++i) {
                command[i].position = positions[index][static_cast<std::size_t>(i)];
            }
            if (step + 1 < positions.size()) {
                ++step;
            }
        }

        motor_manager.write(command);

        next_wakeup += period;
        const auto now = std::chrono::steady_clock::now();
        if (next_wakeup < now) {
            next_wakeup = now;
        } else {
            std::this_thread::sleep_until(next_wakeup);
        }
    }

    motor_manager.stop();
    return 0;
}

// ARTI-D1 demo trajectory tracking: plays the joint targets of an arti_d1 scripts/replay_demo.py log (e.g.
// config/954.yaml) open loop on the real servos at the log dt.
//
// Dynamixel servos have absolute encoders (motor.yaml zero_offset), so there is no homing / set_zero_offset: the
// servos start limp, ramp from the pose they are in to the first log step, play the log, ramp back to the first step
// and hold it until Ctrl+C (Ctrl+C while playing stops right there). The {side}_arm_m8crank_plate targets are URDF plate angles; their motor IDs are the real
// shoulder m7 servos, so they go through ARTI_D1_shoulder like robot::ArtiD1 does.
//
// Optional <tracking_csv>: per played step the target and the measured joint angle of every joint (URDF, plate
// solved from m8 / m7), plus the raw m7 goal / position.

#include <algorithm>
#include <array>
#include <thread>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <exception>
#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "motor/motor_manager.hpp"
#include "motor/core/motor_interface.hpp"
#include "robot/robots/ARTI_D1_shoulder.hpp"

namespace {
volatile std::sig_atomic_t running = 1;
void on_sigint(int) { running = 0; }

namespace shoulder = robot::arti_d1_shoulder;

constexpr int NUM_JOINTS = 14;
constexpr double RAMP_IN_TIME = 3.0;    // s, current pose -> first log step
constexpr double HOLD_TIME = 2.0;       // s, at the first log step before playing
constexpr double RAMP_OUT_TIME = 1.0;   // s, last log step -> first log step

using Pose = std::array<double, NUM_JOINTS>;

// Motor index (ID - 1) order = Isaac Lab arti_d1 joint order (config/robot/ARTI-D1/robot.yaml).
constexpr const char* kMotorNames[NUM_JOINTS] = {
    "left_arm_m8",
    "left_hip_yaw",
    "right_arm_m8",
    "right_hip_yaw",
    "left_arm_m8crank_plate",
    "left_hip_roll",
    "right_arm_m8crank_plate",
    "right_hip_roll",
    "left_hip_pitch",
    "right_hip_pitch",
    "left_knee",
    "right_knee",
    "left_ankle",
    "right_ankle",
};
constexpr int kM8[2] = {0, 2};   // left / right arm_m8 motor index
constexpr int kM7[2] = {4, 6};   // left / right arm_m8crank_plate motor index = shoulder m7 servo

// URDF joint pose -> motor goals: the plate entries become m7 servo angles (ArtiD1::set_motor_positions).
// m7_prev keeps the loop branch continuous between calls. Throws when the 5-bar cannot close.
Pose to_motor(const Pose& joint, std::array<double, 2>& m7_prev) {
    Pose motor = joint;
    for (int s = 0; s < 2; ++s) {
        const auto side = static_cast<shoulder::Side>(s);
        const double m8 = std::clamp(joint[kM8[s]], shoulder::M8_MIN, shoulder::M8_MAX);
        const double plate = std::clamp(joint[kM7[s]], shoulder::PLATE_MIN, shoulder::PLATE_MAX);
        motor[kM7[s]] = shoulder::m7_from(side, m8, plate, m7_prev[s]);
        m7_prev[s] = motor[kM7[s]];
    }
    return motor;
}

// Motor states -> URDF joint angles (plate solved from m8 / m7, ArtiD1::joint_status).
Pose to_joint(const motor_interface::motor_state_t* status) {
    Pose joint{};
    for (int i = 0; i < NUM_JOINTS; ++i) joint[i] = status[i].position;
    for (int s = 0; s < 2; ++s) {
        joint[kM7[s]] = shoulder::plate_from(static_cast<shoulder::Side>(s), status[kM8[s]].position, status[kM7[s]].position);
    }
    return joint;
}

// motor.yaml kp / kd of each motor (Position P / D Gain register values), the gains ARTI-D1 walks with.
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
        std::cerr << "Usage: " << argv[0] << " <motor_config_file> <action_file> [tracking_csv]" << std::endl;
        return 1;
    }

    std::signal(SIGINT, on_sigint);

    const YAML::Node logs = YAML::LoadFile(argv[2]);
    const YAML::Node actions = logs["joint_targets"];
    if (!actions || !actions.IsSequence() || actions.size() == 0) {
        std::cerr << "Invalid action file: 'joint_targets' sequence is missing." << std::endl;
        return 1;
    }

    std::array<int, NUM_JOINTS> action_to_motor{};
    for (int i = 0; i < NUM_JOINTS; ++i) {
        action_to_motor[i] = i;
    }

    const YAML::Node action_names = logs["joint_target_names"] ? logs["joint_target_names"] : logs["action_names"];
    if (action_names && action_names.IsSequence()) {
        if (static_cast<int>(action_names.size()) != NUM_JOINTS) {
            std::cerr << "The action file has " << action_names.size() << " joints, ARTI-D1 has " << NUM_JOINTS << "." << std::endl;
            return 1;
        }
        std::unordered_map<std::string, int> motor_index;
        for (int i = 0; i < NUM_JOINTS; ++i) {
            motor_index.emplace(kMotorNames[i], i);
        }

        for (int i = 0; i < NUM_JOINTS; ++i) {
            const std::string name = action_names[i].as<std::string>();
            const auto it = motor_index.find(name);
            if (it == motor_index.end()) {
                std::cerr << "Unknown action name: " << name << std::endl;
                return 1;
            }
            action_to_motor[i] = it->second;
        }
    }

    // joint_targets (URDF rad, motor index order)
    std::vector<Pose> positions;
    positions.reserve(actions.size());
    for (const auto& step : actions) {
        if (!step.IsSequence() || static_cast<int>(step.size()) != NUM_JOINTS) {
            std::cerr << "Invalid action step: needs " << NUM_JOINTS << " values." << std::endl;
            return 1;
        }

        Pose position{};
        for (int i = 0; i < NUM_JOINTS; ++i) {
            position[static_cast<std::size_t>(action_to_motor[i])] = step[i].as<double>();
        }
        positions.push_back(position);
    }

    const double dt = (logs["dt"] && logs["dt"].as<double>() > 0.0)
        ? logs["dt"].as<double>()
        : (1.0 / 60.0);
    const auto period = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::duration<double>(dt));

    // Solve every shoulder m7 goal before anything moves, so a 5-bar that cannot close stops here.
    std::vector<Pose> motor_positions;
    motor_positions.reserve(positions.size());
    try {
        std::array<double, 2> m7_prev{};
        for (const auto& position : positions) motor_positions.push_back(to_motor(position, m7_prev));
    } catch (const std::exception& e) {
        std::cerr << "Shoulder m7 for step " << motor_positions.size() << ": " << e.what() << std::endl;
        return 1;
    }

    std::FILE* tracking = nullptr;
    if (argc > 3) {
        tracking = std::fopen(argv[3], "w");
        if (tracking == nullptr) {
            std::cerr << "Cannot open " << argv[3] << std::endl;
            return 1;
        }
        std::fprintf(tracking, "t,step");
        for (const char* name : kMotorNames) std::fprintf(tracking, ",q_des_%s", name);
        for (const char* name : kMotorNames) std::fprintf(tracking, ",pos_%s", name);
        for (const char* name : kMotorNames) std::fprintf(tracking, ",err_%s", name);
        std::fprintf(tracking, ",m7_des_left,m7_pos_left,m7_des_right,m7_pos_right\n");
    }

    std::cout << "Loaded " << positions.size() << " steps (" << positions.size() * dt << " s), dt=" << dt
              << " s; ramp in " << RAMP_IN_TIME << " s, hold " << HOLD_TIME << " s, play, back to the first step over "
              << RAMP_OUT_TIME << " s" << std::endl;

    motor_manager::MotorManager motor_manager(argv[1]);
    if (motor_manager.number_of_motors() != NUM_JOINTS) {
        std::cerr << "motor config has " << motor_manager.number_of_motors() << " motors, ARTI-D1 has " << NUM_JOINTS << "." << std::endl;
        return 1;
    }
    motor_manager.start();

    // Limp (P / D Gain 0) while checking the bus; no homing, the servos report absolute joint angles.
    motor_interface::motor_command_t command[motor_interface::MAX_MOTORS]{};
    motor_manager.write(command);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    if (!motor_manager.is_running()) {
        std::cerr << "Motor communication stopped (see the [MotorManager::run] message above)." << std::endl;
        return 1;
    }

    motor_interface::motor_state_t status[motor_interface::MAX_MOTORS]{};
    motor_manager.read(status);
    const Pose start = to_joint(status);
    std::array<double, 2> m7_prev{status[kM7[0]].position, status[kM7[1]].position};
    for (int i = 0; i < NUM_JOINTS; ++i) {
        std::cout << "  " << kMotorNames[i] << " at " << start[i] << " rad -> first step " << positions.front()[i] << " rad" << std::endl;
    }

    apply_gains(motor_manager, command, NUM_JOINTS);

    auto next_wakeup = std::chrono::steady_clock::now();
    const auto send = [&](const Pose& motor) {
        for (int i = 0; i < NUM_JOINTS; ++i) command[i].position = motor[i];
        motor_manager.write(command);
        next_wakeup += period;
        const auto now = std::chrono::steady_clock::now();
        if (next_wakeup < now) {
            next_wakeup = now;
        } else {
            std::this_thread::sleep_until(next_wakeup);
        }
    };

    // Interpolate in URDF joint space, so the shoulder 5-bar stays inside the URDF limits on the way.
    const auto ramp = [&](const Pose& from, const Pose& to, double duration) {
        const int steps = std::max(1, static_cast<int>(duration / dt));
        for (int s = 1; s <= steps && running && motor_manager.is_running(); ++s) {
            const double alpha = static_cast<double>(s) / static_cast<double>(steps);
            Pose pose{};
            for (int i = 0; i < NUM_JOINTS; ++i) pose[i] = from[i] + (to[i] - from[i]) * alpha;
            send(to_motor(pose, m7_prev));
        }
    };

    ramp(start, positions.front(), RAMP_IN_TIME);
    for (int s = 0; s < static_cast<int>(HOLD_TIME / dt) && running; ++s) send(motor_positions.front());

    std::cout << "Playing" << std::endl;
    const auto play_start = std::chrono::steady_clock::now();
    for (std::size_t step = 0; step < positions.size() && running && motor_manager.is_running(); ++step) {
        if (tracking != nullptr) {
            // state measured one step after the previous goal was sent, against that goal
            motor_manager.read(status);
            const Pose joint = to_joint(status);
            const Pose& des = positions[step == 0 ? 0 : step - 1];
            const Pose& motor_des = motor_positions[step == 0 ? 0 : step - 1];
            const double t = std::chrono::duration<double>(std::chrono::steady_clock::now() - play_start).count();
            std::fprintf(tracking, "%.4f,%zu", t, step);
            for (const double v : des) std::fprintf(tracking, ",%.6f", v);
            for (const double v : joint) std::fprintf(tracking, ",%.6f", v);
            for (int i = 0; i < NUM_JOINTS; ++i) std::fprintf(tracking, ",%d", status[i].error);
            for (int s = 0; s < 2; ++s) std::fprintf(tracking, ",%.6f,%.6f", motor_des[kM7[s]], status[kM7[s]].position);
            std::fprintf(tracking, "\n");
        }
        m7_prev = {motor_positions[step][kM7[0]], motor_positions[step][kM7[1]]};
        send(motor_positions[step]);
    }

    if (motor_manager.is_running()) {
        ramp(positions.back(), positions.front(), RAMP_OUT_TIME);
        std::cout << "Done, holding the first step (Ctrl+C to stop)" << std::endl;
        while (running && motor_manager.is_running()) send(motor_positions.front());
    } else {
        std::cerr << "Motor communication stopped (see the [MotorManager::run] message above)." << std::endl;
    }

    if (tracking != nullptr) std::fclose(tracking);
    motor_manager.stop();
    return 0;
}

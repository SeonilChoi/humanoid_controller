// ARTI-D1 joint check / system identification (port of the ARTI-H1 example_sysid): per-joint sign / range / zero
// check, step response, chirp and velocity sweeps, plus an open-loop replay of a replay_demo log (config/681.yaml).
// Every test logs all joints to a CSV (target, position, velocity, current, temperature per joint).
// Order (config/sysid/ARTI-D1.yaml tests): each joint around hold_pose -> home (real servo zero) -> demo replay.
//
//   ./examples/example_sysid                      # = ../config/robot/ARTI-D1/motor.yaml ../config/sysid/ARTI-D1.yaml ./sysid_logs
//   ./examples/example_sysid <motor_config.yaml> <sysid_config.yaml> <output_dir>
//
// Hang the robot (feet off the ground) before running. See config/sysid/ARTI-D1.yaml.
//
// Differences to ARTI-H1: Dynamixel servos keep their zero (absolute encoders, motor.yaml zero_offset), so there is no
// homing run; the servos start limp and move from where they are to hold_pose. 'home' moves to home_pose, the real
// servo zero (every servo at pulse 0). Commands are URDF joint angles; the
// {side}_arm_m8crank_plate entries go through ARTI_D1_shoulder to the real shoulder m7 servos (as robot::ArtiD1 and
// example_action do). Abort = every servo limp (Position P Gain 0). 'torque' of a Dynamixel is its present current (A).

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <filesystem>
#include <functional>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "motor/motor_manager.hpp"
#include "motor/core/motor_interface.hpp"
#include "robot/robots/ARTI_D1_shoulder.hpp"

namespace {

volatile std::sig_atomic_t running = 1;
void on_sigint(int) { running = 0; }

namespace shoulder = robot::arti_d1_shoulder;

constexpr int N = 14;
constexpr double PI = 3.14159265358979323846;
constexpr double HW_MARGIN = 0.05;  // rad a measured joint may pass its URDF limit before the test aborts

// Motor index (ID - 1) order = Isaac Lab arti_d1 joint order (robot::ArtiD1, example_action, robot.yaml).
constexpr const char* kJointNames[N] = {
    "left_arm_m8",          "left_hip_yaw",    "right_arm_m8",           "right_hip_yaw",
    "left_arm_m8crank_plate", "left_hip_roll", "right_arm_m8crank_plate", "right_hip_roll",
    "left_hip_pitch",       "right_hip_pitch", "left_knee",              "right_knee",
    "left_ankle",           "right_ankle",
};
constexpr int kM8[2] = {0, 2};  // left / right arm_m8 motor index
constexpr int kM7[2] = {4, 6};  // left / right arm_m8crank_plate motor index = shoulder m7 servo

using Clock = std::chrono::steady_clock;
using Pose = std::array<double, N>;

struct JointLimits {
    Pose min{};  // URDF joint limits (plates: shoulder PLATE_MIN / MAX, motor.yaml holds the m7 ones)
    Pose max{};
    Pose axis{};         // motor.yaml axis / zero_offset, only to print the servo pulse when a test aborts
    Pose zero_offset{};
};

struct Config {
    int loop_period_us{10000};
    Pose hold_pose{};
    Pose home_pose{};
    double move_speed{0.2};
    Pose lo{};  // safe box = config safe_min/max intersected with the URDF joint limits
    Pose hi{};
    double abort_tracking_error{0.3};
    double abort_current{3.0};
    double abort_temperature{70.0};
    YAML::Node expect_positive;
    YAML::Node tests;
};

int joint_index(const std::string& name) {
    for (int i = 0; i < N; ++i) {
        if (name == kJointNames[i]) return i;
    }
    throw std::runtime_error("Unknown joint name: " + name);
}

Pose read_pose(const YAML::Node& node, const char* key) {
    const auto v = node[key].as<std::vector<double>>();
    if (v.size() != static_cast<std::size_t>(N)) throw std::runtime_error(std::string("'") + key + "' needs 14 values.");
    Pose p{};
    std::copy(v.begin(), v.end(), p.begin());
    return p;
}

JointLimits load_limits(const std::string& motor_config) {
    JointLimits limits;
    std::array<bool, N> seen{};
    const YAML::Node root = YAML::LoadFile(motor_config);
    for (const auto& m : root["masters"]) {
        if (m["type"].as<std::string>() != "dynamixel") throw std::runtime_error("ARTI-D1 sysid expects Dynamixel masters only.");
        for (const auto& motor : m["motors"]) {
            const int index = motor["id"].as<int>() - 1;
            if (index < 0 || index >= N) throw std::runtime_error("Motor id out of range in " + motor_config);
            limits.min[index] = motor["min"].as<double>();
            limits.max[index] = motor["max"].as<double>();
            limits.axis[index] = motor["axis"].as<double>();
            limits.zero_offset[index] = motor["zero_offset"].as<double>(0.0);
            seen[index] = true;
        }
    }
    for (int i = 0; i < N; ++i) {
        if (!seen[i]) throw std::runtime_error(std::string("No motor for joint ") + kJointNames[i]);
    }
    for (const int i : kM7) {
        limits.min[i] = shoulder::PLATE_MIN;
        limits.max[i] = shoulder::PLATE_MAX;
    }
    return limits;
}

Config load_config(const std::string& sysid_config, const JointLimits& limits) {
    const YAML::Node root = YAML::LoadFile(sysid_config);
    Config c;
    c.loop_period_us = root["loop_period_us"].as<int>(10000);
    c.hold_pose = read_pose(root, "hold_pose");
    c.home_pose = read_pose(root, "home_pose");
    c.move_speed = root["move_speed"].as<double>(0.2);
    const Pose safe_min = read_pose(root, "safe_min");
    const Pose safe_max = read_pose(root, "safe_max");
    for (int i = 0; i < N; ++i) {
        // commands stay inside the URDF limits (the training limits; the demo and the default pose touch them)
        c.lo[i] = std::max(safe_min[i], limits.min[i]);
        c.hi[i] = std::min(safe_max[i], limits.max[i]);
        if (c.lo[i] >= c.hi[i]) throw std::runtime_error(std::string("Empty safe range for ") + kJointNames[i]);
        if (c.hold_pose[i] < c.lo[i] || c.hold_pose[i] > c.hi[i]) {
            throw std::runtime_error(std::string("hold_pose outside the safe range for ") + kJointNames[i]);
        }
        if (c.home_pose[i] < c.lo[i] || c.home_pose[i] > c.hi[i]) {
            throw std::runtime_error(std::string("home_pose outside the safe range for ") + kJointNames[i]);
        }
    }
    const YAML::Node abort = root["abort"];
    c.abort_tracking_error = abort["tracking_error"].as<double>(0.3);
    c.abort_current = abort["current"].as<double>(3.0);
    c.abort_temperature = abort["temperature"].as<double>(70.0);
    c.expect_positive = root["expect_positive"];
    c.tests = root["tests"];
    return c;
}

class Rig {
public:
    Rig(motor_manager::MotorManager& manager, const JointLimits& limits, const Config& config)
    : manager_(manager), limits_(limits), config_(config) {
        // Test gains = motor.yaml kp / kd (Position P / D Gain register values), the gains ARTI-D1 walks with.
        for (int i = 0; i < N; ++i) {
            kp_[i] = manager.gains(static_cast<uint8_t>(i)).kp;
            kd_[i] = manager.gains(static_cast<uint8_t>(i)).kd;
        }
    }

    const Pose& kp() const { return kp_; }
    const Pose& kd() const { return kd_; }

    // Measured URDF joint angles (plates solved from m8 / m7); also re-seeds the m7 loop branch.
    Pose read_joints(motor_interface::motor_state_t* status) {
        manager_.read(status);
        Pose joint{};
        for (int i = 0; i < N; ++i) joint[i] = status[i].position;
        for (int s = 0; s < 2; ++s) {
            joint[kM7[s]] = shoulder::plate_from(static_cast<shoulder::Side>(s), status[kM8[s]].position, status[kM7[s]].position);
        }
        return joint;
    }

    void seed_m7(const motor_interface::motor_state_t* status) {
        for (int s = 0; s < 2; ++s) m7_prev_[s] = status[kM7[s]].position;
    }

    // URDF joint pose -> motor goals (plates -> m7) with the walking gains.
    void write_pose(const Pose& target) {
        motor_interface::motor_command_t cmd[motor_interface::MAX_MOTORS]{};
        for (int i = 0; i < N; ++i) {
            cmd[i].position = std::clamp(target[i], config_.lo[i], config_.hi[i]);
            cmd[i].kp = kp_[i];
            cmd[i].kd = kd_[i];
        }
        for (int s = 0; s < 2; ++s) {
            const double m8 = std::clamp(cmd[kM8[s]].position, shoulder::M8_MIN, shoulder::M8_MAX);
            cmd[kM7[s]].position = shoulder::m7_from(static_cast<shoulder::Side>(s), m8, cmd[kM7[s]].position, m7_prev_[s]);
            m7_prev_[s] = cmd[kM7[s]].position;
        }
        manager_.write(cmd);
    }

    // Every servo limp (Position P / D Gain 0).
    void limp() {
        motor_interface::motor_command_t cmd[motor_interface::MAX_MOTORS]{};
        manager_.write(cmd);
    }

    // Empty string = safe.
    std::string check(const Pose& target, const Pose& joint, const motor_interface::motor_state_t* status) const {
        if (!manager_.is_running()) return "motor communication stopped";
        std::ostringstream why;
        for (int i = 0; i < N; ++i) {
            const auto& s = status[i];
            if (s.error != 0) {
                // first, so an overload / hardware alert is named even when the joint then moves out of range
                why << kJointNames[i] << " servo status error 0x" << std::hex << static_cast<int>(s.error) << std::dec
                    << " (0x80 = hardware alert: overload / overheat / voltage; check Hardware Error Status in the Wizard)";
            } else if (joint[i] < limits_.min[i] - HW_MARGIN || joint[i] > limits_.max[i] + HW_MARGIN) {
                why << kJointNames[i] << " position " << joint[i] << " (servo angle " << s.position << " rad = pulse ~"
                    << std::lround((s.position * limits_.axis[i] + limits_.zero_offset[i]) / (2.0 * PI) * 4096.0)
                    << " modulo 4096) outside the joint limits [" << limits_.min[i]
                    << ", " << limits_.max[i] << "] (axis / zero_offset wrong?)";
            } else if (std::abs(std::clamp(target[i], config_.lo[i], config_.hi[i]) - joint[i]) > config_.abort_tracking_error) {
                why << kJointNames[i] << " tracking error " << target[i] - joint[i] << " rad";
            } else if (std::abs(s.torque) > config_.abort_current) {
                why << kJointNames[i] << " current " << s.torque << " A";
            } else if (s.temperature > config_.abort_temperature) {
                why << kJointNames[i] << " temperature " << s.temperature << " C";
            }
            if (why.tellp() > 0) return why.str();
        }
        return "";
    }

    // Run 'profile(t) -> pose' for 'duration' s at loop_period_us, logging every tick to 'csv_path' (if not empty).
    // Returns false when aborted (every servo is then limp).
    bool run(double duration, const std::function<Pose(double)>& profile, const std::string& csv_path,
             const std::string& header_comment) {
        std::FILE* csv = nullptr;
        if (!csv_path.empty()) {
            csv = std::fopen(csv_path.c_str(), "w");
            if (csv == nullptr) throw std::runtime_error("Cannot open " + csv_path);
            std::fprintf(csv, "# %s\n# kp_register", header_comment.c_str());
            for (int i = 0; i < N; ++i) std::fprintf(csv, " %g", kp_[i]);
            std::fprintf(csv, "\n# kd_register");
            for (int i = 0; i < N; ++i) std::fprintf(csv, " %g", kd_[i]);
            std::fprintf(csv, "\n# pos = URDF joint angle (plates solved from m8 / m7), tau = present current (A)\nt");
            for (int i = 0; i < N; ++i) {
                std::fprintf(csv, ",%s_cmd,%s_pos,%s_vel,%s_tau,%s_temp",
                             kJointNames[i], kJointNames[i], kJointNames[i], kJointNames[i], kJointNames[i]);
            }
            std::fprintf(csv, "\n");
        }

        const auto period = std::chrono::microseconds(config_.loop_period_us);
        const auto start = Clock::now();
        auto next = start;
        bool ok = true;
        motor_interface::motor_state_t status[motor_interface::MAX_MOTORS]{};
        while (true) {
            const double t = std::chrono::duration<double>(Clock::now() - start).count();
            if (t > duration) break;
            if (!running) {
                std::cout << "[sysid] Ctrl+C: stopping the test." << std::endl;
                ok = false;
                break;
            }
            Pose target = profile(t);
            for (int i = 0; i < N; ++i) target[i] = std::clamp(target[i], config_.lo[i], config_.hi[i]);
            std::string why;
            try {
                write_pose(target);
            } catch (const std::exception& e) {
                why = std::string("shoulder 5-bar: ") + e.what();
            }
            const Pose joint = read_joints(status);
            if (why.empty()) why = check(target, joint, status);
            if (csv != nullptr) {
                std::fprintf(csv, "%.6f", t);
                for (int i = 0; i < N; ++i) {
                    std::fprintf(csv, ",%.6f,%.6f,%.5f,%.4f,%.0f", target[i], joint[i], status[i].velocity,
                                 status[i].torque, status[i].temperature);
                }
                std::fprintf(csv, "\n");
            }
            if (!why.empty()) {
                limp();
                aborted_ = true;
                std::cerr << "[sysid] ABORT: " << why << " -> every servo limp." << std::endl;
                ok = false;
                break;
            }
            next += period;
            const auto now = Clock::now();
            if (next < now) {
                next = now;
            } else {
                std::this_thread::sleep_until(next);
            }
        }
        if (csv != nullptr) std::fclose(csv);
        return ok;
    }

    bool aborted() const { return aborted_; }

    // Straight-line move of every joint to 'goal' at move_speed (no log), then 0.5 s hold.
    bool move_to(const Pose& from, const Pose& goal, const std::string& csv_path = "") {
        double distance = 0.0;
        for (int i = 0; i < N; ++i) distance = std::max(distance, std::abs(goal[i] - from[i]));
        const double duration = std::max(0.5, distance / config_.move_speed);
        return run(duration, [&](double t) {
            const double a = std::min(1.0, t / duration);
            Pose p{};
            for (int i = 0; i < N; ++i) p[i] = from[i] + (goal[i] - from[i]) * a;
            return p;
        }, csv_path, "type=move") && run(0.5, [&](double) { return goal; }, "", "");
    }

private:
    motor_manager::MotorManager& manager_;
    const JointLimits& limits_;
    const Config& config_;
    Pose kp_{};
    Pose kd_{};
    std::array<double, 2> m7_prev_{};
    bool aborted_{false};  // a safety check tripped: every servo is limp and nothing may move any more
};

bool wait_for_enter(const std::string& message) {
    std::cout << message << "  [Enter = run, s + Enter = skip, q + Enter = quit] " << std::flush;
    std::string line;
    if (!std::getline(std::cin, line)) return false;
    if (line == "q") {
        running = 0;
        return false;
    }
    return line != "s";
}

// Log-frequency chirp with 1 s cosine fade in / out.
double chirp(double t, double amplitude, double f0, double f1, double duration) {
    const double k = f1 / f0;
    const double phase = 2.0 * PI * f0 * duration / std::log(k) * (std::pow(k, t / duration) - 1.0);
    const double fade = std::min({1.0, t, duration - t});
    const double window = fade <= 0.0 ? 0.0 : 0.5 - 0.5 * std::cos(PI * std::min(1.0, fade));
    return amplitude * window * std::sin(phase);
}

// Piecewise-linear profile through (time, value) points.
double piecewise(double t, const std::vector<std::pair<double, double>>& points) {
    if (t <= points.front().first) return points.front().second;
    for (std::size_t i = 1; i < points.size(); ++i) {
        if (t <= points[i].first) {
            const double a = (t - points[i - 1].first) / (points[i].first - points[i - 1].first);
            return points[i - 1].second + a * (points[i].second - points[i - 1].second);
        }
    }
    return points.back().second;
}

}  // namespace

int main(int argc, char* argv[]) {
    bool passive = false;
    std::vector<std::string> args;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--passive") {
            passive = true;
        } else {
            args.push_back(a);
        }
    }
    const std::string motor_config = args.size() > 0 ? args[0] : "../config/robot/ARTI-D1/motor.yaml";
    const std::string sysid_config = args.size() > 1 ? args[1] : "../config/sysid/ARTI-D1.yaml";
    const std::filesystem::path out_dir(args.size() > 2 ? args[2] : "./sysid_logs");
    if (motor_config == "-h" || motor_config == "--help") {
        std::cerr << "Usage: " << argv[0] << " [--passive] [<motor_config.yaml> <sysid_config.yaml> <output_dir>]" << std::endl;
        std::cerr << "  default: ../config/robot/ARTI-D1/motor.yaml ../config/sysid/ARTI-D1.yaml ./sysid_logs" << std::endl;
        std::cerr << "  --passive: every servo limp, print the joint angles; move each joint by hand to check its sign" << std::endl;
        return 1;
    }
    std::signal(SIGINT, on_sigint);

    const JointLimits limits = load_limits(motor_config);
    const Config config = load_config(sysid_config, limits);
    std::filesystem::create_directories(out_dir);
    std::cout << "[sysid] motor config " << motor_config << ", tests " << sysid_config << ", logs " << out_dir << std::endl;

    std::cout << "[sysid] Safe range per joint (rad, URDF joint angle):" << std::endl;
    for (int i = 0; i < N; ++i) {
        std::cout << "  ID " << i + 1 << " " << kJointNames[i] << "  [" << config.lo[i] << ", " << config.hi[i]
                  << "]  hold " << config.hold_pose[i] << "  home " << config.home_pose[i] << std::endl;
    }
    if (!wait_for_enter("[sysid] Robot hanging, feet free, nothing near the legs or arms?")) return 0;

    motor_manager::MotorManager manager(motor_config);
    if (manager.number_of_motors() != N) {
        std::cerr << "[sysid] motor config has " << manager.number_of_motors() << " motors, ARTI-D1 has " << N << "." << std::endl;
        return 1;
    }
    Rig rig(manager, limits, config);
    std::cout << "[sysid] Gains from motor.yaml (Position P / D Gain registers; the sim models P 800, D 0):" << std::endl;
    for (int i = 0; i < N; ++i) {
        std::cout << "  " << kJointNames[i] << "  P " << rig.kp()[i] << "  D " << rig.kd()[i] << std::endl;
    }
    if (!wait_for_enter("[sysid] Gains OK?")) return 0;
    manager.start();
    int exit_code = 0;

    try {
        // Limp while the bus comes up; no homing, the servos report absolute joint angles.
        rig.limp();
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        if (!manager.is_running()) throw std::runtime_error("Motor communication stopped (see the [MotorManager::run] message above).");

        motor_interface::motor_state_t status[motor_interface::MAX_MOTORS]{};
        const Pose start = rig.read_joints(status);
        rig.seed_m7(status);
        std::cout << "[sysid] Measured pose (URDF rad) -> hold_pose:" << std::endl;
        bool outside = false;
        for (int i = 0; i < N; ++i) {
            const bool bad = start[i] < limits.min[i] - HW_MARGIN || start[i] > limits.max[i] + HW_MARGIN;
            outside = outside || bad;
            std::cout << "  " << kJointNames[i] << "  " << start[i] << " -> " << config.hold_pose[i]
                      << (bad ? "   <-- OUTSIDE the joint limits: check axis / zero_offset" : "") << std::endl;
        }
        if (outside && !passive) throw std::runtime_error("Measured pose outside the joint limits; fix motor.yaml before moving.");
        if (!passive && !wait_for_enter("[sysid] Do the measured angles match the pose the robot is in? Move to hold_pose")) {
            throw std::runtime_error("Stopped before moving.");
        }
        if (passive) {
            // No torque at all: move each joint by hand in its + direction (expect_positive) and watch it count up.
            std::cout << "[sysid] PASSIVE: every servo limp. Move one joint by hand in the direction below; its value "
                         "must INCREASE. Ctrl+C to stop." << std::endl;
            for (int i = 0; i < N; ++i) {
                const std::string name = kJointNames[i];
                std::cout << "  ID " << i + 1 << " " << name << " +: "
                          << (config.expect_positive[name] ? config.expect_positive[name].as<std::string>() : "?") << std::endl;
            }
            std::FILE* csv = std::fopen((out_dir / "00_passive.csv").string().c_str(), "w");
            if (csv != nullptr) {
                std::fprintf(csv, "t");
                for (int i = 0; i < N; ++i) std::fprintf(csv, ",%s_pos", kJointNames[i]);
                std::fprintf(csv, "\n");
            }
            const auto t0 = Clock::now();
            int tick = 0;
            while (running && manager.is_running()) {
                rig.limp();
                const Pose q = rig.read_joints(status);
                const double t = std::chrono::duration<double>(Clock::now() - t0).count();
                if (csv != nullptr) {
                    std::fprintf(csv, "%.3f", t);
                    for (int i = 0; i < N; ++i) std::fprintf(csv, ",%.5f", q[i]);
                    std::fprintf(csv, "\n");
                }
                if (tick++ % 5 == 0) {
                    std::ostringstream line;
                    line.setf(std::ios::fixed);
                    line.precision(1);
                    for (int i = 0; i < N; ++i) line << " " << i + 1 << ":" << q[i] * 180.0 / PI;
                    std::cout << "[deg]" << line.str() << std::endl;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            if (csv != nullptr) std::fclose(csv);
            rig.limp();
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            manager.stop();
            return 0;
        }
        if (!rig.move_to(start, config.hold_pose, (out_dir / "00_move_to_hold.csv").string())) {
            throw std::runtime_error("Aborted while moving to hold_pose (log " + (out_dir / "00_move_to_hold.csv").string() + ").");
        }
        Pose here = config.hold_pose;  // pose the robot holds between tests

        int test_number = 0;
        for (const auto& test : config.tests) {
            if (!running) break;
            const std::string type = test["type"].as<std::string>();
            ++test_number;
            char prefix[8];
            std::snprintf(prefix, sizeof(prefix), "%02d", test_number);

            if (type == "home") {
                const double hold = test["hold"].as<double>(2.0);
                if (!wait_for_enter("[sysid] " + std::string(prefix) + " move to home (real servo zero: hip link level, legs "
                                    "straight and vertical, soles flat; every servo at pulse 0).")) continue;
                if (!rig.move_to(here, config.home_pose)) break;
                here = config.home_pose;
                motor_interface::motor_state_t s[motor_interface::MAX_MOTORS]{};
                const Pose at = rig.read_joints(s);
                std::cout << "[sysid] At home: measured - home_pose per joint (rad, ~0 expected):" << std::endl;
                for (int i = 0; i < N; ++i) {
                    std::cout << "  ID " << i + 1 << " " << kJointNames[i] << "  error " << at[i] - config.home_pose[i] << std::endl;
                }
                if (!rig.run(hold, [&](double) { return config.home_pose; }, "", "")) break;
                continue;
            }

            if (type == "replay") {
                const std::string file = test["file"].as<std::string>();
                const YAML::Node log = YAML::LoadFile(file);
                const double dt = log["dt"].as<double>(1.0 / 60.0);
                const YAML::Node names = log["joint_target_names"];
                std::array<int, N> column_to_joint{};
                for (int i = 0; i < N; ++i) column_to_joint[i] = names ? joint_index(names[i].as<std::string>()) : i;
                std::vector<Pose> frames;
                for (const auto& step : log["joint_targets"]) {
                    Pose p{};
                    for (int i = 0; i < N; ++i) p[column_to_joint[i]] = step[i].as<double>();
                    frames.push_back(p);
                }
                if (frames.empty()) throw std::runtime_error("No joint_targets in " + file);
                if (!wait_for_enter("[sysid] " + std::string(prefix) + " replay " + file + " on ALL joints (" +
                                    std::to_string(frames.size() * dt) + " s).")) continue;
                if (!rig.move_to(here, frames.front())) break;
                here = frames.front();
                if (!rig.run(test["hold"].as<double>(2.0), [&](double) { return frames.front(); }, "", "")) break;
                const double duration = frames.size() * dt;
                const std::string csv = (out_dir / (std::string(prefix) + "_replay.csv")).string();
                const bool ok = rig.run(duration, [&](double t) {
                    return frames[std::min(frames.size() - 1, static_cast<std::size_t>(t / dt))];
                }, csv, "type=replay file=" + file + " dt=" + std::to_string(dt));
                std::cout << "[sysid] " << (ok ? "saved " : "ABORTED, partial log ") << csv << std::endl;
                if (!ok || !rig.move_to(frames.back(), frames.front())) break;
                continue;
            }

            const std::string joint = test["joint"].as<std::string>();
            const int j = joint_index(joint);
            if (here != config.hold_pose) {
                if (!rig.move_to(here, config.hold_pose)) break;
                here = config.hold_pose;
            }
            const double c = config.hold_pose[j];
            const double amplitude = test["amplitude"].as<double>();
            if (c + amplitude > config.hi[j] || c - amplitude < config.lo[j]) {
                std::cout << "[sysid] WARNING: " << joint << " +-" << amplitude << " around " << c
                          << " leaves the safe range and will be clipped." << std::endl;
            }

            double duration = 0.0;
            std::function<double(double)> offset;
            std::ostringstream header;
            header << "type=" << type << " joint=" << joint << " id=" << j + 1 << " center=" << c
                   << " amplitude=" << amplitude;

            if (type == "check") {
                const double speed = test["speed"].as<double>(0.1);
                const double leg = amplitude / speed;
                const std::vector<std::pair<double, double>> pts = {
                    {0.0, 0.0}, {0.5, 0.0}, {0.5 + leg, amplitude}, {1.5 + leg, amplitude},
                    {1.5 + 3 * leg, -amplitude}, {2.5 + 3 * leg, -amplitude}, {2.5 + 4 * leg, 0.0}, {3.0 + 4 * leg, 0.0}};
                duration = pts.back().first;
                offset = [pts](double t) { return piecewise(t, pts); };
                header << " speed=" << speed;
                const std::string expect = config.expect_positive[joint] ? config.expect_positive[joint].as<std::string>() : "?";
                std::cout << "[sysid] ID " << j + 1 << " " << joint << ": first moves +" << amplitude
                          << " rad. Expected: " << expect << ". Then -" << amplitude << " (the opposite)." << std::endl;
            } else if (type == "step") {
                const double hold = test["hold"].as<double>(1.0);
                const int repeat = test["repeat"].as<int>(3);
                duration = (4 * repeat + 1) * hold;
                offset = [=](double t) {
                    const int k = static_cast<int>(t / hold);
                    if (k == 0 || k > 4 * repeat) return 0.0;
                    switch ((k - 1) % 4) {
                        case 0: return amplitude;
                        case 2: return -amplitude;
                        default: return 0.0;
                    }
                };
                header << " hold=" << hold << " repeat=" << repeat;
            } else if (type == "chirp") {
                const double f0 = test["f0"].as<double>(0.3);
                const double f1 = test["f1"].as<double>(5.0);
                duration = test["duration"].as<double>(20.0);
                offset = [=](double t) { return chirp(t, amplitude, f0, f1, duration); };
                header << " f0=" << f0 << " f1=" << f1 << " duration=" << duration;
            } else if (type == "sweep") {
                const auto speeds = test["speeds"].as<std::vector<double>>();
                const int cycles = test["cycles"].as<int>(3);
                std::vector<std::pair<double, double>> pts = {{0.0, 0.0}, {0.5, 0.0}};
                header << " cycles=" << cycles << " speeds=";
                for (const double v : speeds) {
                    header << v << ";";
                    double t = pts.back().first;
                    const double leg = amplitude / v;
                    pts.push_back({t += leg, amplitude});
                    for (int k = 0; k < cycles; ++k) {
                        pts.push_back({t += 2 * leg, -amplitude});
                        pts.push_back({t += 2 * leg, amplitude});
                    }
                    pts.push_back({t += leg, 0.0});
                    pts.push_back({t += 0.5, 0.0});
                }
                duration = pts.back().first;
                offset = [pts](double t) { return piecewise(t, pts); };
            } else {
                std::cerr << "[sysid] Unknown test type '" << type << "', skipped." << std::endl;
                continue;
            }

            if (!wait_for_enter("[sysid] " + std::string(prefix) + " " + type + " ID " + std::to_string(j + 1) + " " +
                                joint + " (" + std::to_string(duration) + " s).")) continue;
            const std::string csv = (out_dir / (std::string(prefix) + "_" + type + "_" + joint + ".csv")).string();
            const bool ok = rig.run(duration, [&](double t) {
                Pose p = config.hold_pose;
                p[j] = c + offset(t);
                return p;
            }, csv, header.str());
            std::cout << "[sysid] " << (ok ? "saved " : "ABORTED, partial log ") << csv << std::endl;
            if (!ok) break;
            if (type == "check" &&
                !wait_for_enter("[sysid] Did " + joint + " move as expected first? If NOT, quit (q) and flip its motor.yaml axis.")) {
                break;
            }
        }

        if (rig.aborted()) {
            std::cout << "[sysid] Aborted: every servo stays limp (no move back to home)." << std::endl;
        } else if (running && manager.is_running()) {
            std::cout << "[sysid] Tests done: back to home, hold 2 s, then every servo limp." << std::endl;
            if (rig.move_to(here, config.home_pose)) rig.run(2.0, [&](double) { return config.home_pose; }, "", "");
        }
    } catch (const std::exception& e) {
        std::cerr << "[sysid] " << e.what() << std::endl;
        exit_code = 1;
    }

    rig.limp();
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    manager.stop();
    return exit_code;
}

// System identification for ARTI-H1 joints: sign / range check, step response, chirp and velocity sweeps,
// plus an open-loop replay of a replay_demo log. Every test logs all joints at loop_period_us to a CSV that
// arti_h1/scripts/tuning/sysid_analyze.py turns into latency, PD response, friction and rotor inertia.
//
//   ./example_sysid <motor_config.yaml> <sysid_config.yaml> <output_dir>
//
// Hang the robot (feet off the ground) before running. See config/sysid/ARTI-H1.yaml.

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

namespace {

volatile std::sig_atomic_t running = 1;
void on_sigint(int) { running = 0; }

constexpr int N = 12;  // ARTI-H1 joints
constexpr double PI = 3.14159265358979323846;
constexpr double HW_MARGIN = 0.05;  // rad kept inside motor.yaml min/max

// Isaac joint order = motor id - 1 (same as robot::ArtiH1 and example_action).
constexpr const char* kJointNames[N] = {
    "left_hip_yaw",     "right_hip_yaw",     "left_hip_roll",   "right_hip_roll",
    "left_hip_pitch",   "right_hip_pitch",   "left_knee",       "right_knee",
    "left_ankle_pitch", "right_ankle_pitch", "left_ankle_roll", "right_ankle_roll",
};

// Homing, copied from robot::ArtiH1::home so this example does not need the IMU / joystick / ONNX of the robot.
constexpr auto HOMING_PERIOD = std::chrono::milliseconds(10);
constexpr auto HOMING_SETTLE_TIME = std::chrono::milliseconds(500);
constexpr auto HOMING_TIMEOUT = std::chrono::seconds(15);
constexpr int HOMING_STALL_COUNT = 10;
constexpr double HOMING_MAX_LEAD = 0.15;
constexpr double HOMING_VELOCITY_FILTER = 0.2;
constexpr double HOMING_ZERO_TOLERANCE = 0.03;
constexpr auto HOMING_ZERO_TIMEOUT = std::chrono::seconds(3);

using Clock = std::chrono::steady_clock;
using Pose = std::array<double, N>;

struct MotorInfo {
    bool unitree{false};  // Unitree: rotor-side gains; CubeMars MIT mode: output-side gains
    double gear_ratio{1.0};
    double min{0.0};
    double max{0.0};
};

struct Config {
    std::string zero_mode{"homing"};
    int loop_period_us{1000};
    Pose hold_pose{};
    double move_speed{0.2};
    Pose lo{};  // safe box = config safe_min/max intersected with motor.yaml min/max - HW_MARGIN
    Pose hi{};
    double abort_tracking_error{0.3};
    double abort_torque{18.0};
    double abort_temperature{70.0};
    double abort_damping_kd{1.0};
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
    if (v.size() != static_cast<std::size_t>(N)) throw std::runtime_error(std::string("'") + key + "' needs 12 values.");
    Pose p{};
    std::copy(v.begin(), v.end(), p.begin());
    return p;
}

std::array<MotorInfo, N> load_motor_info(const std::string& motor_config) {
    std::array<MotorInfo, N> info{};
    std::array<bool, N> seen{};
    const YAML::Node root = YAML::LoadFile(motor_config);
    for (const auto& m : root["masters"]) {
        const std::string type = m["type"].as<std::string>();
        for (const auto& motor : m["motors"]) {
            const int index = motor["id"].as<int>() - 1;
            if (index < 0 || index >= N) throw std::runtime_error("Motor id out of range in " + motor_config);
            info[index].unitree = (type == "unitree");
            info[index].gear_ratio = motor["gear_ratio"].as<double>();
            info[index].min = motor["min"].as<double>();
            info[index].max = motor["max"].as<double>();
            seen[index] = true;
        }
    }
    for (int i = 0; i < N; ++i) {
        if (!seen[i]) throw std::runtime_error(std::string("No motor for joint ") + kJointNames[i]);
    }
    return info;
}

Config load_config(const std::string& sysid_config, const std::array<MotorInfo, N>& info) {
    const YAML::Node root = YAML::LoadFile(sysid_config);
    Config c;
    c.zero_mode = root["zero_mode"].as<std::string>("homing");
    c.loop_period_us = root["loop_period_us"].as<int>(1000);
    c.hold_pose = read_pose(root, "hold_pose");
    c.move_speed = root["move_speed"].as<double>(0.2);
    const Pose safe_min = read_pose(root, "safe_min");
    const Pose safe_max = read_pose(root, "safe_max");
    for (int i = 0; i < N; ++i) {
        c.lo[i] = std::max(safe_min[i], info[i].min + HW_MARGIN);
        c.hi[i] = std::min(safe_max[i], info[i].max - HW_MARGIN);
        if (c.lo[i] >= c.hi[i]) throw std::runtime_error(std::string("Empty safe range for ") + kJointNames[i]);
        if (c.hold_pose[i] < c.lo[i] || c.hold_pose[i] > c.hi[i]) {
            throw std::runtime_error(std::string("hold_pose outside the safe range for ") + kJointNames[i]);
        }
    }
    const YAML::Node abort = root["abort"];
    c.abort_tracking_error = abort["tracking_error"].as<double>(0.3);
    c.abort_torque = abort["torque"].as<double>(18.0);
    c.abort_temperature = abort["temperature"].as<double>(70.0);
    c.abort_damping_kd = abort["damping_kd"].as<double>(1.0);
    c.expect_positive = root["expect_positive"];
    c.tests = root["tests"];
    return c;
}

class Rig {
public:
    Rig(motor_manager::MotorManager& manager, const std::array<MotorInfo, N>& info, const Config& config)
    : manager_(manager), info_(info), config_(config) {
        // Test gains = motor.yaml kp / kd (driver units), the same motor_command_ values ARTI-H1 walks with.
        for (int i = 0; i < N; ++i) {
            kp_[i] = manager.gains(static_cast<uint8_t>(i)).kp;
            kd_[i] = manager.gains(static_cast<uint8_t>(i)).kd;
        }
    }

    // Driver units -> joint side (Unitree takes rotor-side gains, CubeMars MIT output-side gains).
    double to_joint(int i) const { return info_[i].unitree ? info_[i].gear_ratio * info_[i].gear_ratio : 1.0; }

    const Pose& kp() const { return kp_; }

    const Pose& kd() const { return kd_; }

    void write_pose(const Pose& target) {
        motor_interface::motor_command_t cmd[N]{};
        for (int i = 0; i < N; ++i) {
            cmd[i].position = std::clamp(target[i], config_.lo[i], config_.hi[i]);
            cmd[i].kp = kp_[i];
            cmd[i].kd = kd_[i];
        }
        manager_.write(cmd);
    }

    // Pure damping; abort.damping_kd is joint side, converted to driver units.
    void damp() {
        motor_interface::motor_command_t cmd[N]{};
        for (int i = 0; i < N; ++i) cmd[i].kd = config_.abort_damping_kd / to_joint(i);
        manager_.write(cmd);
    }

    void read(motor_interface::motor_state_t* status) { manager_.read(status); }

    // Empty string = safe.
    std::string check(const Pose& target, const motor_interface::motor_state_t* status) const {
        if (!manager_.is_running()) return "motor communication stopped";
        std::ostringstream why;
        for (int i = 0; i < N; ++i) {
            const auto& s = status[i];
            if (s.position < info_[i].min - HW_MARGIN || s.position > info_[i].max + HW_MARGIN) {
                why << kJointNames[i] << " position " << s.position << " outside motor.yaml range";
            } else if (std::abs(std::clamp(target[i], config_.lo[i], config_.hi[i]) - s.position) > config_.abort_tracking_error) {
                why << kJointNames[i] << " tracking error " << target[i] - s.position << " rad";
            } else if (std::abs(s.torque) > config_.abort_torque) {
                why << kJointNames[i] << " torque " << s.torque << " Nm";
            } else if (s.temperature > config_.abort_temperature) {
                why << kJointNames[i] << " temperature " << s.temperature << " C";
            } else if (s.error != 0) {
                why << kJointNames[i] << " driver error code " << static_cast<int>(s.error);
            }
            if (why.tellp() > 0) return why.str();
        }
        return "";
    }

    // Run 'profile(t) -> pose' for 'duration' s at loop_period_us, logging every tick to 'csv_path' (if not empty).
    // Returns false when aborted (all joints are then left in pure damping).
    bool run(double duration, const std::function<Pose(double)>& profile, const std::string& csv_path,
             const std::string& header_comment) {
        std::FILE* csv = nullptr;
        if (!csv_path.empty()) {
            csv = std::fopen(csv_path.c_str(), "w");
            if (csv == nullptr) throw std::runtime_error("Cannot open " + csv_path);
            std::fprintf(csv, "# %s\n# kp_driver", header_comment.c_str());
            for (int i = 0; i < N; ++i) std::fprintf(csv, " %g", kp_[i]);
            std::fprintf(csv, "\n# kd_driver");
            for (int i = 0; i < N; ++i) std::fprintf(csv, " %g", kd_[i]);
            std::fprintf(csv, "\n# kp_joint");
            for (int i = 0; i < N; ++i) std::fprintf(csv, " %g", kp_[i] * to_joint(i));
            std::fprintf(csv, "\n# kd_joint");
            for (int i = 0; i < N; ++i) std::fprintf(csv, " %g", kd_[i] * to_joint(i));
            std::fprintf(csv, "\n# driver");
            for (int i = 0; i < N; ++i) std::fprintf(csv, " %s:%g", info_[i].unitree ? "unitree" : "cubemars", info_[i].gear_ratio);
            std::fprintf(csv, "\nt");
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
        motor_interface::motor_state_t status[N]{};
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
            write_pose(target);
            read(status);
            const std::string why = check(target, status);
            if (csv != nullptr) {
                std::fprintf(csv, "%.6f", t);
                for (int i = 0; i < N; ++i) {
                    std::fprintf(csv, ",%.6f,%.6f,%.5f,%.4f,%.0f", target[i], status[i].position, status[i].velocity,
                                 status[i].torque, status[i].temperature);
                }
                std::fprintf(csv, "\n");
            }
            if (!why.empty()) {
                damp();
                std::cerr << "[sysid] ABORT: " << why << " -> all joints in damping." << std::endl;
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

    // Straight-line move of every joint to 'goal' at move_speed (no log).
    bool move_to(const Pose& from, const Pose& goal) {
        double distance = 0.0;
        for (int i = 0; i < N; ++i) distance = std::max(distance, std::abs(goal[i] - from[i]));
        const double duration = std::max(0.5, distance / config_.move_speed);
        return run(duration, [&](double t) {
            const double a = std::min(1.0, t / duration);
            Pose p{};
            for (int i = 0; i < N; ++i) p[i] = from[i] + (goal[i] - from[i]) * a;
            return p;
        }, "", "") && run(0.5, [&](double) { return goal; }, "", "");
    }

private:
    motor_manager::MotorManager& manager_;
    const std::array<MotorInfo, N>& info_;
    const Config& config_;
    Pose kp_{};
    Pose kd_{};
};

void home(motor_manager::MotorManager& manager, motor_interface::motor_command_t* command, uint8_t index) {
    const auto& homing = manager.homing(index);
    const double dt = std::chrono::duration<double>(HOMING_PERIOD).count();
    motor_interface::motor_state_t status[N]{};
    manager.read(status);

    double target = status[index].position;
    command[index].position = target;
    command[index].kp = homing.kp;
    command[index].kd = homing.kd;
    manager.write(command);

    double velocity = 0.0;
    int stall_count = 0;
    const auto start = Clock::now();
    auto next = start;
    while (stall_count < HOMING_STALL_COUNT) {
        next += HOMING_PERIOD;
        std::this_thread::sleep_until(next);
        if (!running) throw std::runtime_error("Ctrl+C during homing.");
        if (!manager.is_running()) throw std::runtime_error("Motor communication stopped during homing.");
        const auto elapsed = Clock::now() - start;
        if (elapsed > HOMING_TIMEOUT) {
            command[index] = motor_interface::motor_command_t{};
            manager.write(command);
            throw std::runtime_error("Homing timeout on " + std::string(kJointNames[index]));
        }
        manager.read(status);
        const double position = status[index].position;
        target += homing.direction * homing.velocity * dt;
        if (homing.direction * (target - position) > HOMING_MAX_LEAD) target = position + homing.direction * HOMING_MAX_LEAD;
        command[index].position = target;
        manager.write(command);
        velocity += HOMING_VELOCITY_FILTER * (status[index].velocity - velocity);
        const double torque = status[index].torque * homing.direction;
        if (elapsed > HOMING_SETTLE_TIME && torque >= homing.torque_threshold &&
            std::abs(velocity) <= homing.velocity_threshold) {
            ++stall_count;
        } else {
            stall_count = 0;
        }
    }

    command[index] = motor_interface::motor_command_t{};
    command[index].kd = homing.hold_kd;
    manager.write(command);
    manager.set_homing_offset(index);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    manager.read(status);
    target = status[index].position;
    command[index].position = target;
    command[index].kp = homing.zero_kp;
    command[index].kd = homing.zero_kd;
    manager.write(command);
    const double step = homing.velocity * dt;
    next = Clock::now();
    while (target != 0.0) {
        next += HOMING_PERIOD;
        std::this_thread::sleep_until(next);
        if (!manager.is_running()) throw std::runtime_error("Motor communication stopped during homing.");
        target = std::abs(target) <= step ? 0.0 : target - std::copysign(step, target);
        command[index].position = target;
        manager.write(command);
    }
    const auto zero_start = Clock::now();
    while (true) {
        std::this_thread::sleep_for(HOMING_PERIOD);
        manager.read(status);
        if (std::abs(status[index].position) <= HOMING_ZERO_TOLERANCE) break;
        if (Clock::now() - zero_start > HOMING_ZERO_TIMEOUT) {
            command[index] = motor_interface::motor_command_t{};
            manager.write(command);
            throw std::runtime_error(std::string(kJointNames[index]) + " did not reach zero after homing.");
        }
    }
    command[index].position = 0.0;
    command[index].kp = homing.hold_kp;
    command[index].kd = homing.hold_kd;
    manager.write(command);
    std::cout << "[sysid] " << kJointNames[index] << " homed." << std::endl;
}

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
    if (argc < 4) {
        std::cerr << "Usage: " << argv[0] << " <motor_config.yaml> <sysid_config.yaml> <output_dir>" << std::endl;
        return 1;
    }
    std::signal(SIGINT, on_sigint);

    const auto info = load_motor_info(argv[1]);
    const Config config = load_config(argv[2], info);
    const std::filesystem::path out_dir(argv[3]);
    std::filesystem::create_directories(out_dir);

    std::cout << "[sysid] Safe range per joint (rad):" << std::endl;
    for (int i = 0; i < N; ++i) {
        std::cout << "  " << kJointNames[i] << "  [" << config.lo[i] << ", " << config.hi[i] << "]  "
                  << (info[i].unitree ? "unitree" : "cubemars") << " gear " << info[i].gear_ratio << std::endl;
    }
    if (!wait_for_enter("[sysid] Robot hanging, feet free, nothing near the legs?")) return 0;

    motor_manager::MotorManager manager(argv[1]);
    Rig rig(manager, info, config);
    std::cout << "[sysid] motor_command_ gains from motor.yaml (driver units -> joint side):" << std::endl;
    for (int i = 0; i < N; ++i) {
        std::cout << "  " << kJointNames[i] << "  kp " << rig.kp()[i] << " kd " << rig.kd()[i] << "  ->  "
                  << rig.kp()[i] * rig.to_joint(i) << " Nm/rad, " << rig.kd()[i] * rig.to_joint(i) << " Nm*s/rad" << std::endl;
    }
    if (!wait_for_enter("[sysid] Gains OK?")) return 0;
    manager.start();
    int exit_code = 0;

    try {
        motor_interface::motor_command_t command[N]{};
        manager.write(command);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        if (config.zero_mode == "homing") {
            for (int i = 0; i < N; ++i) home(manager, command, static_cast<uint8_t>(i));
        } else if (config.zero_mode == "current") {
            manager.set_zero_offset();
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        } else {
            throw std::runtime_error("zero_mode must be 'homing' or 'current'.");
        }

        Pose zero{};
        Pose start{};
        {
            motor_interface::motor_state_t status[N]{};
            rig.read(status);
            for (int i = 0; i < N; ++i) start[i] = status[i].position;
        }
        if (!rig.move_to(start, config.hold_pose)) throw std::runtime_error("Aborted while moving to hold_pose.");

        int test_number = 0;
        for (const auto& test : config.tests) {
            if (!running) break;
            const std::string type = test["type"].as<std::string>();
            ++test_number;
            char prefix[8];
            std::snprintf(prefix, sizeof(prefix), "%02d", test_number);

            if (type == "replay") {
                const std::string file = test["file"].as<std::string>();
                const YAML::Node log = YAML::LoadFile(file);
                const double dt = log["dt"].as<double>(1.0 / 60.0);
                std::vector<Pose> frames;
                for (const auto& step : log["joint_targets"]) {
                    Pose p{};
                    for (int i = 0; i < N; ++i) p[i] = step[i].as<double>();
                    frames.push_back(p);
                }
                if (frames.empty()) throw std::runtime_error("No joint_targets in " + file);
                if (!wait_for_enter("[sysid] " + std::string(prefix) + " replay " + file + " on ALL joints (" +
                                    std::to_string(frames.size() * dt) + " s).")) continue;
                if (!rig.move_to(config.hold_pose, frames.front())) break;
                const double duration = frames.size() * dt;
                const bool ok = rig.run(duration, [&](double t) {
                    return frames[std::min(frames.size() - 1, static_cast<std::size_t>(t / dt))];
                }, (out_dir / (std::string(prefix) + "_replay.csv")).string(), "type=replay file=" + file +
                   " dt=" + std::to_string(dt));
                if (!ok || !rig.move_to(frames.back(), config.hold_pose)) break;
                continue;
            }

            const std::string joint = test["joint"].as<std::string>();
            const int j = joint_index(joint);
            const double c = config.hold_pose[j];
            const double amplitude = test["amplitude"].as<double>();
            if (c + amplitude > config.hi[j] || c - amplitude < config.lo[j]) {
                std::cout << "[sysid] WARNING: " << joint << " +-" << amplitude << " around " << c
                          << " leaves the safe range and will be clipped." << std::endl;
            }

            double duration = 0.0;
            std::function<double(double)> offset;
            std::ostringstream header;
            header << "type=" << type << " joint=" << joint << " index=" << j << " center=" << c
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
                std::cout << "[sysid] " << joint << ": first moves +" << amplitude << " rad. Expected: " << expect
                          << ". Then -" << amplitude << " (the opposite)." << std::endl;
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
                const double f1 = test["f1"].as<double>(8.0);
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

            if (!wait_for_enter("[sysid] " + std::string(prefix) + " " + type + " " + joint + " (" +
                                std::to_string(duration) + " s).")) continue;
            const std::string csv = (out_dir / (std::string(prefix) + "_" + type + "_" + joint + ".csv")).string();
            const bool ok = rig.run(duration, [&](double t) {
                Pose p = config.hold_pose;
                p[j] = c + offset(t);
                return p;
            }, csv, header.str());
            std::cout << "[sysid] " << (ok ? "saved " : "ABORTED, partial log ") << csv << std::endl;
            if (!ok) break;
            if (type == "check" &&
                !wait_for_enter("[sysid] Did " + joint + " move as expected first? If NOT, quit (q) and fix motor.yaml axis.")) {
                break;
            }
        }

        if (running) {
            motor_interface::motor_state_t status[N]{};
            rig.read(status);
            Pose now{};
            for (int i = 0; i < N; ++i) now[i] = status[i].position;
            rig.move_to(now, zero);
        }
    } catch (const std::exception& e) {
        std::cerr << "[sysid] " << e.what() << std::endl;
        rig.damp();
        exit_code = 1;
    }

    rig.damp();
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    manager.stop();
    return exit_code;
}

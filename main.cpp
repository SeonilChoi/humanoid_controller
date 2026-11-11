#include <thread>
#include <chrono>
#include <csignal>
#include <iostream>
#include <stdexcept>
#include <string>

#include <yaml-cpp/yaml.h>

#include "robot/robots/ARTI_H1.hpp"

namespace {
volatile std::sig_atomic_t running = 1;
void on_sigint(int) { running = 0; }
}

int main(int argc, char* argv[])
{
    const auto usage = [&]() {
        std::cerr << "Usage: " << argv[0] << " <config_file> [--ppo | --model <file.onnx>] [--observe [--limp-right] | --blind-imu]" << std::endl;
        std::cerr << "  --ppo        run the RL-only (PPO) policy: robot.yaml ppo_model_file instead of model_file (AMP)" << std::endl;
        std::cerr << "  --model F    run the ONNX policy F instead of robot.yaml model_file (same obs / q_des interface)" << std::endl;
        std::cerr << "  --observe    home, stand in the default pose and print the observation at 5 Hz (no policy)" << std::endl;
        std::cerr << "  --limp-right with --observe: right leg limp (kp 0), prints the right-leg zero error vs the left leg" << std::endl;
        std::cerr << "               (SUPPORT THE ROBOT: it cannot stand on the right leg in this mode)" << std::endl;
        std::cerr << "  --blind-imu  test: the policy gets the standing tilt and zero angular velocity instead of the IMU" << std::endl;
        std::cerr << "               (the IMU is still logged and still stops the robot on a fall)" << std::endl;
    };
    if (argc < 2) {
        usage();
        return 1;
    }

    const std::string config_file = argv[1];
    std::string model_file;
    bool observe = false;
    bool blind_imu = false;
    bool limp_right = false;
    for (int i = 2; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--observe") {
            observe = true;
        } else if (arg == "--limp-right") {
            limp_right = true;
        } else if (arg == "--blind-imu") {
            blind_imu = true;
        } else if (arg == "--ppo") {
            const YAML::Node ppo = YAML::LoadFile(config_file)["ppo_model_file"];
            if (!ppo || ppo.as<std::string>().empty()) {
                std::cerr << "[main] --ppo: " << config_file << " has no ppo_model_file." << std::endl;
                return 1;
            }
            model_file = ppo.as<std::string>();
        } else if (arg == "--model" && i + 1 < argc) {
            model_file = argv[++i];
        } else {
            std::cerr << "[main] Unknown argument: " << arg << std::endl;
            usage();
            return 1;
        }
    }

    if (limp_right && !observe) {
        std::cerr << "[main] --limp-right only works with --observe." << std::endl;
        return 1;
    }

    std::signal(SIGINT, on_sigint);

    robot::ArtiH1 robot(config_file, model_file);

    if (blind_imu) {
        robot.set_blind_imu(true);
        std::cout << "[main] --blind-imu: the policy does NOT see the IMU (tilt latched at start, angular velocity 0)." << std::endl;
    }

    robot.start();

    try {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        robot.initialize();

        if (observe) {
            // observation check only: the motors keep holding the default pose from initialize(); the observation is
            // logged at the control rate (60 Hz) and printed at 5 Hz
            constexpr auto kObservePeriod = std::chrono::microseconds(1'000'000 / 60);
            if (limp_right) robot.set_limp_right();
            auto next_wakeup = std::chrono::steady_clock::now();
            int tick = 0;
            while (running) {
                if (!robot.motors_running()) throw std::runtime_error("[main] Motor communication stopped (see the [MotorManager::run] message above).");
                robot.observation();
                robot.log_observation();
                robot.update_limp_stats();
                if (limp_right) {
                    if (tick++ % 30 == 0) robot.print_limp_report();   // 2 Hz
                } else if (tick++ % 12 == 0) {
                    robot.print_observation();
                }
                next_wakeup += kObservePeriod;
                std::this_thread::sleep_until(next_wakeup);
            }
            robot.stop();
            return 0;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10000));

        constexpr auto kControlPeriod = std::chrono::microseconds(1'000'000 / 60);
        auto next_wakeup = std::chrono::steady_clock::now();

        while (running) {
            robot.observation();
            robot.control();

            next_wakeup += kControlPeriod;
            const auto now = std::chrono::steady_clock::now();
            if (next_wakeup < now) {
                next_wakeup = now;
            } else {
                std::this_thread::sleep_until(next_wakeup);
            }
        }
    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        robot.stop();
        return 1;
    }

    robot.stop();
    return 0;
}
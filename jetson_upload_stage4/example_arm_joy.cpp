// ARTI-D1 arm check with the DualSense: move each arm in URDF joint angles and compare with the real arm.
//
//   ./examples/example_arm_joy                     # = ../config/robot/ARTI-D1/motor.yaml ../config/robot/ARTI-D1/joy.yaml
//   ./examples/example_arm_joy <motor.yaml> <joy.yaml>
//
// Left stick = left arm, right stick = right arm:
//   stick down / up     -> {side}_arm_m8 + / -            (URDF limits -60 .. +20 deg)
//   stick in / out      -> {side}_arm_m8crank_plate + / - (URDF limits -13 .. +67 deg; "in" = toward the middle:
//                          left stick to the right, right stick to the left)
//   triangle            -> both arms slowly to 0 / 0 (arm origin, servo pulse 0)
//   circle              -> both arms slowly to m8 0 / plate -13 deg (681.yaml and the old default pose)
//   Ctrl+C              -> stop, every servo limp (hold the arms)
// The targets are URDF angles; the plate target goes to the real shoulder m7 servo through ARTI_D1_shoulder
// (= shoulder_ik.py), like robot::ArtiD1. The measured plate angle is solved back from the m8 / m7 readings.
// The legs stay limp the whole time. Target, measured URDF angles and the m7 servo angle print 5x per second.

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <exception>
#include <iostream>
#include <string>
#include <thread>

#include "joy/joy_handler.hpp"
#include "joy/core/joy_interface.hpp"
#include "motor/motor_manager.hpp"
#include "motor/core/motor_interface.hpp"
#include "robot/robots/ARTI_D1_shoulder.hpp"

namespace {
volatile std::sig_atomic_t running = 1;
void on_sigint(int) { running = 0; }

namespace shoulder = robot::arti_d1_shoulder;

constexpr int NUM_JOINTS = 14;
constexpr int kM8[2] = {0, 2};      // motor index of left / right arm_m8 (ID 1 / 3)
constexpr int kM7[2] = {4, 6};      // motor index of left / right shoulder m7 (ID 5 / 7)
constexpr const char* kSide[2] = {"L", "R"};
constexpr double kRate = 60.0;          // Hz
constexpr double kStickSpeed = 0.8;     // rad/s at full stick
constexpr double kMoveSpeed = 0.5;      // rad/s for the triangle / circle moves
constexpr double kDeadzone = 0.1;
constexpr double kMargin = 0.01;        // rad inside the URDF limits
constexpr double kDeg = 180.0 / M_PI;
constexpr double kPlateRest = -0.2268;  // 681.yaml plate angle

double dead(double v) { return std::abs(v) < kDeadzone ? 0.0 : v; }
double step_to(double from, double to, double max_step) { return from + std::clamp(to - from, -max_step, max_step); }
}

int main(int argc, char* argv[])
{
    const char* motor_config_file = argc > 1 ? argv[1] : "../config/robot/ARTI-D1/motor.yaml";
    const char* joy_config_file = argc > 2 ? argv[2] : "../config/robot/ARTI-D1/joy.yaml";
    if (std::string(motor_config_file) == "-h" || std::string(motor_config_file) == "--help") {
        std::cerr << "Usage: " << argv[0] << " [<motor.yaml> <joy.yaml>]" << std::endl;
        return 1;
    }
    std::signal(SIGINT, on_sigint);

    joy_handler::JoyHandler joy(joy_config_file);
    joy.start();

    motor_manager::MotorManager motor_manager(motor_config_file);
    if (motor_manager.number_of_motors() != NUM_JOINTS) {
        std::cerr << "motor config has " << motor_manager.number_of_motors() << " motors, ARTI-D1 has " << NUM_JOINTS << "." << std::endl;
        joy.stop();
        return 1;
    }
    motor_manager.start();

    // everything limp while checking the bus
    motor_interface::motor_command_t command[motor_interface::MAX_MOTORS]{};
    motor_manager.write(command);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    if (!motor_manager.is_running()) {
        std::cerr << "Motor communication stopped (see the [MotorManager::run] message above)." << std::endl;
        joy.stop();
        return 1;
    }

    // start from where the arms are (no jump): measured URDF angles
    motor_interface::motor_state_t status[motor_interface::MAX_MOTORS]{};
    motor_manager.read(status);
    std::array<double, 2> m8{}, plate{}, m7_prev{};
    for (int s = 0; s < 2; ++s) {
        const auto side = static_cast<shoulder::Side>(s);
        m8[s] = std::clamp(status[kM8[s]].position, shoulder::M8_MIN + kMargin, shoulder::M8_MAX - kMargin);
        m7_prev[s] = status[kM7[s]].position;
        plate[s] = std::clamp(shoulder::plate_from(side, m8[s], m7_prev[s]),
                              shoulder::PLATE_MIN + 0.0005, shoulder::PLATE_MAX - kMargin);
        std::printf("  %s arm at m8 %6.1f deg, plate %6.1f deg (m7 servo %6.1f deg)\n",
                    kSide[s], m8[s] * kDeg, plate[s] * kDeg, m7_prev[s] * kDeg);
    }
    std::cout << "Legs limp, arms hold where they are. Sticks move the arms, triangle = 0/0, circle = plate -13, "
                 "Ctrl+C = stop (arms go limp)." << std::endl;

    for (int s = 0; s < 2; ++s) {
        for (const int i : {kM8[s], kM7[s]}) {
            command[i].kp = motor_manager.gains(static_cast<uint8_t>(i)).kp;
            command[i].kd = motor_manager.gains(static_cast<uint8_t>(i)).kd;
        }
    }

    const double dt = 1.0 / kRate;
    const auto period = std::chrono::microseconds(static_cast<int>(1e6 / kRate));
    auto next_wakeup = std::chrono::steady_clock::now();
    bool moving_to_goal = false;
    std::array<double, 2> goal_m8{}, goal_plate{};
    long tick = 0;

    while (running && motor_manager.is_running()) {
        joy_interface::joy_data_t pad{};
        joy.read(pad);

        if (pad.triangle) { moving_to_goal = true; goal_m8 = {0.0, 0.0}; goal_plate = {0.0, 0.0}; }
        if (pad.circle) { moving_to_goal = true; goal_m8 = {0.0, 0.0}; goal_plate = {kPlateRest, kPlateRest}; }

        // stick per side (reversed 2026-11-04, user request): stick down = m8 +, stick in (toward the other stick)
        // = plate +, i.e. left stick right (+x) / right stick left (-x)
        const double stick_up[2] = {-dead(pad.stick_ly), -dead(pad.stick_ry)};
        const double stick_out[2] = {dead(pad.stick_lx), -dead(pad.stick_rx)};
        if (stick_up[0] != 0.0 || stick_up[1] != 0.0 || stick_out[0] != 0.0 || stick_out[1] != 0.0) {
            moving_to_goal = false;   // a stick takes over from a button move
        }

        for (int s = 0; s < 2; ++s) {
            const auto side = static_cast<shoulder::Side>(s);
            double new_m8 = m8[s], new_plate = plate[s];
            if (moving_to_goal) {
                new_m8 = step_to(m8[s], goal_m8[s], kMoveSpeed * dt);
                new_plate = step_to(plate[s], goal_plate[s], kMoveSpeed * dt);
            } else {
                new_m8 += stick_up[s] * kStickSpeed * dt;
                new_plate += stick_out[s] * kStickSpeed * dt;
            }
            new_m8 = std::clamp(new_m8, shoulder::M8_MIN + kMargin, shoulder::M8_MAX - kMargin);
            // the plate rest (-13 deg) sits on the URDF lower limit, so only a tiny margin there
            new_plate = std::clamp(new_plate, shoulder::PLATE_MIN + 0.0005, shoulder::PLATE_MAX - kMargin);
            try {
                const double m7 = shoulder::m7_from(side, new_m8, new_plate, m7_prev[s]);
                m8[s] = new_m8;
                plate[s] = new_plate;
                m7_prev[s] = m7;
            } catch (const std::exception&) {
                // the 5-bar cannot close there: keep the last reachable target
            }
            command[kM8[s]].position = m8[s];
            command[kM7[s]].position = m7_prev[s];
        }
        motor_manager.write(command);

        if (tick % static_cast<int>(kRate / 5) == 0) {
            motor_manager.read(status);
            for (int s = 0; s < 2; ++s) {
                const auto side = static_cast<shoulder::Side>(s);
                const double meas_m8 = status[kM8[s]].position;
                const double meas_plate = shoulder::plate_from(side, meas_m8, status[kM7[s]].position);
                std::printf("%s m8 %6.1f/%6.1f  plate %6.1f/%6.1f  m7 %6.1f/%6.1f   ", kSide[s],
                            m8[s] * kDeg, meas_m8 * kDeg, plate[s] * kDeg, meas_plate * kDeg,
                            m7_prev[s] * kDeg, status[kM7[s]].position * kDeg);
            }
            std::printf("(target/measured deg)\n");
        }

        ++tick;
        next_wakeup += period;
        const auto now = std::chrono::steady_clock::now();
        if (next_wakeup < now) next_wakeup = now; else std::this_thread::sleep_until(next_wakeup);
    }
    if (!motor_manager.is_running()) {
        std::cerr << "Motor communication stopped (see the [MotorManager::run] message above)." << std::endl;
    }
    std::cout << "Stopping: every servo limp." << std::endl;
    motor_manager.stop();
    joy.stop();
    return 0;
}

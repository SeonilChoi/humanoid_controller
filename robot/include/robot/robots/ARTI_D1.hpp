#ifndef ROBOT_ROBOTS_ARTI_D1_HPP_
#define ROBOT_ROBOTS_ARTI_D1_HPP_

#include <array>
#include <chrono>
#include <cstdio>
#include <string>
#include <unordered_map>
#include <vector>

#include "robot/core/robot.hpp"
#include "robot/controllers/amp_controller.hpp"

namespace robot {

class ArtiD1 : public Robot {
public:
    explicit ArtiD1(const std::string& config_file);

    virtual ~ArtiD1();

    void initialize() override;

    void observation() override;

    void control() override;

    // Observation check (main.cpp --observe): print the last observation() in readable units (no policy, the motors
    // keep holding the default pose). Tilt / move the robot by hand and check the signs.
    void print_observation() const;

    // Append the last observation() to the runtime log (if robot.yaml log_file is set); used by --observe.
    void log_observation();

private:
    std::vector<pinocchio::JointIndex> joint_ids_;

    std::unordered_map<pinocchio::JointIndex, uint8_t> joint_id_to_motor_index_;

    uint8_t imu_sensor_index_{};

    pinocchio::FrameIndex imu_frame_id_{};

    Eigen::Vector3d imu_translation_{};

    Eigen::Quaterniond frame_q_imu_{
        Eigen::Quaterniond::Identity()
    };

    std::vector<pinocchio::FrameIndex> foot_frame_ids_;

    std::vector<Eigen::Vector3d> foot_toe_offsets_;

    // Policy settings from robot.yaml 'policy' (must match the training env, see arti_d1_env_cfg.py).
    // nullptr while robot.yaml model_file does not exist yet (only --observe works then).
    controller::AmpController* amp_controller_{nullptr};

    std::array<double, 14> default_pose_{};   // first reference frame, policy order

    std::array<double, 14> policy_kp_{};      // motor.yaml kp / kd = Dynamixel Position P / D Gain, motor index order

    std::array<double, 14> policy_kd_{};

    std::array<double, 3> command_scale_{};   // observation = command * command_scale

    std::array<double, 3> command_max_{};     // command at full stick: vx (m/s), vy (m/s), yaw rate (rad/s)

    double gait_phase_rate_{0.0};             // gait clock, cycles per second

    double control_dt_{1.0 / 60.0};

    double gait_phase_{0.0};                  // 0 = left single support start, as in the reference clips

    double initial_gait_phase_{0.0};          // clock at the policy start (training standstill = reference frame 0)

    double max_tilt_{0.7};                     // rad; beyond it the policy stops (safe_stop)

    // Standing (training cfg gait_rest_phases / stand_command_threshold): under a zero command the clock runs on to
    // the next rest phase (mid double support) and stops there; empty = the clock always runs.
    std::vector<double> gait_rest_phases_;

    double stand_command_threshold_{0.0};

    double stick_deadzone_{0.0};                 // |stick| below it counts as 0, so a released stick means "stand"

    std::array<double, 3> command_{};           // last command (vx m/s, vy m/s, yaw rate rad/s)

    std::array<double, 14> previous_actions_{};

    // Runtime log (robot.yaml 'log_file', empty = off): one CSV row per control step with the observation, the ONNX
    // outputs, the motor states and the raw IMU, so a real run can be replayed / compared with the simulator.
    std::FILE* log_file_{nullptr};

    std::chrono::steady_clock::time_point log_start_{};

    motor_interface::motor_state_t last_motor_status_[14]{};

    std::array<double, 4> last_imu_orientation_{};   // w, x, y, z (sensor to world)

    std::array<double, 3> last_imu_angular_velocity_{};

    void write_log_row();

    // Position P Gain 0 on every servo (only the motor.yaml D gain acts), then throw: used when the robot falls or
    // the motor link dies.
    [[noreturn]] void safe_stop(const std::string& reason);
};

} // namespace robot

#endif // ROBOT_ROBOTS_ARTI_D1_HPP_
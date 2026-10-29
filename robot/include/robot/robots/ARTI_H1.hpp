#ifndef ROBOT_ROBOTS_ARTI_H1_HPP_
#define ROBOT_ROBOTS_ARTI_H1_HPP_

#include <array>
#include <chrono>
#include <cstdio>
#include <string>
#include <unordered_map>
#include <vector>

#include "robot/core/robot.hpp"
#include "robot/controllers/amp_controller.hpp"
#include "sensor/core/sensor_interface.hpp"

namespace robot {

class ArtiH1 : public Robot {
public:
    explicit ArtiH1(const std::string& config_file);

    virtual ~ArtiH1();

    void initialize() override;

    void observation() override;

    void control() override;

    // Observation check (main.cpp --observe): print every value of the last observation() in readable units (no
    // policy, the motors keep holding the default pose). Tilt / move the robot by hand and check the signs.
    void print_observation() const;

    // Append the last observation() to the runtime log (if robot.yaml log_file is set); used by --observe.
    void log_observation();

    // Test mode (main.cpp --blind-imu): the policy gets the pelvis tilt latched at the first observation and zero
    // angular velocity instead of the IMU; the real IMU is still logged and still used for the max_tilt fall check.
    void set_blind_imu(bool blind) { blind_imu_ = blind; }

private:
    void home(uint8_t index);

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

    // Policy settings from robot.yaml 'policy' (must match the training env, see arti_h1_env_cfg.py).
    controller::AmpController* amp_controller_{nullptr};

    std::array<double, 12> default_pose_{};   // first reference frame, policy order

    std::array<double, 12> policy_kp_{};      // motor.yaml kp / kd, driver units, motor index order

    std::array<double, 12> policy_kd_{};

    std::array<double, 3> command_scale_{};   // observation = command * command_scale

    std::array<double, 3> command_max_{};     // command at full stick: vx (m/s), vy (m/s), yaw rate (rad/s)

    double gait_phase_rate_{0.0};             // gait clock, cycles per second

    double control_dt_{1.0 / 60.0};

    double gait_phase_{0.0};                  // 0 = left single support start, as in the reference clips

    double initial_gait_phase_{0.0};          // clock at the policy start (training standstill = reference frame 0)

    double max_tilt_{0.7};

    double imu_still_time_{0.0};              // s: hold the default pose still to average the gyro bias; 0 = off

    double imu_converge_tolerance_{0.5};      // deg: IMU roll / pitch vs accelerometer tilt before the policy starts

    double imu_converge_timeout_{180.0};      // s: start anyway (with a warning) after this long

    std::array<double, 3> imu_gyro_bias_{};   // rad/s, sensor frame; subtracted from the gyro in observation()

    double imu_attitude_gain_{0.0};           // 1/s: own gyro + accelerometer attitude filter; 0 = MTi orientation

    Eigen::Quaterniond attitude_q_{Eigen::Quaterniond::Identity()};   // sensor to world (own filter)

    bool attitude_initialized_{false};

    std::chrono::steady_clock::time_point attitude_time_{};

    std::array<double, 4> last_attitude_{};   // w, x, y, z: the orientation the observation used

    double imu_stale_time_{0.2};              // s: stop when the IMU gyro / accelerometer samples stop changing this long

    std::array<double, 6> imu_last_sample_{};

    std::chrono::steady_clock::time_point imu_sample_time_{};

    bool imu_stale_{false};                   // set in observation(), control() stops on it

    bool blind_imu_{false};

    bool blind_latched_{false};

    Eigen::Matrix3d blind_heading_R_root_{Eigen::Matrix3d::Identity()};

    double true_tilt_cos_{1.0};               // z of the real pelvis z axis (max_tilt check, also when blind)

    Eigen::Quaterniond update_attitude(const sensor_interface::imu_data_t& imu_data, const Eigen::Quaterniond& mti_q);

    // Standing (training cfg gait_rest_phases / stand_command_threshold): under a zero command the clock runs on to
    // the next rest phase (mid double support) and stops there; empty = the clock always runs.
    std::vector<double> gait_rest_phases_;

    double stand_command_threshold_{0.0};

    double stick_deadzone_{0.0};                 // |stick| below it counts as 0, so a released stick means "stand"

    std::array<double, 3> command_{};           // last command (vx m/s, vy m/s, yaw rate rad/s)                    // rad; beyond it the policy stops and the joints are only damped

    std::array<double, 12> previous_actions_{};

    // Runtime log (robot.yaml 'log_file', empty = off): one CSV row per control step with the observation, the ONNX
    // outputs, the motor states and the raw IMU, so a real run can be replayed / compared with the simulator.
    std::FILE* log_file_{nullptr};

    std::chrono::steady_clock::time_point log_start_{};

    motor_interface::motor_state_t last_motor_status_[12]{};

    std::array<double, 4> last_imu_orientation_{};   // w, x, y, z (sensor to world)

    std::array<double, 3> last_imu_angular_velocity_{};

    std::array<double, 3> last_imu_linear_acceleration_{};   // sensor frame, m/s^2 (gravity included)

    double last_imu_temperature_{0.0};

    void write_log_row();

    // Damping only (kp 0) on every joint, then throw: used when the robot falls or the motor link dies.
    [[noreturn]] void safe_stop(const std::string& reason);
};

} // namespace robot

#endif // ROBOT_ROBOTS_ARTI_H1_HPP_
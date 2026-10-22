#ifndef ROBOT_ROBOTS_ARTI_H1_HPP_
#define ROBOT_ROBOTS_ARTI_H1_HPP_

#include <array>
#include <unordered_map>

#include "robot/core/robot.hpp"
#include "robot/controllers/amp_controller.hpp"

namespace robot {

class ArtiH1 : public Robot {
public:
    explicit ArtiH1(const std::string& config_file);

    virtual ~ArtiH1() = default;

    void initialize() override;

    void observation() override;

    void control() override;

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

    std::array<double, 12> previous_actions_{};
};

} // namespace robot

#endif // ROBOT_ROBOTS_ARTI_H1_HPP_
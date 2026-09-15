#ifndef ROBOT_ROBOTS_OLAF_HPP_
#define ROBOT_ROBOTS_OLAF_HPP_

#include <array>
#include <unordered_map>

#include "robot/core/robot.hpp"

namespace olaf {

class Olaf : public robot::Robot {
public:
    explicit Olaf(const std::string& config_file);

    virtual ~Olaf() = default;

    void observation(std::vector<double>& observation) override;

    void control(const std::vector<double>& observation, std::vector<double>& action) override;

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
};

} // namespace olaf

#endif // ROBOT_ROBOTS_OLAF_HPP_
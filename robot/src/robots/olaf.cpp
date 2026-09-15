#include <cmath>
#include <iostream>
#include <stdexcept>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "motor/core/motor_interface.hpp"
#include "sensor/core/sensor_interface.hpp"
#include "joy/core/joy_interface.hpp"

#include "controller/amp_controller.hpp"

#include "sensor/core/imu.hpp"
#include "robot/robots/olaf.hpp"

namespace {

constexpr std::size_t NUM_JOINTS = 12;
constexpr std::size_t OBSERVATION_SIZE = 104;
constexpr double PI = 3.14159265358979323846;

void append_vector(const Eigen::Vector3d& value, std::vector<double>& vector) {
    vector.push_back(value.x());
    vector.push_back(value.y());
    vector.push_back(value.z());
}

void append_rotation(const Eigen::Matrix3d& value, std::vector<double>& vector) {
    append_vector(value.col(0), vector);
    append_vector(value.col(2), vector);
}

} // namespace

olaf::Olaf::Olaf(const std::string& config_file)
    : robot::Robot(config_file) {
    if (motors_.size() != NUM_JOINTS) {
        throw std::runtime_error("[Olaf::Olaf] Invalid number of joints.");
    }

    for (const auto& motor : motors_) {
        const auto joint_id = kinematics_->joint_id(motor.name);
        
        joint_ids_.push_back(joint_id);
        joint_id_to_motor_index_[joint_id] = motor.index;
    }

    for (const auto& sensor : sensors_) {
        if (sensor.name == "imu_body") {
            imu_sensor_index_ = sensor.index;
            imu_frame_id_ = kinematics_->frame_id(sensor.frame);
            imu_translation_ = Eigen::Vector3d(sensor.translation.data());
            frame_q_imu_ = Eigen::Quaterniond(sensor.orientation.data()).normalized();
        }
    }

    for (const auto& foot : foots_) {
        foot_frame_ids_.push_back(kinematics_->frame_id(foot.name));
        foot_toe_offsets_.push_back(Eigen::Vector3d(foot.offset.data()));
    }

    controller_ = std::make_unique<amp::AmpController>(model_file_);
}

void olaf::Olaf::observation(std::vector<double>& observation) {
    // read motor status
    /*
    motor_interface::motor_state_t motor_status[NUM_JOINTS]{};
    motor_manager_->read(motor_status);
    */

    observation.clear();
    observation.reserve(OBSERVATION_SIZE);

    motor_interface::motor_state_t motor_status[4]{};
    motor_manager_->read(motor_status);
    
    // update kinematics with current motor positions
    Eigen::VectorXd joint_positions = Eigen::VectorXd::Zero(NUM_JOINTS);

    /*
    for (const auto& joint_id : joint_ids_) {
        const int joint_index = kinematics_->joint_index(joint_id);
        joint_positions[joint_index] = motor_status[joint_id_to_motor_index_.at(joint_id)].position;
    }
    */

    kinematics_->update(joint_positions);

    // read IMU data
    sensor_interface::imu_data_t imu_data{};
    sensor_manager_->sensor<sensor_interface::Imu>(imu_sensor_index_).read(imu_data);

    Eigen::Quaterniond world_q_imu(
        imu_data.orientation[0],
        imu_data.orientation[1],
        imu_data.orientation[2],
        imu_data.orientation[3]
    );
    if (world_q_imu.norm() < 1.0e-8) {
        throw std::runtime_error("[Olaf::observation] Invalid IMU orientation.");
    }
    world_q_imu.normalize();

    const Eigen::Matrix3d world_R_imu = world_q_imu.toRotationMatrix();

    const Eigen::Matrix3d root_R_frame = kinematics_->frame_rotation(imu_frame_id_);

    const Eigen::Matrix3d frame_R_imu = frame_q_imu_.toRotationMatrix();

    const Eigen::Matrix3d root_R_imu = root_R_frame * frame_R_imu;

    const Eigen::Matrix3d world_R_root = world_R_imu * root_R_imu.transpose();

    const double heading = std::atan2(world_R_root(1, 0), world_R_root(0, 0));

    const Eigen::Matrix3d heading_R_world = Eigen::AngleAxisd(
        -heading,
        Eigen::Vector3d::UnitZ()
    ).toRotationMatrix();

    const Eigen::Matrix3d heading_R_root = heading_R_world * world_R_root;

    const Eigen::Vector3d angular_velocity_imu(
        imu_data.angular_velocity[0],
        imu_data.angular_velocity[1],
        imu_data.angular_velocity[2]
    );

    const Eigen::Vector3d angular_velocity_world = world_R_imu * angular_velocity_imu;

    const Eigen::Vector3d angular_velocity_heading = heading_R_world * angular_velocity_world;

    // heading rotation [0:6]
    append_rotation(heading_R_root, observation);

    // body angular velocity in heading frame [6:9]
    append_vector(angular_velocity_heading, observation);

    // joint rotations [9:81]
    for (const auto& joint_id : joint_ids_) {
        append_rotation(kinematics_->joint_local_rotation(joint_id), observation);
    }

    // joint velocities [81:93]
    for (const auto& joint_id : joint_ids_) {
        observation.push_back(motor_status[joint_id_to_motor_index_.at(joint_id)].velocity);
    }

    // foot positions [93:99]
    for (std::size_t i = 0; i < foot_frame_ids_.size(); ++i) {
        const auto foot_frame_id = foot_frame_ids_[i];
        const auto foot_toe_offset = foot_toe_offsets_[i];

        const Eigen::Vector3d toe_position = 
            kinematics_->frame_position(foot_frame_id) +
            kinematics_->frame_rotation(foot_frame_id) * foot_toe_offset;

        append_vector(heading_R_root * toe_position, observation);
    }

    // read joy data
    joy_interface::joy_data_t joy_data{};
    joy_handler_->read(joy_data);

    // command [99:102]
    observation.push_back(joy_data.stick_ly * 0.5);
    observation.push_back(-joy_data.stick_lx * 0.5);
    observation.push_back(-joy_data.stick_rx * 0.25);
}

void olaf::Olaf::control(const std::vector<double>& observation, std::vector<double>& action) {

    controller_->update(observation, action);

    motor_interface::motor_command_t motor_command[4]{};

    motor_command[0].kp = 0.5;
    motor_command[0].kd = 0.1;
    motor_command[1].kp = 0.5;
    motor_command[1].kd = 0.1;
    motor_command[2].kp = 10.0;
    motor_command[2].kd = 5.0;
    motor_command[3].kp = 10.0;
    motor_command[3].kd = 5.0;

    for (std::size_t i = 0; i < 4; ++i) {
        motor_command[i].position = action[i];
    }

    motor_manager_->write(motor_command);
}
#include <cmath>
#include <stdexcept>

#include <Eigen/Core>
#include <Eigen/Geometry>

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

    test_command_[0].position = 1.5;
    test_command_[0].velocity = 0.0;
    test_command_[0].torque = 0.0;
    test_command_[0].kp = 0.1;
    test_command_[0].kd = 0.1;

    test_command_[1].position = 0.0;
    test_command_[1].velocity = 0.0;
    test_command_[1].torque = 0.0;
    test_command_[1].kp = 0.1;
    test_command_[1].kd = 0.1;

    test_command_[2].position = 1.5;
    test_command_[2].velocity = 0.0;
    test_command_[2].torque = 0.0;
    test_command_[2].kp = 10.0;
    test_command_[2].kd = 5.0;

    test_command_[3].position = 0.0;
    test_command_[3].velocity = 0.0;
    test_command_[3].torque = 0.0;
    test_command_[3].kp = 10.0;
    test_command_[3].kd = 5.0;
}

const std::vector<double>& olaf::Olaf::observation() {
    // read motor status
    /*
    motor_interface::motor_state_t motor_status[NUM_JOINTS]{};
    motor_manager_->read(motor_status);
    */

    motor_interface::motor_state_t motor_status[4]{};
    motor_manager_->read(motor_status);
    
    // update kinematics with current motor positions
    Eigen::VectorXd joint_positions = Eigen::VectorXd::Zero(NUM_JOINTS);

    for (const auto& joint_id : joint_ids_) {
        const int joint_index = kinematics_->joint_index(joint_id);
        joint_positions[joint_index] = motor_status[joint_id_to_motor_index_.at(joint_id)].position;
    }

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

    observation_.clear();
    observation_.reserve(OBSERVATION_SIZE);

    // heading rotation [0:6]
    append_rotation(heading_R_root, observation_);

    // body angular velocity in heading frame [6:9]
    append_vector(angular_velocity_heading, observation_);

    // joint rotations [9:81]
    for (const auto& joint_id : joint_ids_) {
        append_rotation(kinematics_->joint_local_rotation(joint_id), observation_);
    }

    // joint velocities [81:93]
    for (const auto& joint_id : joint_ids_) {
        observation_.push_back(motor_status[joint_id_to_motor_index_.at(joint_id)].velocity);
    }

    // foot positions [93:99]
    for (std::size_t i = 0; i < foot_frame_ids_.size(); ++i) {
        const auto foot_frame_id = foot_frame_ids_[i];
        const auto foot_toe_offset = foot_toe_offsets_[i];

        const Eigen::Vector3d toe_position = 
            kinematics_->frame_position(foot_frame_id) +
            kinematics_->frame_rotation(foot_frame_id) * foot_toe_offset;

        append_vector(heading_R_root * toe_position, observation_);
    }

    const double phase_angle = 2.0 * PI * gait_phase_;

    // gait phase [99:101]
    observation_.push_back(std::sin(phase_angle));
    observation_.push_back(std::cos(phase_angle));

    // command [101:104]
    observation_.push_back(command_[0] * 0.5);
    observation_.push_back(command_[1] * 0.5);
    observation_.push_back(command_[2] * 0.25);

    return observation_;
}


void olaf::Olaf::control() {
    if (test_count_ % 100 == 0) {
        double tmp = test_command_[1].position;
        test_command_[1].position = test_command_[0].position;
        test_command_[0].position = tmp;

        tmp = test_command_[3].position;
        test_command_[3].position = test_command_[2].position;
        test_command_[2].position = tmp;
    }
    test_count_++;

    motor_manager_->write(test_command_);
}
#include <algorithm>
#include <array>
#include <cmath>
#include <string>
#include <thread>
#include <chrono>
#include <iostream>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "motor/core/motor_interface.hpp"
#include "sensor/core/sensor_interface.hpp"
#include "joy/core/joy_interface.hpp"

#include "sensor/core/imu.hpp"
#include "robot/controllers/amp_controller.hpp"

#include "robot/robots/ARTI_H1.hpp"

namespace {

constexpr std::size_t NUM_JOINTS = 12;
// root tan-norm 6, angular velocity 3, joint tan-norm 72, joint velocity 12, toes 6, previous actions 12,
// gait phase sin/cos 2, commands 3 (ArtiH1AmpEnv._get_observations).
constexpr std::size_t OBSERVATION_SIZE = 116;
constexpr auto STAND_UP_TIME = std::chrono::seconds(3);
constexpr double PI = 3.14159265358979323846;

constexpr auto HOMING_PERIOD = std::chrono::milliseconds(10);
constexpr auto HOMING_SETTLE_TIME = std::chrono::milliseconds(500);
constexpr auto HOMING_TIMEOUT = std::chrono::seconds(15);
constexpr int HOMING_STALL_COUNT = 10;
constexpr double HOMING_MAX_LEAD = 0.15;
constexpr double HOMING_VELOCITY_FILTER = 0.2;
constexpr double HOMING_ZERO_TOLERANCE = 0.03;
constexpr auto HOMING_ZERO_TIMEOUT = std::chrono::seconds(3);

constexpr const char* kPolicyJointNames[NUM_JOINTS] = {
    "left_hip_yaw",
    "right_hip_yaw",
    "left_hip_roll",
    "right_hip_roll",
    "left_hip_pitch",
    "right_hip_pitch",
    "left_knee",
    "right_knee",
    "left_ankle_pitch",
    "right_ankle_pitch",
    "left_ankle_roll",
    "right_ankle_roll",
};

void append_vector(const Eigen::Vector3d& value, std::vector<double>& vector) {
    vector.push_back(value.x());
    vector.push_back(value.y());
    vector.push_back(value.z());
}

template <std::size_t M>
std::array<double, M> read_array(const YAML::Node& node, const char* key) {
    const auto values = node[key].as<std::vector<double>>();
    if (values.size() != M) {
        throw std::runtime_error(std::string("[ArtiH1] robot.yaml policy.") + key + " needs " + std::to_string(M) + " values.");
    }
    std::array<double, M> out{};
    std::copy(values.begin(), values.end(), out.begin());
    return out;
}

void append_rotation(const Eigen::Matrix3d& value, std::vector<double>& vector) {
    append_vector(value.col(0), vector);
    append_vector(value.col(2), vector);
}

} // namespace

robot::ArtiH1::ArtiH1(const std::string& config_file)
    : robot::Robot(config_file) {
    if (motors_.size() != NUM_JOINTS) {
        throw std::runtime_error("[ArtiH1::ArtiH1] Invalid number of joints.");
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

    auto amp_controller = std::make_unique<controller::AmpController>(model_file_);
    amp_controller_ = amp_controller.get();
    controller_ = std::move(amp_controller);
    controller_->initialize();

    const YAML::Node policy = YAML::LoadFile(config_file)["policy"];
    if (!policy) throw std::runtime_error("[ArtiH1::ArtiH1] robot.yaml has no 'policy' section.");
    default_pose_ = read_array<NUM_JOINTS>(policy, "default_pose");
    // Walking PD gains: motor.yaml 'kp' / 'kd' per motor (driver units, the real robot gains).
    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        policy_kp_[i] = motor_manager_->gains(static_cast<uint8_t>(i)).kp;
        policy_kd_[i] = motor_manager_->gains(static_cast<uint8_t>(i)).kd;
    }
    command_scale_ = read_array<3>(policy, "command_scale");
    command_max_ = read_array<3>(policy, "command_max");
    gait_phase_rate_ = policy["gait_phase_rate"].as<double>();
    control_dt_ = policy["control_dt"].as<double>();

    observation_.clear();
    action_.clear();

    for (std::size_t i = 0; i < 8; ++i) {
        motor_command_[i].kp = 0.0;
        motor_command_[i].kd = 0.0;
    }
    for (std::size_t i = 8; i < 12; ++i) {
        motor_command_[i].kp = 0.0;
        motor_command_[i].kd = 0.0;
    }
}

void robot::ArtiH1::initialize() {
    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        motor_command_[i] = motor_interface::motor_command_t{};
    }
    motor_manager_->write(motor_command_);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    for (const auto& motor : motors_) {
        home(motor.index);
    }

    // homed motors hold 0.0 with weak gains (homing_hold_kp / homing_hold_kd); now move to the policy's default
    // pose with the policy gains, so the first q_des does not jump.
    motor_interface::motor_state_t motor_status[NUM_JOINTS]{};
    motor_manager_->read(motor_status);
    std::array<double, NUM_JOINTS> start{};
    std::array<double, NUM_JOINTS> goal{};
    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        const auto motor_index = joint_id_to_motor_index_.at(kinematics_->joint_id(kPolicyJointNames[i]));
        start[motor_index] = motor_status[motor_index].position;
        goal[motor_index] = default_pose_[i];
    }
    const auto begin = std::chrono::steady_clock::now();
    const double duration = std::chrono::duration<double>(STAND_UP_TIME).count();
    auto next_wakeup = begin;
    while (true) {
        const double t = std::chrono::duration<double>(std::chrono::steady_clock::now() - begin).count();
        const double alpha = std::min(1.0, t / duration);
        for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
            motor_command_[i].position = start[i] + (goal[i] - start[i]) * alpha;
            motor_command_[i].kp = policy_kp_[i];
            motor_command_[i].kd = policy_kd_[i];
        }
        motor_manager_->write(motor_command_);
        if (alpha >= 1.0) break;
        next_wakeup += HOMING_PERIOD;
        std::this_thread::sleep_until(next_wakeup);
    }
    gait_phase_ = 0.0;
    previous_actions_.fill(0.0);
}

void robot::ArtiH1::home(uint8_t index) {
    const auto& homing = motor_manager_->homing(index);
    const double dt = std::chrono::duration<double>(HOMING_PERIOD).count();

    motor_interface::motor_state_t motor_status[NUM_JOINTS]{};
    motor_manager_->read(motor_status);

    // only the homing motor is driven, already homed motors hold 0.0, the rest stay passive
    double target = motor_status[index].position;
    motor_command_[index].position = target;
    motor_command_[index].kp = homing.kp;
    motor_command_[index].kd = homing.kd;
    motor_manager_->write(motor_command_);

    double velocity = 0.0;
    int stall_count = 0;
    int debug_count = 0;

    const auto start = std::chrono::steady_clock::now();
    auto next_wakeup = start;

    while (stall_count < HOMING_STALL_COUNT) {
        next_wakeup += HOMING_PERIOD;
        std::this_thread::sleep_until(next_wakeup);
        if (!motor_manager_->is_running()) {
            throw std::runtime_error("[ArtiH1::home] Motor communication stopped on motor index " + std::to_string(index) + ".");
        }

        const auto elapsed = std::chrono::steady_clock::now() - start;
        if (elapsed > HOMING_TIMEOUT) {
            motor_command_[index] = motor_interface::motor_command_t{};
            motor_manager_->write(motor_command_);
            throw std::runtime_error("[ArtiH1::home] Timeout on motor index " + std::to_string(index) + ".");
        }

        motor_manager_->read(motor_status);
        const double position = motor_status[index].position;

        // move the target slowly, but never more than HOMING_MAX_LEAD ahead of the motor,
        // so the force at the stopper stays around kp * HOMING_MAX_LEAD
        target += homing.direction * homing.velocity * dt;
        if (homing.direction * (target - position) > HOMING_MAX_LEAD) {
            target = position + homing.direction * HOMING_MAX_LEAD;
        }
        motor_command_[index].position = target;
        motor_manager_->write(motor_command_);

        // stalled against the stopper: pushing in the homing direction but not moving
        velocity += HOMING_VELOCITY_FILTER * (motor_status[index].velocity - velocity);
        const double torque = motor_status[index].torque * homing.direction;

        if (++debug_count % 10 == 0) {
            std::cout << "[ArtiH1::home] index " << static_cast<int>(index)
                      << " target " << target
                      << " position " << position
                      << " velocity " << velocity
                      << " torque " << motor_status[index].torque << std::endl;
        }

        if (elapsed > HOMING_SETTLE_TIME &&
            torque >= homing.torque_threshold &&
            std::abs(velocity) <= homing.velocity_threshold) {
            ++stall_count;
        } else {
            stall_count = 0;
        }
    }

    // damping only while the offset changes: a position hold would jump when the position frame shifts
    motor_command_[index] = motor_interface::motor_command_t{};
    motor_command_[index].kd = homing.hold_kd;
    motor_manager_->write(motor_command_);

    motor_manager_->set_homing_offset(index);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    std::cout << "[ArtiH1::home] Motor index " << static_cast<int>(index) << " homed." << std::endl;

    // move from the stopper (homing offset) to 0.0
    motor_manager_->read(motor_status);
    target = motor_status[index].position;

    motor_command_[index].position = target;
    motor_command_[index].kp = homing.zero_kp;
    motor_command_[index].kd = homing.zero_kd;
    motor_manager_->write(motor_command_);

    const double step = homing.velocity * dt;
    next_wakeup = std::chrono::steady_clock::now();

    while (target != 0.0) {
        next_wakeup += HOMING_PERIOD;
        std::this_thread::sleep_until(next_wakeup);
        if (!motor_manager_->is_running()) {
            throw std::runtime_error("[ArtiH1::home] Motor communication stopped on motor index " + std::to_string(index) + ".");
        }

        target = std::abs(target) <= step ? 0.0 : target - std::copysign(step, target);
        motor_command_[index].position = target;
        motor_manager_->write(motor_command_);
    }

    // wait until the motor actually reaches 0.0
    const auto zero_start = std::chrono::steady_clock::now();

    while (true) {
        std::this_thread::sleep_for(HOMING_PERIOD);
        if (!motor_manager_->is_running()) {
            throw std::runtime_error("[ArtiH1::home] Motor communication stopped on motor index " + std::to_string(index) + ".");
        }

        motor_manager_->read(motor_status);

        if (std::abs(motor_status[index].position) <= HOMING_ZERO_TOLERANCE) break;

        if (std::chrono::steady_clock::now() - zero_start > HOMING_ZERO_TIMEOUT) {
            motor_command_[index] = motor_interface::motor_command_t{};
            motor_manager_->write(motor_command_);
            throw std::runtime_error("[ArtiH1::home] Motor index " + std::to_string(index) +
                                     " did not reach zero, position " + std::to_string(motor_status[index].position) + ".");
        }
    }

    std::cout << "[ArtiH1::home] Motor index " << static_cast<int>(index)
              << " moved to zero, position " << motor_status[index].position << std::endl;

    // keep holding 0.0 with weak gains
    motor_command_[index].position = 0.0;
    motor_command_[index].kp = homing.hold_kp;
    motor_command_[index].kd = homing.hold_kd;
    motor_manager_->write(motor_command_);
}

void robot::ArtiH1::observation() {
    observation_.clear();
    observation_.reserve(OBSERVATION_SIZE);

    motor_interface::motor_state_t motor_status[NUM_JOINTS]{};
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
        throw std::runtime_error("[ArtiH1::observation] Invalid IMU orientation.");
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
    append_rotation(heading_R_root, observation_);

    // body angular velocity in heading frame [6:9]
    append_vector(angular_velocity_heading, observation_);

    // joint rotations [9:81], policy order
    for (const char* name : kPolicyJointNames) {
        const auto joint_id = kinematics_->joint_id(name);
        append_rotation(kinematics_->joint_local_rotation(joint_id), observation_);
    }

    // joint velocities [81:93], policy order
    for (const char* name : kPolicyJointNames) {
        const auto joint_id = kinematics_->joint_id(name);
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

    // previous actions [99:111], policy order (raw ONNX "actions", already clamped to [-1, 1])
    for (const double action : previous_actions_) {
        observation_.push_back(action);
    }

    // gait clock [111:113]
    observation_.push_back(std::sin(2.0 * PI * gait_phase_));
    observation_.push_back(std::cos(2.0 * PI * gait_phase_));

    // read joy data
    joy_interface::joy_data_t joy_data{};
    joy_handler_->read(joy_data);

    // command [113:116] = command * command_scale, full stick = the training command range
    observation_.push_back(joy_data.stick_ly * command_max_[0] * command_scale_[0]);
    observation_.push_back(-joy_data.stick_lx * command_max_[1] * command_scale_[1]);
    observation_.push_back(-joy_data.stick_rx * command_max_[2] * command_scale_[2]);
}

void robot::ArtiH1::control() {
    if (!observation_.empty()) {
        // action_ = q_des (rad, policy order): default pose + scaled action + nominal gait at the next clock
        // phase, clamped to the training joint limits (hip_roll +-15 deg).
        controller_->update(observation_, action_);
        for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
            const auto joint_id = kinematics_->joint_id(kPolicyJointNames[i]);
            const auto motor_index = joint_id_to_motor_index_.at(joint_id);
            motor_command_[motor_index].position = action_[i];
            motor_command_[motor_index].kp = policy_kp_[motor_index];
            motor_command_[motor_index].kd = policy_kd_[motor_index];
        }

        const auto& actions = amp_controller_->policy_actions();
        if (actions.size() != NUM_JOINTS) {
            throw std::runtime_error("[ArtiH1::control] The ONNX model has no 'actions' output (needs pt_to_onnx.py export).");
        }
        std::copy(actions.begin(), actions.end(), previous_actions_.begin());

        // the env advances the clock once per control step (ArtiH1AmpEnv._pre_physics_step)
        gait_phase_ = std::fmod(gait_phase_ + control_dt_ * gait_phase_rate_, 1.0);
    }
    motor_manager_->write(motor_command_);
}
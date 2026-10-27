#include <algorithm>
#include <array>
#include <cmath>
#include <ctime>
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

#include "robot/robots/ARTI_D1.hpp"

namespace {

constexpr std::size_t NUM_JOINTS = 14;
// root tan-norm 6, angular velocity 3, joint tan-norm 84, joint velocity 14, toes 6, previous actions 14,
// gait phase sin/cos 2, commands 3 (ArtiD1AmpEnv._get_observations).
constexpr std::size_t OBS_JOINT_ROT = 9;
constexpr std::size_t OBS_JOINT_VEL = OBS_JOINT_ROT + 6 * NUM_JOINTS;   // 93
constexpr std::size_t OBS_TOES = OBS_JOINT_VEL + NUM_JOINTS;            // 107
constexpr std::size_t OBS_ACTIONS = OBS_TOES + 6;                       // 113
constexpr std::size_t OBS_PHASE = OBS_ACTIONS + NUM_JOINTS;             // 127
constexpr std::size_t OBS_COMMAND = OBS_PHASE + 2;                      // 129
constexpr std::size_t OBSERVATION_SIZE = OBS_COMMAND + 3;               // 132
constexpr auto STAND_UP_TIME = std::chrono::seconds(3);
constexpr auto STAND_UP_PERIOD = std::chrono::milliseconds(10);
constexpr double PI = 3.14159265358979323846;

// Isaac Lab articulation order of arti_d1 (robot.data.joint_names) = the policy observation / action order. The
// Dynamixel IDs 1-14 (robot.yaml motor index 0-13) follow the same order.
constexpr const char* kPolicyJointNames[NUM_JOINTS] = {
    "left_arm_m8",
    "left_hip_yaw",
    "right_arm_m8",
    "right_hip_yaw",
    "left_arm_m8crank_plate",
    "left_hip_roll",
    "right_arm_m8crank_plate",
    "right_hip_roll",
    "left_hip_pitch",
    "right_hip_pitch",
    "left_knee",
    "right_knee",
    "left_ankle",
    "right_ankle",
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
        throw std::runtime_error(std::string("[ArtiD1] robot.yaml policy.") + key + " needs " + std::to_string(M) + " values.");
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

robot::ArtiD1::ArtiD1(const std::string& config_file)
    : robot::Robot(config_file) {
    if (motors_.size() != NUM_JOINTS) {
        throw std::runtime_error("[ArtiD1::ArtiD1] Invalid number of joints.");
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

    if (std::FILE* model = std::fopen(model_file_.c_str(), "rb")) {
        std::fclose(model);
        auto amp_controller = std::make_unique<controller::AmpController>(model_file_);
        amp_controller_ = amp_controller.get();
        controller_ = std::move(amp_controller);
        controller_->initialize();
    } else {
        std::cerr << "[ArtiD1] No policy at " << model_file_ << " (export one with arti_d1 scripts/pt_to_onnx.py); "
                  << "only --observe works." << std::endl;
    }

    const YAML::Node policy = YAML::LoadFile(config_file)["policy"];
    if (!policy) throw std::runtime_error("[ArtiD1::ArtiD1] robot.yaml has no 'policy' section.");
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
    if (policy["initial_gait_phase"]) initial_gait_phase_ = policy["initial_gait_phase"].as<double>();
    if (policy["max_tilt"]) max_tilt_ = policy["max_tilt"].as<double>();
    if (policy["gait_rest_phases"]) gait_rest_phases_ = policy["gait_rest_phases"].as<std::vector<double>>();
    if (policy["stand_command_threshold"]) stand_command_threshold_ = policy["stand_command_threshold"].as<double>();
    if (policy["stick_deadzone"]) stick_deadzone_ = policy["stick_deadzone"].as<double>();

    const YAML::Node log_file = YAML::LoadFile(config_file)["log_file"];
    if (log_file && !log_file.as<std::string>().empty()) {
        // <log_file>_YYYYmmdd_HHMMSS.csv, so runs never overwrite each other
        char stamp[32];
        const std::time_t now = std::time(nullptr);
        std::strftime(stamp, sizeof(stamp), "_%Y%m%d_%H%M%S.csv", std::localtime(&now));
        const std::string path = log_file.as<std::string>() + stamp;
        log_file_ = std::fopen(path.c_str(), "w");
        if (log_file_ == nullptr) {
            std::cerr << "[ArtiD1] Cannot open log file " << path << ", logging disabled." << std::endl;
        } else {
            std::fprintf(log_file_, "t,phase");
            for (std::size_t i = 0; i < OBSERVATION_SIZE; ++i) std::fprintf(log_file_, ",obs_%zu", i);
            for (const char* name : kPolicyJointNames) std::fprintf(log_file_, ",q_des_%s", name);
            for (const char* name : kPolicyJointNames) std::fprintf(log_file_, ",action_%s", name);
            for (const char* name : kPolicyJointNames) {
                std::fprintf(log_file_, ",pos_%s,vel_%s,tau_%s,temp_%s", name, name, name, name);
            }
            std::fprintf(log_file_, ",imu_qw,imu_qx,imu_qy,imu_qz,imu_gx,imu_gy,imu_gz");
            // Dynamixel status packet error byte (bit 7: Hardware Error Status set, e.g. overheat / overload)
            for (const char* name : kPolicyJointNames) std::fprintf(log_file_, ",err_%s", name);
            std::fprintf(log_file_, "\n");
            std::cout << "[ArtiD1] Logging every control step to " << path << std::endl;
        }
    }

    observation_.clear();
    action_.clear();

    // Position P / D Gain 0 until initialize(): the servos stay limp.
    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        motor_command_[i].kp = 0.0;
        motor_command_[i].kd = 0.0;
    }
}

robot::ArtiD1::~ArtiD1() {
    if (log_file_ != nullptr) {
        std::fclose(log_file_);
        log_file_ = nullptr;
    }
}

void robot::ArtiD1::print_observation() const {
    if (observation_.size() != OBSERVATION_SIZE) return;
    const auto& o = observation_;
    constexpr double DEG = 180.0 / PI;
    // [0:6] = heading-frame root x axis (tan) and z axis (norm): body z in the heading frame gives roll / pitch
    const double roll = std::atan2(-o[4], o[5]) * DEG;   // + = right side down (x forward, y left, z up)
    const double pitch = std::atan2(o[3], o[5]) * DEG;   // + = nose down
    std::printf("\n[observe] root roll %+6.2f deg  pitch %+6.2f deg  | tan (%+.3f %+.3f %+.3f) norm (%+.3f %+.3f %+.3f)\n",
                roll, pitch, o[0], o[1], o[2], o[3], o[4], o[5]);
    std::printf("[observe] ang vel heading frame (rad/s)  x %+6.3f  y %+6.3f  z %+6.3f\n", o[6], o[7], o[8]);
    std::printf("[observe] joint               motor(deg)  |angle| from obs tan-norm(deg)  vel(rad/s)\n");
    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        // joint rotation R: tan = R e_x, norm = R e_z, R e_y = norm x tan; |angle| = acos((trace - 1) / 2) must equal
        // |motor angle| (a mismatch means the observation joint order / mapping is wrong)
        const double* r = &o[OBS_JOINT_ROT + 6 * i];
        const double y_y = r[5] * r[0] - r[3] * r[2];   // (norm x tan).y
        const double trace = r[0] + y_y + r[5];
        const double angle = std::acos(std::clamp((trace - 1.0) / 2.0, -1.0, 1.0));
        const auto motor_index = joint_id_to_motor_index_.at(kinematics_->joint_id(kPolicyJointNames[i]));
        const double motor = last_motor_status_[motor_index].position;
        std::printf("[observe]   %-18s %+8.2f      %7.2f                      %+7.3f\n",
                    kPolicyJointNames[i], motor * DEG, angle * DEG, o[OBS_JOINT_VEL + i]);
    }
    const double* toe = &o[OBS_TOES];
    std::printf("[observe] toes (heading frame, mm)  left (%+6.1f %+6.1f %+6.1f)  right (%+6.1f %+6.1f %+6.1f)\n",
                toe[0] * 1000, toe[1] * 1000, toe[2] * 1000, toe[3] * 1000, toe[4] * 1000, toe[5] * 1000);
    std::printf("[observe] phase sin %+.3f cos %+.3f | command (%+.4f %+.4f %+.4f) | imu q (%+.3f %+.3f %+.3f %+.3f)\n",
                o[OBS_PHASE], o[OBS_PHASE + 1], o[OBS_COMMAND], o[OBS_COMMAND + 1], o[OBS_COMMAND + 2],
                last_imu_orientation_[0], last_imu_orientation_[1],
                last_imu_orientation_[2], last_imu_orientation_[3]);
    std::fflush(stdout);
}

void robot::ArtiD1::log_observation() {
    if (log_file_ != nullptr) write_log_row();
}

void robot::ArtiD1::write_log_row() {
    // row = state the policy saw (observation_, motor / IMU read in observation()) and what it answered
    if (log_start_ == std::chrono::steady_clock::time_point{}) log_start_ = std::chrono::steady_clock::now();
    const double t = std::chrono::duration<double>(std::chrono::steady_clock::now() - log_start_).count();
    std::fprintf(log_file_, "%.4f,%.5f", t, gait_phase_);
    for (const double v : observation_) std::fprintf(log_file_, ",%.7g", v);   // full precision: replayed through ONNX
    // --observe mode has no policy output: q_des / action columns are nan there
    const bool policy_ran = action_.size() == NUM_JOINTS;
    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        if (policy_ran) std::fprintf(log_file_, ",%.7g", action_[i]); else std::fprintf(log_file_, ",nan");
    }
    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        if (policy_ran) std::fprintf(log_file_, ",%.7g", previous_actions_[i]); else std::fprintf(log_file_, ",nan");
    }
    for (const char* name : kPolicyJointNames) {
        const auto& s = last_motor_status_[joint_id_to_motor_index_.at(kinematics_->joint_id(name))];
        std::fprintf(log_file_, ",%.7g,%.7g,%.5g,%.0f", s.position, s.velocity, s.torque, s.temperature);
    }
    for (const double v : last_imu_orientation_) std::fprintf(log_file_, ",%.8g", v);
    for (const double v : last_imu_angular_velocity_) std::fprintf(log_file_, ",%.7g", v);
    for (const char* name : kPolicyJointNames) {
        std::fprintf(log_file_, ",%d", last_motor_status_[joint_id_to_motor_index_.at(kinematics_->joint_id(name))].error);
    }
    std::fprintf(log_file_, "\n");
}

void robot::ArtiD1::initialize() {
    // Dynamixel servos have absolute encoders (motor.yaml zero_offset): no homing run. Start limp, check that the bus
    // runs, then move to the policy's default pose with the policy gains, so the first q_des does not jump.
    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        motor_command_[i] = motor_interface::motor_command_t{};
    }
    motor_manager_->write(motor_command_);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    if (!motor_manager_->is_running()) {
        throw std::runtime_error("[ArtiD1::initialize] Motor communication stopped (see the [MotorManager::run] message above).");
    }

    motor_interface::motor_state_t motor_status[NUM_JOINTS]{};
    motor_manager_->read(motor_status);
    std::array<double, NUM_JOINTS> start{};
    std::array<double, NUM_JOINTS> goal{};
    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        const auto motor_index = joint_id_to_motor_index_.at(kinematics_->joint_id(kPolicyJointNames[i]));
        start[motor_index] = motor_status[motor_index].position;
        goal[motor_index] = default_pose_[i];
        std::cout << "[ArtiD1::initialize] " << kPolicyJointNames[i] << " (motor index " << static_cast<int>(motor_index)
                  << ") at " << start[motor_index] << " rad -> default pose " << goal[motor_index] << " rad" << std::endl;
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
        next_wakeup += STAND_UP_PERIOD;
        std::this_thread::sleep_until(next_wakeup);
    }
    gait_phase_ = initial_gait_phase_;
    previous_actions_.fill(0.0);
}

void robot::ArtiD1::observation() {
    observation_.clear();
    observation_.reserve(OBSERVATION_SIZE);

    motor_interface::motor_state_t motor_status[NUM_JOINTS]{};
    motor_manager_->read(motor_status);
    std::copy(motor_status, motor_status + NUM_JOINTS, last_motor_status_);

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
    for (std::size_t i = 0; i < 4; ++i) last_imu_orientation_[i] = imu_data.orientation[i];
    for (std::size_t i = 0; i < 3; ++i) last_imu_angular_velocity_[i] = imu_data.angular_velocity[i];

    Eigen::Quaterniond world_q_imu(
        imu_data.orientation[0],
        imu_data.orientation[1],
        imu_data.orientation[2],
        imu_data.orientation[3]
    );
    if (world_q_imu.norm() < 1.0e-8) {
        throw std::runtime_error("[ArtiD1::observation] Invalid IMU orientation.");
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

    // joint rotations [9:93], policy order
    for (const char* name : kPolicyJointNames) {
        const auto joint_id = kinematics_->joint_id(name);
        append_rotation(kinematics_->joint_local_rotation(joint_id), observation_);
    }

    // joint velocities [93:107], policy order
    for (const char* name : kPolicyJointNames) {
        const auto joint_id = kinematics_->joint_id(name);
        observation_.push_back(motor_status[joint_id_to_motor_index_.at(joint_id)].velocity);
    }

    // foot (sole centre) positions [107:113]
    for (std::size_t i = 0; i < foot_frame_ids_.size(); ++i) {
        const auto foot_frame_id = foot_frame_ids_[i];
        const auto foot_toe_offset = foot_toe_offsets_[i];

        const Eigen::Vector3d toe_position = 
            kinematics_->frame_position(foot_frame_id) +
            kinematics_->frame_rotation(foot_frame_id) * foot_toe_offset;

        append_vector(heading_R_root * toe_position, observation_);
    }

    // previous actions [113:127], policy order (raw ONNX "actions", already clamped to [-1, 1])
    for (const double action : previous_actions_) {
        observation_.push_back(action);
    }

    // gait clock [127:129]
    observation_.push_back(std::sin(2.0 * PI * gait_phase_));
    observation_.push_back(std::cos(2.0 * PI * gait_phase_));

    // read joy data
    joy_interface::joy_data_t joy_data{};
    joy_handler_->read(joy_data);

    // command [129:132] = command * command_scale, full stick = the training command range
    const auto dead = [this](double v) { return std::abs(v) < stick_deadzone_ ? 0.0 : v; };
    command_ = {dead(joy_data.stick_ly) * command_max_[0], -dead(joy_data.stick_lx) * command_max_[1],
                -dead(joy_data.stick_rx) * command_max_[2]};
    for (std::size_t i = 0; i < 3; ++i) observation_.push_back(command_[i] * command_scale_[i]);
}

void robot::ArtiD1::safe_stop(const std::string& reason) {
    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        motor_command_[i] = motor_interface::motor_command_t{};
        motor_command_[i].kd = policy_kd_[i];
    }
    if (motor_manager_->is_running()) {
        motor_manager_->write(motor_command_);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));   // let every master send the damping command
    }
    if (log_file_ != nullptr) std::fflush(log_file_);
    throw std::runtime_error("[ArtiD1::control] " + reason);
}

void robot::ArtiD1::control() {
    // A master thread that throws ([MotorManager::run] ...) stops every motor thread: the motor states freeze and no
    // command reaches the motors any more, so never keep running the policy on that data.
    if (!motor_manager_->is_running()) {
        safe_stop("Motor communication stopped (see the [MotorManager::run] message above).");
    }
    // observation_[5] = z of the pelvis z axis in the heading frame = cos(tilt)
    if (!observation_.empty() && observation_[5] < std::cos(max_tilt_)) {
        safe_stop("Pelvis tilt " + std::to_string(std::acos(std::clamp(observation_[5], -1.0, 1.0)) * 180.0 / PI) +
                  " deg > max_tilt: fallen, joints damped.");
    }
    if (controller_ == nullptr) {
        safe_stop("No policy (robot.yaml model_file missing): run with --observe or export agent.onnx first.");
    }
    if (!observation_.empty()) {
        // action_ = q_des (rad, policy order): default pose + scaled action + nominal gait at the next clock
        // phase, clamped to the training (URDF) joint limits.
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
            throw std::runtime_error("[ArtiD1::control] The ONNX model has no 'actions' output (needs pt_to_onnx.py export).");
        }
        std::copy(actions.begin(), actions.end(), previous_actions_.begin());

        if (log_file_ != nullptr) write_log_row();

        // the env advances the clock once per control step with the same standing rule (ArtiD1AmpEnv._advance_gait_phase)
        const double step = control_dt_ * gait_phase_rate_;
        double next = std::fmod(gait_phase_ + step, 1.0);
        const bool standing = std::max({std::abs(command_[0]), std::abs(command_[1]), std::abs(command_[2])})
                              <= stand_command_threshold_;
        if (standing) {
            for (const double rest : gait_rest_phases_) {
                const double ahead = std::fmod(rest - gait_phase_ + 1.0, 1.0);
                if (ahead < step || ahead > 1.0 - 1e-6) next = rest;
            }
        }
        gait_phase_ = next;
    }
    motor_manager_->write(motor_command_);
}
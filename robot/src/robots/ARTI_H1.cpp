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

#include "robot/robots/ARTI_H1.hpp"

namespace {

constexpr std::size_t NUM_JOINTS = 12;
// root tan-norm 6, angular velocity 3, joint tan-norm 72, joint velocity 12, toes 6, previous actions 12,
// gait phase sin/cos 2, commands 3 (ArtiH1AmpEnv._get_observations).
constexpr std::size_t OBSERVATION_SIZE = 116;
// ArtiH1VelocityEnv._get_observations: gyro 3, gravity 3, joint pos - default 12, joint vel 12, previous actions 12,
// command 3 (robot.yaml policy.observation: velocity).
constexpr std::size_t VELOCITY_OBSERVATION_SIZE = 45;
// obs_action_history: gyro 3, gravity 3, joint pos - default 12, joint vel 12, actions a(t), a(t-1), a(t-2) 36,
// joint target - default 12, command 3 (robot.yaml policy.observation: velocity_history).
constexpr std::size_t VELOCITY_HISTORY_OBSERVATION_SIZE = 81;
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

robot::ArtiH1::ArtiH1(const std::string& config_file, const std::string& model_file)
    : robot::Robot(config_file) {
    const bool model_override = !model_file.empty() && model_file != model_file_;
    if (!model_file.empty()) model_file_ = model_file;
    std::cout << "[ArtiH1] policy model: " << model_file_ << std::endl;
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
    if (policy["observation"]) {
        const auto type = policy["observation"].as<std::string>();
        if (type == "velocity") {
            velocity_obs_ = true;
        } else if (type == "velocity_history") {
            velocity_obs_ = true;
            action_history_obs_ = true;
        } else if (type != "amp") {
            throw std::runtime_error(
                "[ArtiH1::ArtiH1] robot.yaml policy.observation must be 'amp', 'velocity' or 'velocity_history'.");
        }
    }
    observation_size_ = action_history_obs_ ? VELOCITY_HISTORY_OBSERVATION_SIZE
                        : velocity_obs_     ? VELOCITY_OBSERVATION_SIZE
                                            : OBSERVATION_SIZE;
    std::cout << "[ArtiH1] observation: "
              << (action_history_obs_ ? "velocity_history (81)" : velocity_obs_ ? "velocity (45)" : "amp (116)")
              << std::endl;
    if (policy["target_rate_limit"]) target_rate_limit_ = policy["target_rate_limit"].as<double>();
    if (policy["target_lowpass_alpha"]) target_lowpass_alpha_ = policy["target_lowpass_alpha"].as<double>();
    if (!(target_lowpass_alpha_ > 0.0 && target_lowpass_alpha_ <= 1.0)) {
        throw std::runtime_error("[ArtiH1::ArtiH1] robot.yaml policy.target_lowpass_alpha must be in (0, 1].");
    }
    std::cout << "[ArtiH1] joint target low-pass alpha: " << target_lowpass_alpha_
              << (target_lowpass_alpha_ < 1.0 ? "" : " (off)") << std::endl;
    std::cout << "[ArtiH1] joint target rate limit: "
              << (target_rate_limit_ > 0.0 ? std::to_string(target_rate_limit_) + " rad/s" : std::string("off"))
              << std::endl;
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
    if (policy["imu_still_time"]) imu_still_time_ = policy["imu_still_time"].as<double>();
    if (policy["imu_converge_tolerance"]) imu_converge_tolerance_ = policy["imu_converge_tolerance"].as<double>();
    if (policy["imu_converge_timeout"]) imu_converge_timeout_ = policy["imu_converge_timeout"].as<double>();
    if (policy["imu_attitude_gain"]) imu_attitude_gain_ = policy["imu_attitude_gain"].as<double>();
    if (policy["imu_stale_time"]) imu_stale_time_ = policy["imu_stale_time"].as<double>();
    if (policy["fault_settle_time"]) fault_settle_time_ = policy["fault_settle_time"].as<double>();
    if (policy["joy_loss_hold"]) joy_loss_hold_ = policy["joy_loss_hold"].as<bool>();
    if (policy["gait_rest_phases"]) gait_rest_phases_ = policy["gait_rest_phases"].as<std::vector<double>>();
    if (policy["stand_command_threshold"]) stand_command_threshold_ = policy["stand_command_threshold"].as<double>();
    if (policy["stick_deadzone"]) stick_deadzone_ = policy["stick_deadzone"].as<double>();

    const YAML::Node log_file = YAML::LoadFile(config_file)["log_file"];
    if (log_file && !log_file.as<std::string>().empty()) {
        // <log_file>_YYYYmmdd_HHMMSS.csv, so runs never overwrite each other; a model picked on the command line
        // (--ppo / --model) adds its file name: <log_file>_<model>_YYYYmmdd_HHMMSS.csv
        std::string prefix = log_file.as<std::string>();
        if (model_override) {
            std::string stem = model_file_.substr(model_file_.find_last_of('/') + 1);
            if (stem.size() > 5 && stem.compare(stem.size() - 5, 5, ".onnx") == 0) stem.resize(stem.size() - 5);
            prefix += "_" + stem;
        }
        char stamp[32];
        const std::time_t now = std::time(nullptr);
        std::strftime(stamp, sizeof(stamp), "_%Y%m%d_%H%M%S.csv", std::localtime(&now));
        const std::string path = prefix + stamp;
        log_file_ = std::fopen(path.c_str(), "w");
        if (log_file_ == nullptr) {
            std::cerr << "[ArtiH1] Cannot open log file " << path << ", logging disabled." << std::endl;
        } else {
            std::fprintf(log_file_, "t,phase");
            for (std::size_t i = 0; i < observation_size_; ++i) std::fprintf(log_file_, ",obs_%zu", i);
            for (const char* name : kPolicyJointNames) std::fprintf(log_file_, ",q_des_%s", name);
            for (const char* name : kPolicyJointNames) std::fprintf(log_file_, ",action_%s", name);
            for (const char* name : kPolicyJointNames) {
                std::fprintf(log_file_, ",pos_%s,vel_%s,tau_%s,temp_%s", name, name, name, name);
            }
            std::fprintf(log_file_, ",imu_qw,imu_qx,imu_qy,imu_qz,imu_gx,imu_gy,imu_gz,imu_ax,imu_ay,imu_az,imu_temp,att_qw,att_qx,att_qy,att_qz");
            // motor error code (Unitree: 1 overheat, 2 overcurrent, 3 overvoltage, 4 encoder fault)
            for (const char* name : kPolicyJointNames) std::fprintf(log_file_, ",err_%s", name);
            // fault bits: 1 IMU stopped, 2 motor link, 4 joystick lost, 8 holding (policy stopped, pose held)
            std::fprintf(log_file_, ",fault");
            std::fprintf(log_file_, "\n");
            std::cout << "[ArtiH1] Logging every control step to " << path << std::endl;
        }
    }

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

robot::ArtiH1::~ArtiH1() {
    if (log_file_ != nullptr) {
        std::fclose(log_file_);
        log_file_ = nullptr;
    }
}

void robot::ArtiH1::print_observation() const {
    const auto& o = observation_;
    constexpr double DEG = 180.0 / PI;
    if (velocity_obs_) {
        if (o.size() != observation_size_) return;
        const std::size_t c = o.size() - 3;   // command index
        // [3:6] = gravity direction in the pelvis frame: (0, 0, -1) when level
        const double roll = std::atan2(o[4], -o[5]) * DEG;    // + = right side down
        const double pitch = std::atan2(-o[3], -o[5]) * DEG;  // + = nose down
        std::printf("\n[observe] pelvis roll %+6.2f deg  pitch %+6.2f deg  | gravity (%+.3f %+.3f %+.3f)  gyro (%+.3f %+.3f %+.3f) rad/s\n",
                    roll, pitch, o[3], o[4], o[5], o[0], o[1], o[2]);
        std::printf("[observe] joint               pos-default(deg)  vel(rad/s)  prev action\n");
        for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
            std::printf("[observe]   %-18s %+8.2f          %+7.3f     %+.3f\n", kPolicyJointNames[i], o[6 + i] * DEG,
                        o[18 + i], o[30 + i]);
        }
        std::printf("[observe] command (%+.4f %+.4f %+.4f)\n", o[c], o[c + 1], o[c + 2]);
        std::fflush(stdout);
        return;
    }
    if (o.size() != OBSERVATION_SIZE) return;
    // [0:6] = heading-frame root x axis (tan) and z axis (norm): body z in the heading frame gives roll / pitch
    const double roll = std::atan2(-o[4], o[5]) * DEG;   // + = right side down (x forward, y left, z up)
    const double pitch = std::atan2(o[3], o[5]) * DEG;   // + = nose down
    std::printf("\n[observe] root roll %+6.2f deg  pitch %+6.2f deg  | tan (%+.3f %+.3f %+.3f) norm (%+.3f %+.3f %+.3f)\n",
                roll, pitch, o[0], o[1], o[2], o[3], o[4], o[5]);
    std::printf("[observe] ang vel heading frame (rad/s)  x %+6.3f  y %+6.3f  z %+6.3f\n", o[6], o[7], o[8]);
    std::printf("[observe] joint               motor(deg)  |angle| from obs tan-norm(deg)  tan (x y z)              norm (x y z)              vel(rad/s)  prev action\n");
    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        // joint rotation R: tan = R e_x, norm = R e_z, R e_y = norm x tan; |angle| = acos((trace - 1) / 2) must equal
        // |motor angle| (a mismatch means the observation joint order / mapping is wrong)
        const double* r = &o[9 + 6 * i];
        const double y_y = r[5] * r[0] - r[3] * r[2];   // (norm x tan).y
        const double trace = r[0] + y_y + r[5];
        const double angle = std::acos(std::clamp((trace - 1.0) / 2.0, -1.0, 1.0));
        const auto motor_index = joint_id_to_motor_index_.at(kinematics_->joint_id(kPolicyJointNames[i]));
        const double motor = last_motor_status_[motor_index].position;
        std::printf("[observe]   %-18s %+8.2f      %7.2f                      (%+.3f %+.3f %+.3f)  (%+.3f %+.3f %+.3f)  %+7.3f     %+.3f\n",
                    kPolicyJointNames[i], motor * DEG, angle * DEG, r[0], r[1], r[2], r[3], r[4], r[5], o[81 + i],
                    o[99 + i]);
    }
    std::printf("[observe] toes (heading frame, mm)  left (%+6.1f %+6.1f %+6.1f)  right (%+6.1f %+6.1f %+6.1f)\n",
                o[93] * 1000, o[94] * 1000, o[95] * 1000, o[96] * 1000, o[97] * 1000, o[98] * 1000);
    std::printf("[observe] phase sin %+.3f cos %+.3f | command (%+.4f %+.4f %+.4f) | imu q (%+.3f %+.3f %+.3f %+.3f)\n",
                o[111], o[112], o[113], o[114], o[115], last_imu_orientation_[0], last_imu_orientation_[1],
                last_imu_orientation_[2], last_imu_orientation_[3]);
    std::fflush(stdout);
}

void robot::ArtiH1::set_limp_right() {
    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        if (std::string(kPolicyJointNames[i]).rfind("right_", 0) != 0) continue;
        const auto motor_index = joint_id_to_motor_index_.at(kinematics_->joint_id(kPolicyJointNames[i]));
        motor_command_[motor_index].kp = 0.0;
        motor_command_[motor_index].kd = policy_kd_[motor_index];
    }
    motor_manager_->write(motor_command_);
    limp_right_ = true;
    limp_stats_init_ = false;
    std::cout << "[ArtiH1] --limp-right: right-leg motors kp 0 (damping only). Hold the pelvis level and press both soles"
                 " flat on the floor, the right foot as the mirror image of the left." << std::endl;
}

void robot::ArtiH1::update_limp_stats() {
    if (!limp_right_) return;
    constexpr double kAlpha = 1.0 / 120.0;   // ~2 s at 60 Hz
    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        const auto motor_index = joint_id_to_motor_index_.at(kinematics_->joint_id(kPolicyJointNames[i]));
        const double q = last_motor_status_[motor_index].position;
        limp_mean_pos_[i] = limp_stats_init_ ? limp_mean_pos_[i] + kAlpha * (q - limp_mean_pos_[i]) : q;
    }
    limp_stats_init_ = true;
}

void robot::ArtiH1::print_limp_report() const {
    if (!limp_right_ || !limp_stats_init_) return;
    constexpr double DEG = 180.0 / PI;
    double roll = 0.0, pitch = 0.0;
    if (observation_.size() >= 6) {
        roll = std::atan2(observation_[4], -observation_[5]) * DEG;
        pitch = std::atan2(-observation_[3], -observation_[5]) * DEG;
    }
    std::printf("\n[limp] pelvis roll %+5.2f pitch %+5.2f deg (keep both near 0)   ~2 s averages:\n", roll, pitch);
    std::printf("[limp] joint        left(deg)  right(deg)  right-left(deg)  right motor id  homing_offset now -> suggested\n");
    // policy order is (left, right) pairs: hip_yaw, hip_roll, hip_pitch, knee, ankle_pitch, ankle_roll
    for (std::size_t i = 0; i + 1 < NUM_JOINTS; i += 2) {
        const double l = limp_mean_pos_[i], r = limp_mean_pos_[i + 1];
        const auto motor_index = joint_id_to_motor_index_.at(kinematics_->joint_id(kPolicyJointNames[i + 1]));
        const double offset = motor_manager_->homing(motor_index).offset;
        // after homing the reading is raw - (raw_at_stopper - homing_offset): a reading too high by e needs offset - e
        std::printf("[limp]   %-12s %+8.2f   %+8.2f     %+8.2f        %2d          %+.4f -> %+.4f\n",
                    kPolicyJointNames[i + 1] + 6, l * DEG, r * DEG, (r - l) * DEG, static_cast<int>(motor_index) + 1,
                    offset, offset - (r - l));
    }
    std::fflush(stdout);
}

void robot::ArtiH1::log_observation() {
    if (log_file_ != nullptr) write_log_row();
}

void robot::ArtiH1::write_log_row() {
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
    for (const double v : last_imu_linear_acceleration_) std::fprintf(log_file_, ",%.7g", v);
    std::fprintf(log_file_, ",%.2f", last_imu_temperature_);
    for (const double v : last_attitude_) std::fprintf(log_file_, ",%.8g", v);
    for (const char* name : kPolicyJointNames) {
        std::fprintf(log_file_, ",%d", last_motor_status_[joint_id_to_motor_index_.at(kinematics_->joint_id(name))].error);
    }
    std::fprintf(log_file_, ",%d", fault_);
    std::fprintf(log_file_, "\n");
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

    // 10-28 15:19 observe log (joints still to 0.01 deg): the MTi-3 roll / pitch started 3-4 deg away from the
    // accelerometer tilt and took ~80 s to converge onto it, and the gyro read a constant bias of up to 1 deg/s
    // (training randomizes +-0.57 deg/s). Hold the default pose still: average the gyro bias (subtracted in
    // observation()), then wait until the IMU attitude agrees with the accelerometer before the policy starts.
    if (imu_still_time_ > 0.0) {
        auto& imu = sensor_manager_->sensor<sensor_interface::Imu>(imu_sensor_index_);
        imu.no_rotation_update(static_cast<uint16_t>(std::lround(imu_still_time_)));
        std::cout << "[ArtiH1::initialize] IMU calibration: keep the robot still ..." << std::endl;

        const auto tilt = [](const sensor_interface::imu_data_t& d, double& qr, double& qp, double& ar, double& ap) {
            const Eigen::Quaterniond q(d.orientation[0], d.orientation[1], d.orientation[2], d.orientation[3]);
            const Eigen::Vector3d g = q.normalized().toRotationMatrix().transpose() * Eigen::Vector3d::UnitZ();
            qr = std::atan2(g.y(), g.z()) * 180.0 / PI;
            qp = -std::asin(std::clamp(g.x(), -1.0, 1.0)) * 180.0 / PI;
            const double* a = d.linear_acceleration;
            ar = std::atan2(a[1], a[2]) * 180.0 / PI;
            ap = std::atan2(-a[0], std::hypot(a[1], a[2])) * 180.0 / PI;
        };

        constexpr auto kPeriod = std::chrono::milliseconds(10);
        const auto begin = std::chrono::steady_clock::now();
        auto next = begin;
        std::array<double, 3> gyro_sum{};
        std::size_t gyro_count = 0;
        double agree_since = -1.0;
        double last_print = 0.0;
        while (true) {
            next += kPeriod;
            std::this_thread::sleep_until(next);
            const double t = std::chrono::duration<double>(std::chrono::steady_clock::now() - begin).count();
            sensor_interface::imu_data_t d{};
            imu.read(d);
            if (t < imu_still_time_) {
                for (std::size_t i = 0; i < 3; ++i) gyro_sum[i] += d.angular_velocity[i];
                ++gyro_count;
                continue;
            }
            double qr, qp, ar, ap;
            tilt(d, qr, qp, ar, ap);
            const double err = std::max(std::abs(qr - ar), std::abs(qp - ap));
            if (err <= imu_converge_tolerance_) {
                if (agree_since < 0.0) agree_since = t;
            } else {
                agree_since = -1.0;
            }
            if (t - last_print >= 5.0) {
                last_print = t;
                std::printf("[ArtiH1::initialize] %5.1f s  IMU roll %+.2f pitch %+.2f | accel roll %+.2f pitch %+.2f deg"
                            "  (temp %.1f C)\n", t, qr, qp, ar, ap, d.temperature);
            }
            if (agree_since >= 0.0 && t - agree_since >= 1.0) break;
            if (t >= imu_converge_timeout_) {
                std::cerr << "[ArtiH1::initialize] WARNING: IMU attitude still " << err
                          << " deg away from the accelerometer after " << t << " s; starting anyway." << std::endl;
                break;
            }
        }
        for (std::size_t i = 0; i < 3; ++i) imu_gyro_bias_[i] = gyro_count > 0 ? gyro_sum[i] / gyro_count : 0.0;
        std::printf("[ArtiH1::initialize] IMU ready: gyro bias (%+.3f %+.3f %+.3f) deg/s removed from the observation\n",
                    imu_gyro_bias_[0] * 180.0 / PI, imu_gyro_bias_[1] * 180.0 / PI, imu_gyro_bias_[2] * 180.0 / PI);
    }
    attitude_initialized_ = false;
    imu_sample_time_ = std::chrono::steady_clock::now();
    gait_phase_ = initial_gait_phase_;
    previous_actions_.fill(0.0);
    previous_actions2_.fill(0.0);
    previous_actions3_.fill(0.0);
    q_target_ = default_pose_;   // the env resets its joint target to the default pose
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
        // homing must not run on stale states (a frozen reading looks like a stall and sets a wrong zero)
        if (!motor_manager_->is_running() || motor_manager_->comm_fault()) {
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
        // homing must not run on stale states (a frozen reading looks like a stall and sets a wrong zero)
        if (!motor_manager_->is_running() || motor_manager_->comm_fault()) {
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
        // homing must not run on stale states (a frozen reading looks like a stall and sets a wrong zero)
        if (!motor_manager_->is_running() || motor_manager_->comm_fault()) {
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

// 10-28 16:05 walking log: in every still window after walking the MTi-3 roll / pitch was 1.5-3.4 deg away from the
// accelerometer tilt (robot standing still, so the accelerometer is the truth). Own attitude instead: integrate the
// bias-corrected gyro and pull roll / pitch toward the accelerometer with imu_attitude_gain (1/s), trusting the
// accelerometer less when |a| is far from g (impacts). Offline on that log (gain 0.5-1): still-window error <= 0.4 deg.
// Starts from the MTi orientation (checked against the accelerometer in initialize()); yaw only follows the gyro,
// which is fine because the policy only sees the heading frame.
Eigen::Quaterniond robot::ArtiH1::update_attitude(const sensor_interface::imu_data_t& imu_data,
                                                  const Eigen::Quaterniond& mti_q) {
    const auto now = std::chrono::steady_clock::now();
    if (!attitude_initialized_) {
        attitude_q_ = mti_q;
        attitude_time_ = now;
        attitude_initialized_ = true;
        return attitude_q_;
    }
    const double dt = std::clamp(std::chrono::duration<double>(now - attitude_time_).count(), 0.0, 0.1);
    attitude_time_ = now;

    Eigen::Vector3d omega(imu_data.angular_velocity[0] - imu_gyro_bias_[0],
                          imu_data.angular_velocity[1] - imu_gyro_bias_[1],
                          imu_data.angular_velocity[2] - imu_gyro_bias_[2]);
    const Eigen::Vector3d accel(imu_data.linear_acceleration[0], imu_data.linear_acceleration[1],
                                imu_data.linear_acceleration[2]);
    const double accel_norm = accel.norm();
    if (accel_norm > 1.0) {
        // up direction in the sensor frame: estimated (from the attitude) vs measured (accelerometer)
        const Eigen::Vector3d up = attitude_q_.toRotationMatrix().transpose() * Eigen::Vector3d::UnitZ();
        const double trust = std::exp(-std::pow((accel_norm - 9.81) / 0.5, 2));
        omega += imu_attitude_gain_ * trust * (accel / accel_norm).cross(up);
    }
    const Eigen::Quaterniond omega_q(0.0, omega.x(), omega.y(), omega.z());
    attitude_q_.coeffs() += 0.5 * dt * (attitude_q_ * omega_q).coeffs();
    attitude_q_.normalize();
    return attitude_q_;
}

void robot::ArtiH1::observation() {
    observation_.clear();
    observation_.reserve(observation_size_);

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
    for (std::size_t i = 0; i < 3; ++i) last_imu_linear_acceleration_[i] = imu_data.linear_acceleration[i];
    last_imu_temperature_ = imu_data.temperature;

    // 10-29 09:44 walking log: every IMU field froze at 71.5 s (no more MTi packets) and the policy walked on the
    // stale attitude for 90 s. Real samples never repeat exactly at 60 Hz, so a sample that stops changing = IMU lost.
    const std::array<double, 6> imu_sample{imu_data.angular_velocity[0], imu_data.angular_velocity[1],
                                           imu_data.angular_velocity[2], imu_data.linear_acceleration[0],
                                           imu_data.linear_acceleration[1], imu_data.linear_acceleration[2]};
    const auto now = std::chrono::steady_clock::now();
    if (imu_sample != imu_last_sample_) {
        imu_last_sample_ = imu_sample;
        imu_sample_time_ = now;
        imu_stale_ = false;
    } else if (imu_stale_time_ > 0.0 &&
               std::chrono::duration<double>(now - imu_sample_time_).count() > imu_stale_time_) {
        imu_stale_ = true;
    }

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
    if (imu_attitude_gain_ > 0.0) world_q_imu = update_attitude(imu_data, world_q_imu);
    last_attitude_ = {world_q_imu.w(), world_q_imu.x(), world_q_imu.y(), world_q_imu.z()};

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

    const Eigen::Matrix3d true_heading_R_root = heading_R_world * world_R_root;
    true_tilt_cos_ = true_heading_R_root(2, 2);
    if (blind_imu_ && !blind_latched_) {
        blind_heading_R_root_ = true_heading_R_root;
        blind_latched_ = true;
    }
    const Eigen::Matrix3d& heading_R_root = blind_imu_ ? blind_heading_R_root_ : true_heading_R_root;

    const Eigen::Vector3d angular_velocity_imu(
        imu_data.angular_velocity[0] - imu_gyro_bias_[0],
        imu_data.angular_velocity[1] - imu_gyro_bias_[1],
        imu_data.angular_velocity[2] - imu_gyro_bias_[2]
    );

    const Eigen::Vector3d angular_velocity_world = world_R_imu * angular_velocity_imu;

    const Eigen::Vector3d angular_velocity_heading =
        blind_imu_ ? Eigen::Vector3d(Eigen::Vector3d::Zero()) : Eigen::Vector3d(heading_R_world * angular_velocity_world);

    if (velocity_obs_) {
        // ArtiH1VelocityEnv: the sim IMU sits on the pelvis with identity orientation, so its gyro and gravity are in
        // the pelvis frame (root_R_imu maps the real sensor axes onto it).
        const Eigen::Vector3d gyro_root =
            blind_imu_ ? Eigen::Vector3d(Eigen::Vector3d::Zero()) : Eigen::Vector3d(root_R_imu * angular_velocity_imu);
        // gravity direction in the pelvis frame; heading_R_root has the same tilt as world_R_root (heading is about z)
        const Eigen::Vector3d gravity_root = heading_R_root.transpose() * Eigen::Vector3d(0.0, 0.0, -1.0);
        append_vector(gyro_root, observation_);                       // [0:3]
        append_vector(gravity_root, observation_);                    // [3:6]
        for (std::size_t i = 0; i < NUM_JOINTS; ++i) {                // [6:18] joint pos - default pose, policy order
            const auto joint_id = kinematics_->joint_id(kPolicyJointNames[i]);
            observation_.push_back(motor_status[joint_id_to_motor_index_.at(joint_id)].position - default_pose_[i]);
        }
        for (const char* name : kPolicyJointNames) {                  // [18:30] joint velocities
            const auto joint_id = kinematics_->joint_id(name);
            observation_.push_back(motor_status[joint_id_to_motor_index_.at(joint_id)].velocity);
        }
        for (const double action : previous_actions_) observation_.push_back(action);   // [30:42] a(t)
        if (action_history_obs_) {
            for (const double action : previous_actions2_) observation_.push_back(action);   // [42:54] a(t-1)
            for (const double action : previous_actions3_) observation_.push_back(action);   // [54:66] a(t-2)
            for (std::size_t i = 0; i < NUM_JOINTS; ++i) {                                // [66:78] target - default
                observation_.push_back(q_target_[i] - default_pose_[i]);
            }
        }
        joy_interface::joy_data_t joy_data{};
        joy_handler_->read(joy_data);
        const auto dead = [this](double v) { return std::abs(v) < stick_deadzone_ ? 0.0 : v; };
        command_ = {dead(joy_data.stick_ly) * command_max_[0], -dead(joy_data.stick_lx) * command_max_[1],
                    -dead(joy_data.stick_rx) * command_max_[2]};
        for (std::size_t i = 0; i < 3; ++i) observation_.push_back(command_[i] * command_scale_[i]);   // last 3
        return;
    }

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
    const auto dead = [this](double v) { return std::abs(v) < stick_deadzone_ ? 0.0 : v; };
    command_ = {dead(joy_data.stick_ly) * command_max_[0], -dead(joy_data.stick_lx) * command_max_[1],
                -dead(joy_data.stick_rx) * command_max_[2]};
    for (std::size_t i = 0; i < 3; ++i) observation_.push_back(command_[i] * command_scale_[i]);
}

void robot::ArtiH1::safe_stop(const std::string& reason) {
    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        motor_command_[i] = motor_interface::motor_command_t{};
        motor_command_[i].kd = policy_kd_[i];
    }
    if (motor_manager_->is_running()) {
        motor_manager_->write(motor_command_);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));   // let every master send the damping command
    }
    if (log_file_ != nullptr) std::fflush(log_file_);
    throw std::runtime_error("[ArtiH1::control] " + reason);
}

int robot::ArtiH1::fault_status() const {
    int status = 0;
    if (imu_stale_) status |= FAULT_IMU;
    if (motor_manager_->comm_fault()) status |= FAULT_MOTOR_COMM;
    if (!joy_handler_->connected()) status |= FAULT_JOY;
    return status;
}

void robot::ArtiH1::report_faults(int status) {
    const int previous = fault_ & ~FAULT_HOLDING;
    if (status == previous) return;
    const double t = log_start_ == std::chrono::steady_clock::time_point{} ? 0.0
                     : std::chrono::duration<double>(std::chrono::steady_clock::now() - log_start_).count();
    const auto say = [&](int bit, const char* what) {
        if ((status & bit) && !(previous & bit)) std::cerr << "[ArtiH1] t=" << t << " s FAULT: " << what << std::endl;
        if (!(status & bit) && (previous & bit)) std::cerr << "[ArtiH1] t=" << t << " s cleared: " << what
                                                           << (holding_ ? " (still holding, restart to walk again)" : "")
                                                           << std::endl;
    };
    say(FAULT_IMU, "IMU samples stopped changing (MTi-3 disconnected?)");
    say(FAULT_MOTOR_COMM, "motor link (see the [MotorManager] / [UnitreeMaster] messages)");
    say(FAULT_JOY, joy_loss_hold_ ? "joystick lost" : "joystick lost (zero command: the policy stands until the pad is back)");
    if (log_file_ != nullptr) std::fflush(log_file_);
}

void robot::ArtiH1::hold_control() {
    const double t = std::chrono::duration<double>(std::chrono::steady_clock::now() - hold_start_).count();
    const double alpha = fault_settle_time_ > 0.0 ? std::min(1.0, t / fault_settle_time_) : 0.0;
    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        q_target_[i] = hold_from_[i] + alpha * (default_pose_[i] - hold_from_[i]);
        const auto motor_index = joint_id_to_motor_index_.at(kinematics_->joint_id(kPolicyJointNames[i]));
        motor_command_[motor_index].position = q_target_[i];
        motor_command_[motor_index].kp = policy_kp_[motor_index];
        motor_command_[motor_index].kd = policy_kd_[motor_index];
    }
    motor_manager_->write(motor_command_);
    if (log_file_ != nullptr) write_log_row();
}

void robot::ArtiH1::control() {
    // real cos(tilt) of the pelvis (= observation_[5] unless --blind-imu); a fallen robot is only damped
    if (!observation_.empty() && true_tilt_cos_ < std::cos(max_tilt_)) {
        safe_stop("Pelvis tilt " + std::to_string(std::acos(std::clamp(true_tilt_cos_, -1.0, 1.0)) * 180.0 / PI) +
                  " deg > max_tilt: fallen, joints damped.");
    }
    // IMU / motor link / joystick fault: stop the policy and hold the pose, keep running and logging
    const int status = fault_status();
    report_faults(status);
    const int hold_mask = FAULT_IMU | FAULT_MOTOR_COMM | (joy_loss_hold_ ? FAULT_JOY : 0);
    if ((status & hold_mask) != 0 && !holding_) {
        holding_ = true;
        hold_from_ = q_target_;
        hold_start_ = std::chrono::steady_clock::now();
        std::cerr << "[ArtiH1] policy stopped, holding the pose (settle to the default stance over "
                  << fault_settle_time_ << " s). Ctrl+C to exit." << std::endl;
    }
    fault_ = status | (holding_ ? FAULT_HOLDING : 0);
    if (holding_) {
        hold_control();
        return;
    }
    if (!observation_.empty()) {
        // action_ = q_des (rad, policy order): default pose + scaled action + nominal gait at the next clock
        // phase, clamped to the training joint limits (hip_roll +-15 deg).
        controller_->update(observation_, action_);
        // ArtiH1VelocityEnv._pre_physics_step: low-pass and slew limit against the previous target, then remember it.
        for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
            if (target_lowpass_alpha_ < 1.0) action_[i] = q_target_[i] + target_lowpass_alpha_ * (action_[i] - q_target_[i]);
            if (target_rate_limit_ > 0.0) {
                const double dq = target_rate_limit_ * control_dt_;
                action_[i] = std::clamp(action_[i], q_target_[i] - dq, q_target_[i] + dq);
            }
            q_target_[i] = action_[i];
        }
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
        previous_actions3_ = previous_actions2_;
        previous_actions2_ = previous_actions_;
        std::copy(actions.begin(), actions.end(), previous_actions_.begin());

        if (log_file_ != nullptr) write_log_row();

        // the env advances the clock once per control step (ArtiH1AmpEnv._pre_physics_step)
        // the env advances the clock the same way (ArtiH1AmpEnv._advance_gait_phase)
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
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
#include "robot/robots/ARTI_D1_shoulder.hpp"

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
constexpr double STAND_UP_SPEED = 0.3;   // rad/s, largest joint speed while moving to home / the default pose
constexpr double PI = 3.14159265358979323846;

// Isaac Lab articulation order of arti_d1 (robot.data.joint_names) = the policy observation / action order. The
// Dynamixel IDs 1-14 (robot.yaml motor index 0-13) follow the same order; the {side}_arm_m8crank_plate IDs are the
// real m7 servos of the shoulder 5-bar (ARTI_D1_shoulder.hpp).
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

// DualSense button by robot.yaml name (stage2.buttons). dpad_y > 0 = up (dualsense.cpp flips axis 7).
bool joy_pressed(const joy_interface::joy_data_t& j, const std::string& name) {
    if (name == "square") return j.square;
    if (name == "circle") return j.circle;
    if (name == "triangle") return j.triangle;
    if (name == "cross") return j.cross;
    if (name == "l1") return j.l1;
    if (name == "r1") return j.r1;
    if (name == "l2") return j.l2;
    if (name == "r2") return j.r2;
    if (name == "select") return j.select;
    if (name == "start") return j.start;
    if (name == "center") return j.center;
    if (name == "dpad_up") return j.dpad_y > 0.5f;
    if (name == "dpad_down") return j.dpad_y < -0.5f;
    if (name == "dpad_left") return j.dpad_x < -0.5f;
    if (name == "dpad_right") return j.dpad_x > 0.5f;
    if (name == "none" || name.empty()) return false;
    throw std::runtime_error("[ArtiD1] robot.yaml stage2.buttons: unknown button " + name);
}

constexpr const char* kStage2Actions[] = {
    "height_up",      // hold: height target rises at height_step_rate
    "height_down",    // hold: height target falls
    "height_low",     // press: target 0 (frog default pose)
    "height_high",    // press: target 1 (legs nearly straight)
    "height_mid",     // press: target 0.5
    "step_toggle",    // press: step in place on / off
    "reset_toggle",   // press: stop the policy and return to the default pose / press again: restart the policy
};

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

    const auto motor_index = [this](const char* name) { return joint_id_to_motor_index_.at(kinematics_->joint_id(name)); };
    shoulder_m8_motor_index_ = {motor_index("left_arm_m8"), motor_index("right_arm_m8")};
    shoulder_m7_motor_index_ = {motor_index("left_arm_m8crank_plate"), motor_index("right_arm_m8crank_plate")};

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
    if (policy["home_pose"]) {
        home_pose_ = read_array<NUM_JOINTS>(policy, "home_pose");
        has_home_pose_ = true;
    }
    // policy.fixed_joints {name: rad}: these joints ignore the ONNX q_des and hold the given angle (USER 2026-11-03:
    // arms fixed at 0). Also the start pose uses them, so nothing jumps when the policy starts.
    if (const YAML::Node fixed = policy["fixed_joints"]) {
        for (const auto& item : fixed) {
            const std::string name = item.first.as<std::string>();
            bool found = false;
            for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
                if (name == kPolicyJointNames[i]) {
                    fixed_joint_[i] = true;
                    fixed_value_[i] = item.second.as<double>();
                    found = true;
                }
            }
            if (!found) throw std::runtime_error("[ArtiD1] robot.yaml policy.fixed_joints: unknown joint " + name);
            std::cout << "[ArtiD1] " << name << " fixed at " << item.second.as<double>() << " rad (ONNX output ignored)" << std::endl;
        }
    }
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
    if (policy["action_filter_alpha"]) {
        action_filter_alpha_ = std::clamp(policy["action_filter_alpha"].as<double>(), 0.05, 1.0);
        std::cout << "[ArtiD1] q_des low-pass filter alpha " << action_filter_alpha_ << std::endl;
    }
    if (policy["stick_axes"]) {
        const auto axes = policy["stick_axes"].as<std::vector<std::string>>();
        if (axes.size() != 3) throw std::runtime_error("[ArtiD1] robot.yaml policy.stick_axes needs 3 names (vx vy yaw)");
        for (std::size_t i = 0; i < 3; ++i) {
            if (axes[i] != "lx" && axes[i] != "ly" && axes[i] != "rx" && axes[i] != "ry") {
                throw std::runtime_error("[ArtiD1] robot.yaml policy.stick_axes: unknown stick " + axes[i] + " (lx ly rx ry)");
            }
            stick_axes_[i] = axes[i];
        }
    }
    if (policy["stick_signs"]) {
        const auto signs = policy["stick_signs"].as<std::vector<double>>();
        if (signs.size() != 3) throw std::runtime_error("[ArtiD1] robot.yaml policy.stick_signs needs 3 values");
        for (std::size_t i = 0; i < 3; ++i) stick_signs_[i] = signs[i];
    }
    std::cout << "[ArtiD1] sticks: vx = " << stick_signs_[0] << " * " << stick_axes_[0] << ", vy = " << stick_signs_[1]
              << " * " << stick_axes_[1] << ", yaw = " << stick_signs_[2] << " * " << stick_axes_[2] << std::endl;

    if (const YAML::Node s2 = YAML::LoadFile(config_file)["stage2"]; s2 && s2["enabled"] && s2["enabled"].as<bool>()) {
        stage2_ = true;
        obs_extra_ = 2;
        if (s2["height_rate"]) height_rate_ = s2["height_rate"].as<double>();
        if (s2["height_step_rate"]) height_step_rate_ = s2["height_step_rate"].as<double>();
        if (s2["initial_height"]) initial_height_ = std::clamp(s2["initial_height"].as<double>(), 0.0, 1.0);
        height_cmd_ = height_target_ = initial_height_;
        const std::unordered_map<std::string, std::string> defaults = {
            {"height_up", "dpad_up"}, {"height_down", "dpad_down"}, {"height_low", "cross"},
            {"height_high", "triangle"}, {"height_mid", "square"}, {"step_toggle", "circle"},
            {"reset_toggle", "start"}};
        if (s2["fall_hold"]) fall_hold_ = s2["fall_hold"].as<bool>();
        for (const char* action : kStage2Actions) {
            std::string button = defaults.at(action);
            if (s2["buttons"] && s2["buttons"][action]) button = s2["buttons"][action].as<std::string>();
            joy_pressed(joy_interface::joy_data_t{}, button);   // validates the name
            stage2_buttons_[action] = button;
            stage2_prev_pressed_[action] = false;
            std::cout << "[ArtiD1] stage 2 " << action << " = " << button << std::endl;
        }
        std::cout << "[ArtiD1] stage 2: obs " << OBSERVATION_SIZE + obs_extra_ << " (height command, step in place), "
                  << "height ramp " << height_rate_ << " /s, start height " << height_cmd_ << std::endl;
    }

    if (const YAML::Node arms = YAML::LoadFile(config_file)["arms"]; arms && arms["enabled"] && arms["enabled"].as<bool>()) {
        if (!stage2_) throw std::runtime_error("[ArtiD1] robot.yaml arms needs the stage2 section (button handling)");
        const char* names[4] = {"left_arm_m8", "left_arm_m8crank_plate", "right_arm_m8", "right_arm_m8crank_plate"};
        for (std::size_t k = 0; k < 4; ++k) {
            bool found = false;
            for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
                if (std::string(kPolicyJointNames[i]) == names[k]) {
                    if (!fixed_joint_[i]) throw std::runtime_error(std::string("[ArtiD1] arms: ") + names[k] + " must be in policy.fixed_joints");
                    arm_index_[k] = i;
                    found = true;
                }
            }
            if (!found) throw std::runtime_error(std::string("[ArtiD1] arms: unknown joint ") + names[k]);
            arm_cur_[k] = fixed_value_[arm_index_[k]];
        }
        if (arms["max_speed"]) arm_max_speed_ = arms["max_speed"].as<double>();
        if (arms["raise_stick"]) arm_raise_stick_ = arms["raise_stick"].as<std::string>();
        if (arms["raise_sign"]) arm_raise_sign_ = arms["raise_sign"].as<double>();
        const std::unordered_map<std::string, std::string> arm_defaults = {
            {"wave", "l1"}, {"updown", "r1"}, {"swing", "l2"}, {"off", "r2"}, {"m8_forward", "dpad_up"}, {"m8_back", "dpad_down"}};
        for (const auto& [action, def] : arm_defaults) {
            std::string button = def;
            if (arms["buttons"] && arms["buttons"][action]) button = arms["buttons"][action].as<std::string>();
            joy_pressed(joy_interface::joy_data_t{}, button);   // validates the name
            arm_buttons_[action] = button;
            stage2_prev_pressed_["arm_" + action] = false;
            std::cout << "[ArtiD1] arms " << action << " = " << button << std::endl;
        }
        arms_enabled_ = true;
        std::cout << "[ArtiD1] arms: motion player on, max speed " << arm_max_speed_ << " rad/s, raise stick "
                  << arm_raise_sign_ << " * " << arm_raise_stick_ << std::endl;
    }

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
            for (std::size_t i = 0; i < OBSERVATION_SIZE + obs_extra_; ++i) std::fprintf(log_file_, ",obs_%zu", i);
            for (const char* name : kPolicyJointNames) std::fprintf(log_file_, ",q_des_%s", name);
            for (const char* name : kPolicyJointNames) std::fprintf(log_file_, ",action_%s", name);
            for (const char* name : kPolicyJointNames) {
                std::fprintf(log_file_, ",pos_%s,vel_%s,tau_%s,temp_%s", name, name, name, name);
            }
            std::fprintf(log_file_, ",imu_qw,imu_qx,imu_qy,imu_qz,imu_gx,imu_gy,imu_gz");
            // Dynamixel status packet error byte (bit 7: Hardware Error Status set, e.g. overheat / overload)
            for (const char* name : kPolicyJointNames) std::fprintf(log_file_, ",err_%s", name);
            // raw shoulder m7 servo states (pos_ / vel_ of the plate joints are the URDF angles solved from them)
            std::fprintf(log_file_, ",m7_pos_left,m7_vel_left,m7_pos_right,m7_vel_right");
            // controller state (stage 2: height command / target, step in place; also the stick command)
            std::fprintf(log_file_, ",height_cmd,height_target,step_in_place,cmd_vx,cmd_vy,cmd_wz,mode,episode");
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
    if (observation_.size() != OBSERVATION_SIZE + obs_extra_) return;
    const auto& o = observation_;
    const std::size_t phase_i = OBS_PHASE + obs_extra_;
    const std::size_t command_i = OBS_COMMAND + obs_extra_;
    constexpr double DEG = 180.0 / PI;
    // [0:6] = heading-frame root x axis (tan) and z axis (norm): body z in the heading frame gives roll / pitch
    const double roll = std::atan2(-o[4], o[5]) * DEG;   // + = right side down (x forward, y left, z up)
    const double pitch = std::atan2(o[3], o[5]) * DEG;   // + = nose down
    std::printf("\n[observe] root roll %+6.2f deg  pitch %+6.2f deg  | tan (%+.3f %+.3f %+.3f) norm (%+.3f %+.3f %+.3f)\n",
                roll, pitch, o[0], o[1], o[2], o[3], o[4], o[5]);
    std::printf("[observe] ang vel heading frame (rad/s)  x %+6.3f  y %+6.3f  z %+6.3f\n", o[6], o[7], o[8]);
    std::printf("[observe] joint               URDF(deg)   |angle| from obs tan-norm(deg)  vel(rad/s)\n");
    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        // joint rotation R: tan = R e_x, norm = R e_z, R e_y = norm x tan; |angle| = acos((trace - 1) / 2) must equal
        // |motor angle| (a mismatch means the observation joint order / mapping is wrong)
        const double* r = &o[OBS_JOINT_ROT + 6 * i];
        const double y_y = r[5] * r[0] - r[3] * r[2];   // (norm x tan).y
        const double trace = r[0] + y_y + r[5];
        const double angle = std::acos(std::clamp((trace - 1.0) / 2.0, -1.0, 1.0));
        const auto motor_index = joint_id_to_motor_index_.at(kinematics_->joint_id(kPolicyJointNames[i]));
        const double joint = last_joint_status_[motor_index].position;
        std::printf("[observe]   %-18s %+8.2f      %7.2f                      %+7.3f\n",
                    kPolicyJointNames[i], joint * DEG, angle * DEG, o[OBS_JOINT_VEL + i]);
    }
    // the plate joints above are solved from the m8 / m7 servos (ARTI_D1_shoulder.hpp)
    std::printf("[observe] shoulder m7 servo (deg)  left %+7.2f  right %+7.2f\n",
                last_motor_status_[shoulder_m7_motor_index_[0]].position * DEG,
                last_motor_status_[shoulder_m7_motor_index_[1]].position * DEG);
    const double* toe = &o[OBS_TOES];
    std::printf("[observe] toes (heading frame, mm)  left (%+6.1f %+6.1f %+6.1f)  right (%+6.1f %+6.1f %+6.1f)\n",
                toe[0] * 1000, toe[1] * 1000, toe[2] * 1000, toe[3] * 1000, toe[4] * 1000, toe[5] * 1000);
    std::printf("[observe] phase sin %+.3f cos %+.3f | command (%+.4f %+.4f %+.4f) | imu q (%+.3f %+.3f %+.3f %+.3f)\n",
                o[phase_i], o[phase_i + 1], o[command_i], o[command_i + 1], o[command_i + 2],
                last_imu_orientation_[0], last_imu_orientation_[1],
                last_imu_orientation_[2], last_imu_orientation_[3]);
    if (stage2_) {
        std::printf("[observe] stage 2: height cmd %.2f (target %.2f)  step in place %s\n", height_cmd_, height_target_,
                    step_in_place_ ? "ON" : "off");
    }
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
        const auto& s = last_joint_status_[joint_id_to_motor_index_.at(kinematics_->joint_id(name))];
        std::fprintf(log_file_, ",%.7g,%.7g,%.5g,%.0f", s.position, s.velocity, s.torque, s.temperature);
    }
    for (const double v : last_imu_orientation_) std::fprintf(log_file_, ",%.8g", v);
    for (const double v : last_imu_angular_velocity_) std::fprintf(log_file_, ",%.7g", v);
    for (const char* name : kPolicyJointNames) {
        std::fprintf(log_file_, ",%d", last_motor_status_[joint_id_to_motor_index_.at(kinematics_->joint_id(name))].error);
    }
    for (const uint8_t m7 : shoulder_m7_motor_index_) {
        std::fprintf(log_file_, ",%.7g,%.7g", last_motor_status_[m7].position, last_motor_status_[m7].velocity);
    }
    std::fprintf(log_file_, ",%.5f,%.5f,%d,%.5f,%.5f,%.5f,%d,%d", height_cmd_, height_target_, step_in_place_ ? 1 : 0,
                 command_[0], command_[1], command_[2], static_cast<int>(mode_), episode_);
    std::fprintf(log_file_, "\n");
}

void robot::ArtiD1::joint_status(const motor_interface::motor_state_t* motor_status,
                                 motor_interface::motor_state_t* joint_status) const {
    std::copy(motor_status, motor_status + NUM_JOINTS, joint_status);
    // the plate joint is passive on the real robot: solve it from the m8 / m7 servos (torque / temperature / error
    // stay the m7 servo's)
    for (std::size_t s = 0; s < 2; ++s) {
        const auto side = static_cast<arti_d1_shoulder::Side>(s);
        const auto& m8 = motor_status[shoulder_m8_motor_index_[s]];
        const auto& m7 = motor_status[shoulder_m7_motor_index_[s]];
        auto& plate = joint_status[shoulder_m7_motor_index_[s]];
        plate.position = arti_d1_shoulder::plate_from(side, m8.position, m7.position);
        plate.velocity = arti_d1_shoulder::plate_velocity(side, m8.position, plate.position, m8.velocity, m7.velocity);
    }
}

void robot::ArtiD1::set_motor_positions(const double* joint_positions) {
    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        motor_command_[joint_id_to_motor_index_.at(kinematics_->joint_id(kPolicyJointNames[i]))].position = joint_positions[i];
    }
    // the plate motor index drives m7: m7 from the (URDF-limited) m8 / plate targets
    for (std::size_t s = 0; s < 2; ++s) {
        const auto side = static_cast<arti_d1_shoulder::Side>(s);
        const double m8 = std::clamp(motor_command_[shoulder_m8_motor_index_[s]].position,
                                     arti_d1_shoulder::M8_MIN, arti_d1_shoulder::M8_MAX);
        auto& m7 = motor_command_[shoulder_m7_motor_index_[s]].position;
        const double plate = std::clamp(m7, arti_d1_shoulder::PLATE_MIN, arti_d1_shoulder::PLATE_MAX);
        m7 = arti_d1_shoulder::m7_from(side, m8, plate, shoulder_m7_command_[s]);
        shoulder_m7_command_[s] = m7;
    }
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
    motor_interface::motor_state_t joint_now[NUM_JOINTS]{};
    motor_manager_->read(motor_status);
    joint_status(motor_status, joint_now);
    for (std::size_t s = 0; s < 2; ++s) shoulder_m7_command_[s] = motor_status[shoulder_m7_motor_index_[s]].position;
    // interpolate in URDF joint space (policy order), so the shoulder 5-bar stays inside the URDF limits on the way
    std::array<double, NUM_JOINTS> start{};
    std::array<double, NUM_JOINTS> goal{};
    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        const auto motor_index = joint_id_to_motor_index_.at(kinematics_->joint_id(kPolicyJointNames[i]));
        start[i] = joint_now[motor_index].position;
        goal[i] = fixed_joint_[i] ? fixed_value_[i] : default_pose_[i];
        std::cout << "[ArtiD1::initialize] " << kPolicyJointNames[i] << " (motor index " << static_cast<int>(motor_index)
                  << ") at " << start[i] << " rad -> default pose " << goal[i] << " rad" << std::endl;
    }
    for (std::size_t s = 0; s < 2; ++s) {
        std::cout << "[ArtiD1::initialize] " << (s == 0 ? "left" : "right") << " shoulder m7 servo at "
                  << motor_status[shoulder_m7_motor_index_[s]].position << " rad" << std::endl;
    }
    // measured pose -> home (robot.yaml policy.home_pose, the real servo zero) -> default pose (= the demo's first frame,
    // where the policy starts); each move at <= STAND_UP_SPEED, at least STAND_UP_TIME
    const auto move = [&](const std::array<double, NUM_JOINTS>& from, const std::array<double, NUM_JOINTS>& to,
                          const char* name) {
        double distance = 0.0;
        for (std::size_t i = 0; i < NUM_JOINTS; ++i) distance = std::max(distance, std::abs(to[i] - from[i]));
        const double duration = std::max(std::chrono::duration<double>(STAND_UP_TIME).count(), distance / STAND_UP_SPEED);
        std::cout << "[ArtiD1::initialize] moving to " << name << " (" << duration << " s)" << std::endl;
        std::array<double, NUM_JOINTS> pose{};
        const auto begin = std::chrono::steady_clock::now();
        auto next_wakeup = begin;
        while (true) {
            if (!motor_manager_->is_running()) {
                throw std::runtime_error("[ArtiD1::initialize] Motor communication stopped while moving to the start pose.");
            }
            const double t = std::chrono::duration<double>(std::chrono::steady_clock::now() - begin).count();
            const double alpha = std::min(1.0, t / duration);
            for (std::size_t i = 0; i < NUM_JOINTS; ++i) pose[i] = from[i] + (to[i] - from[i]) * alpha;
            set_motor_positions(pose.data());
            for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
                motor_command_[i].kp = policy_kp_[i];
                motor_command_[i].kd = policy_kd_[i];
            }
            motor_manager_->write(motor_command_);
            if (alpha >= 1.0) break;
            next_wakeup += STAND_UP_PERIOD;
            std::this_thread::sleep_until(next_wakeup);
        }
    };
    if (has_home_pose_) {
        move(start, home_pose_, "home (real servo zero)");
        std::this_thread::sleep_for(std::chrono::seconds(1));
        move(home_pose_, goal, "default pose (demo first frame)");
    } else {
        move(start, goal, "default pose (demo first frame)");
    }
    gait_phase_ = initial_gait_phase_;
    previous_actions_.fill(0.0);
}

void robot::ArtiD1::observation() {
    observation_.clear();
    observation_.reserve(OBSERVATION_SIZE + obs_extra_);

    motor_interface::motor_state_t motor_status[NUM_JOINTS]{};
    motor_manager_->read(motor_status);
    std::copy(motor_status, motor_status + NUM_JOINTS, last_motor_status_);
    joint_status(motor_status, last_joint_status_);

    // update kinematics with the current URDF joint positions (shoulder plates solved from m8 / m7)
    Eigen::VectorXd joint_positions = Eigen::VectorXd::Zero(NUM_JOINTS);

    for (const auto& joint_id : joint_ids_) {
        const int joint_index = kinematics_->joint_index(joint_id);
        joint_positions[joint_index] = last_joint_status_[joint_id_to_motor_index_.at(joint_id)].position;
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
        observation_.push_back(last_joint_status_[joint_id_to_motor_index_.at(joint_id)].velocity);
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

    // read joy data
    joy_interface::joy_data_t joy_data{};
    joy_handler_->read(joy_data);

    // command = stick * command_max (full stick = the training command range)
    const auto dead = [this](double v) { return std::abs(v) < stick_deadzone_ ? 0.0 : v; };
    const auto stick = [&](const std::string& name) -> double {
        if (name == "lx") return joy_data.stick_lx;
        if (name == "ly") return joy_data.stick_ly;
        if (name == "rx") return joy_data.stick_rx;
        return joy_data.stick_ry;
    };
    for (std::size_t i = 0; i < 3; ++i) {
        command_[i] = std::clamp(stick_signs_[i] * dead(stick(stick_axes_[i])), -1.0, 1.0) * command_max_[i];
    }
    const bool moving_command =
        std::max({std::abs(command_[0]), std::abs(command_[1]), std::abs(command_[2])}) > stand_command_threshold_;

    // stage 2 [127:129]: height command (ramped), step flag. Stage 3 trained every walking clip with the flag on,
    // so a stick command turns it on too (stage 2 has command_max 0: no change there).
    if (stage2_) {
        update_stage2_commands(joy_data);
        observation_.push_back(height_cmd_);
        observation_.push_back((step_in_place_ || moving_command) ? 1.0 : 0.0);
    }

    // gait clock [127:129] (stage 2: [129:131])
    observation_.push_back(std::sin(2.0 * PI * gait_phase_));
    observation_.push_back(std::cos(2.0 * PI * gait_phase_));

    // command [129:132] = command * command_scale
    for (std::size_t i = 0; i < 3; ++i) observation_.push_back(command_[i] * command_scale_[i]);
}

void robot::ArtiD1::update_stage2_commands(const joy_interface::joy_data_t& joy) {
    // edge = button went down this step (press actions fire once per press)
    const auto pressed = [&](const char* action) { return joy_pressed(joy, stage2_buttons_.at(action)); };
    const auto edge = [&](const char* action) {
        const bool now = pressed(action);
        const bool fired = now && !stage2_prev_pressed_[action];
        stage2_prev_pressed_[action] = now;
        return fired;
    };
    const double dt = control_dt_;
    if (pressed("height_up")) height_target_ += height_step_rate_ * dt;
    if (pressed("height_down")) height_target_ -= height_step_rate_ * dt;
    if (edge("height_low")) height_target_ = 0.0;
    if (edge("height_high")) height_target_ = 1.0;
    if (edge("height_mid")) height_target_ = 0.5;
    height_target_ = std::clamp(height_target_, 0.0, 1.0);
    if (edge("step_toggle") && mode_ == Mode::Policy) {
        step_in_place_ = !step_in_place_;
        std::cout << "[ArtiD1] step in place " << (step_in_place_ ? "ON" : "off") << std::endl;
    }
    if (edge("reset_toggle")) reset_pressed_ = true;
    if (arms_enabled_) update_arms(joy);
    if (mode_ != Mode::Policy) {
        // policy stopped: keep the commands at the restart values (initial height, standing)
        height_target_ = height_cmd_ = initial_height_;
        step_in_place_ = false;
        return;
    }
    // same ramp as the training env (ArtiD1AmpEnv._update_stage2_commands, cfg height_rate)
    const double dh = height_rate_ * dt;
    height_cmd_ += std::clamp(height_target_ - height_cmd_, -dh, dh);
}

// Stage 4 arm player: same motions, limits and slew limit as arti_d1 ArtiD1ImitEnv._arm_update.
//   wave (toggle): right arm raised, arm_m8 swinging; updown (toggle): both arms up / down; swing (toggle): arm_m8
//   anti-phase on the gait clock; off: all toggles off; manual (no toggle): raise stick lifts both arms, m8 buttons
//   move both arm_m8. Outside the policy (fall / reset / hold) the arms go back to 0.
void robot::ArtiD1::update_arms(const joy_interface::joy_data_t& joy) {
    const double dt = control_dt_;
    const auto edge = [&](const std::string& action) {
        const bool now = joy_pressed(joy, arm_buttons_.at(action));
        bool& prev = stage2_prev_pressed_["arm_" + action];
        const bool fired = now && !prev;
        prev = now;
        return fired;
    };
    const auto held = [&](const std::string& action) { return joy_pressed(joy, arm_buttons_.at(action)); };
    const bool policy = mode_ == Mode::Policy;
    if (edge("wave") && policy) { arm_wave_ = !arm_wave_; arm_updown_ = false; arm_t_ = 0.0; std::cout << "[ArtiD1] arms wave " << (arm_wave_ ? "ON" : "off") << std::endl; }
    if (edge("updown") && policy) { arm_updown_ = !arm_updown_; arm_wave_ = false; arm_t_ = 0.0; std::cout << "[ArtiD1] arms up/down " << (arm_updown_ ? "ON" : "off") << std::endl; }
    if (edge("swing") && policy) { arm_swing_ = !arm_swing_; std::cout << "[ArtiD1] arms walking swing " << (arm_swing_ ? "ON" : "off") << std::endl; }
    if (edge("off")) { arm_wave_ = arm_updown_ = arm_swing_ = false; arm_m8_manual_ = 0.0; std::cout << "[ArtiD1] arms off" << std::endl; }
    if (held("m8_forward")) arm_m8_manual_ -= 1.0 * dt;   // arm_m8 < 0 = arm forward (URDF axis ~ +z / -z mirrored)
    if (held("m8_back")) arm_m8_manual_ += 1.0 * dt;
    arm_m8_manual_ = std::clamp(arm_m8_manual_, -1.0, 0.3);
    arm_t_ += dt;

    std::array<double, 4> des{};
    if (!policy) {
        arm_wave_ = arm_updown_ = arm_swing_ = false;
        arm_m8_manual_ = 0.0;
    } else if (arm_wave_) {
        const double w = std::sin(2.0 * PI * 1.2 * arm_t_);
        des = {0.0, 0.0, -0.35 + 0.3 * w, 1.0};
    } else if (arm_updown_) {
        const double up = 0.55 * (1.0 - std::cos(2.0 * PI * 0.45 * arm_t_));   // sim: 0.4-0.9 Hz
        des = {-0.2, up, -0.2, up};
    } else {
        const double s = arm_swing_ ? 0.25 * std::sin(2.0 * PI * gait_phase_) : 0.0;
        double raise = 0.0;
        if (arm_raise_stick_ == "lx") raise = joy.stick_lx;
        else if (arm_raise_stick_ == "ly") raise = joy.stick_ly;
        else if (arm_raise_stick_ == "rx") raise = joy.stick_rx;
        else if (arm_raise_stick_ == "ry") raise = joy.stick_ry;
        raise = std::abs(raise) < stick_deadzone_ ? 0.0 : std::clamp(arm_raise_sign_ * raise, 0.0, 1.0) * 1.1;
        const double base = arm_swing_ ? 0.05 : 0.0;
        des = {arm_m8_manual_ + (arm_swing_ ? -0.1 + s : 0.0), std::max(base, raise),
               arm_m8_manual_ + (arm_swing_ ? -0.1 - s : 0.0), std::max(base, raise)};
    }
    static constexpr double lo[4] = {-1.0, -0.20, -1.0, -0.20};
    static constexpr double hi[4] = {0.30, 1.10, 0.30, 1.10};
    const double max_step = arm_max_speed_ * dt;
    for (std::size_t k = 0; k < 4; ++k) {
        const double d = std::clamp(des[k], lo[k], hi[k]) - arm_cur_[k];
        arm_cur_[k] += std::clamp(d, -max_step, max_step);
        fixed_value_[arm_index_[k]] = arm_cur_[k];
    }
}

void robot::ArtiD1::start_return(const char* why) {
    // start from the measured pose, so nothing jumps (the robot may have been picked up with the legs anywhere)
    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        reset_pose_[i] = last_joint_status_[joint_id_to_motor_index_.at(kinematics_->joint_id(kPolicyJointNames[i]))].position;
    }
    for (std::size_t s = 0; s < 2; ++s) shoulder_m7_command_[s] = last_motor_status_[shoulder_m7_motor_index_[s]].position;
    mode_ = Mode::Returning;
    height_target_ = height_cmd_ = initial_height_;
    step_in_place_ = false;
    std::cout << "[ArtiD1] Policy stopped (" << why << "): returning to the default pose. Set the robot down, then press "
              << stage2_buttons_.at("reset_toggle") << " again to restart the policy." << std::endl;
}

void robot::ArtiD1::resume_policy() {
    previous_actions_.fill(0.0);
    q_des_filter_ready_ = false;
    gait_phase_ = initial_gait_phase_;
    height_target_ = height_cmd_ = initial_height_;
    step_in_place_ = false;
    ++episode_;
    mode_ = Mode::Policy;
    std::cout << "[ArtiD1] Policy restarted (episode " << episode_ << ")." << std::endl;
}

void robot::ArtiD1::control_without_policy() {
    action_.assign(NUM_JOINTS, 0.0);
    if (mode_ == Mode::Limp) {
        // like safe_stop: Position P Gain 0, the motor.yaml D gain only
        for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
            const auto motor_index = joint_id_to_motor_index_.at(kinematics_->joint_id(kPolicyJointNames[i]));
            action_[i] = last_joint_status_[motor_index].position;
            motor_command_[i] = motor_interface::motor_command_t{};
            motor_command_[i].kd = policy_kd_[i];
        }
    } else {
        // Returning: each joint moves toward its default (or fixed) angle at <= STAND_UP_SPEED; Holding: stay there
        bool arrived = true;
        const double max_step = STAND_UP_SPEED * control_dt_;
        for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
            const double goal = fixed_joint_[i] ? fixed_value_[i] : default_pose_[i];
            const double d = goal - reset_pose_[i];
            if (std::abs(d) > max_step) arrived = false;
            reset_pose_[i] += std::clamp(d, -max_step, max_step);
            action_[i] = reset_pose_[i];
        }
        set_motor_positions(reset_pose_.data());
        for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
            motor_command_[i].kp = policy_kp_[i];
            motor_command_[i].kd = policy_kd_[i];
        }
        if (mode_ == Mode::Returning && arrived) {
            mode_ = Mode::Holding;
            std::cout << "[ArtiD1] Default pose reached, holding. Press " << stage2_buttons_.at("reset_toggle")
                      << " to restart the policy." << std::endl;
        }
    }
    if (log_file_ != nullptr) write_log_row();
    motor_manager_->write(motor_command_);
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
    if (mode_ == Mode::Policy && !observation_.empty() && observation_[5] < std::cos(max_tilt_)) {
        const std::string tilt = std::to_string(std::acos(std::clamp(observation_[5], -1.0, 1.0)) * 180.0 / PI);
        if (!fall_hold_) safe_stop("Pelvis tilt " + tilt + " deg > max_tilt: fallen, joints damped.");
        mode_ = Mode::Limp;
        std::cout << "[ArtiD1] Pelvis tilt " << tilt << " deg > max_tilt: fallen, servos limp. Pick the robot up and "
                  << "press the reset button (" << stage2_buttons_.at("reset_toggle") << ")." << std::endl;
    }
    if (reset_pressed_) {
        reset_pressed_ = false;
        if (mode_ == Mode::Policy || mode_ == Mode::Limp) {
            start_return(mode_ == Mode::Limp ? "after the fall" : "reset button");
        } else if (mode_ == Mode::Holding) {
            resume_policy();
        }   // Returning: wait until the pose is reached
    }
    if (mode_ != Mode::Policy) {
        control_without_policy();
        return;
    }
    if (controller_ == nullptr) {
        safe_stop("No policy (robot.yaml model_file missing): run with --observe or export agent.onnx first.");
    }
    if (!observation_.empty()) {
        // action_ = q_des (rad, policy order): default pose + scaled action + nominal gait at the next clock
        // phase, clamped to the training (URDF) joint limits. The shoulder plate targets become m7 servo goals.
        controller_->update(observation_, action_);
        if (action_.size() != NUM_JOINTS) {
            throw std::runtime_error("[ArtiD1::control] The ONNX model has no 14-joint 'q_des' output.");
        }
        if (action_filter_alpha_ < 1.0) {
            // EMA on q_des = EMA on the scaled action (q_des is linear in the action at a fixed height)
            for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
                if (!q_des_filter_ready_) q_des_filtered_[i] = action_[i];
                q_des_filtered_[i] += action_filter_alpha_ * (action_[i] - q_des_filtered_[i]);
                action_[i] = q_des_filtered_[i];
            }
            q_des_filter_ready_ = true;
        }
        for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
            if (fixed_joint_[i]) action_[i] = fixed_value_[i];
        }
        set_motor_positions(action_.data());
        for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
            motor_command_[i].kp = policy_kp_[i];
            motor_command_[i].kd = policy_kd_[i];
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
        // stage 2: the step-in-place flag keeps the clock running at zero command (ArtiD1AmpEnv._standing_command)
        const bool standing = std::max({std::abs(command_[0]), std::abs(command_[1]), std::abs(command_[2])})
                              <= stand_command_threshold_ && !step_in_place_;
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
#include <stdexcept>

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include "robot/core/kinematics.hpp"

kinematics::Kinematics::Kinematics(const std::string& urdf_file) {
    pinocchio::urdf::buildModel(urdf_file, model_);

    data_ = pinocchio::Data(model_);
}

void kinematics::Kinematics::update(const Eigen::VectorXd& q) {
    if (q.size() != model_.nq) throw std::runtime_error("[Kinematics::update] Invalid q size.");

    pinocchio::forwardKinematics(model_, data_, q);

    pinocchio::updateFramePlacements(model_, data_);
}

const std::string& kinematics::Kinematics::frame_name(pinocchio::FrameIndex frame_id) const {
    if (frame_id >= static_cast<pinocchio::FrameIndex>(model_.nframes))
        throw std::runtime_error("[Kinematics::frame_name] Frame not found.");

    return model_.frames[frame_id].name;
}

const std::string& kinematics::Kinematics::joint_name(pinocchio::JointIndex joint_id) const {
    if (joint_id >= static_cast<pinocchio::JointIndex>(model_.njoints))
        throw std::runtime_error("[Kinematics::joint_name] Joint not found.");
    return model_.names[joint_id];
}

pinocchio::JointIndex kinematics::Kinematics::joint_id(const std::string& name) const {
    const auto id = model_.getJointId(name);

    if (id >= model_.njoints) throw std::runtime_error("[Kinematics::joint_id] Joint not found.");

    return id;
}

pinocchio::FrameIndex kinematics::Kinematics::frame_id(const std::string& name) const {
    const auto id = model_.getFrameId(name);

    if (id >= model_.nframes) throw std::runtime_error("[Kinematics::frame_id] Frame not found.");

    return id;
}

int kinematics::Kinematics::joint_index(pinocchio::JointIndex joint_id) const {
    return model_.joints[joint_id].idx_q();
}

Eigen::Matrix3d kinematics::Kinematics::joint_rotation(pinocchio::JointIndex joint_id) const {
    return data_.oMi[joint_id].rotation();
}

Eigen::Vector3d kinematics::Kinematics::frame_position(pinocchio::FrameIndex frame_id) const {
    return data_.oMf[frame_id].translation();
}

Eigen::Matrix3d kinematics::Kinematics::frame_rotation(pinocchio::FrameIndex frame_id) const {
    return data_.oMf[frame_id].rotation();
}
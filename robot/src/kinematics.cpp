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

pinocchio::FrameIndex kinematics::Kinematics::frame_id(const std::string& name) const {
    const auto id = model_.getFrameID(name);

    if (id >= model_.nframes) throw std::runtime_error("[Kinematics::frame_id] Frame not found.");

    return id;
}

Eigen::Vector3d kinematics::Kinematics::frame_position(pinocchio::FrameIndex frame_id) const {
    return data_.oMf[frame_id].translation();
}

Eigen::Matrix3d kinematics::Kinematics::frame_rotation(pinocchio::FrameIndex frame_id) const {
    return data_.oMf[frame_id].rotation();
}


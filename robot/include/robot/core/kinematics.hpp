#ifndef ROBOT_CORE_KINEMATICS_HPP_
#define ROBOT_CORE_KINEMATICS_HPP_

#include <string>

#include <Eigen/Core>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>

namespace kinematics {

class Kinematics {
public:
    explicit Kinematics(const std::string& urdf_file);

    void update(const Eigen::VectorXd& q);

    pinocchio::JointIndex joint_id(const std::string& name) const;

    pinocchio::FrameIndex frame_id(const std::string& name) const;

    int joint_index(pinocchio::JointIndex joint_id) const;

    Eigen::Matrix3d joint_rotation(pinocchio::JointIndex joint_id) const;

    Eigen::Vector3d frame_position(pinocchio::FrameIndex frame_id) const;

    Eigen::Matrix3d frame_rotation(pinocchio::FrameIndex frame_id) const;

    const pinocchio::Model& model() const { return model_; }

private:
    pinocchio::Model model_;

    pinocchio::Data data_;
};

} // namespace kinematics
 
#endif // ROBOT_CORE_KINEMATICS_HPP_
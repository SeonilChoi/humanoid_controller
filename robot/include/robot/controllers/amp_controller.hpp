#ifndef ROBOT_CONTROLLERS_AMP_CONTROLLER_HPP_
#define ROBOT_CONTROLLERS_AMP_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "robot/core/controller.hpp"

namespace controller {

class AmpController : public Controller {
public:
    explicit AmpController(const std::string& model_file);

    virtual ~AmpController();

    void initialize() override;

    void shutdown() override;

    // action = joint position targets (ONNX output "q_des" when the model has it, else its first output).
    void update(const std::vector<double>& observation, std::vector<double>& action) override;

    // Raw policy actions clamped to [-1, 1] (ONNX output "actions") from the last update; the policy observes
    // them as previous_actions. Empty for single-output models.
    const std::vector<double>& policy_actions() const { return policy_actions_; }

private:
    std::vector<double> policy_actions_;

    struct Impl;

    std::unique_ptr<Impl> impl_;

    const std::string model_file_;

    bool initialized_{false};
};

}

#endif // ROBOT_CONTROLLERS_AMP_CONTROLLER_HPP_
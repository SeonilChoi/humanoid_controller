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

    void update(const std::vector<double>& observation, std::vector<double>& action) override;

private:
    struct Impl;

    std::unique_ptr<Impl> impl_;

    const std::string model_file_;

    bool initialized_{false};
};

}

#endif // ROBOT_CONTROLLERS_AMP_CONTROLLER_HPP_
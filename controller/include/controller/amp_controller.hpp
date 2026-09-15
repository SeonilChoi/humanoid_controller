#ifndef CONTROLLER_AMP_CONTROLLER_HPP_
#define CONTROLLER_AMP_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller/core/controller.hpp"

namespace amp {

class AmpController : public controller::Controller {
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

} // namespace amp

#endif // CONTROLLER_AMP_CONTROLLER_HPP_
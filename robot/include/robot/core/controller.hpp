#ifndef ROBOT_CORE_CONTROLLER_HPP_
#define ROBOT_CORE_CONTROLLER_HPP_

#include <vector>

namespace controller {

class Controller {
public:
    virtual ~Controller() = default;

    virtual void initialize() = 0;

    virtual void shutdown() = 0;

    virtual void update(const std::vector<double>& observation, std::vector<double>& action) = 0;
};

} // namespace controller

#endif // ROBOT_CORE_CONTROLLER_HPP_
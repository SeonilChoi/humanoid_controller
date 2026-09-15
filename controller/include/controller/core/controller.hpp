#ifndef CONTROLLER_CORE_CONTROLLER_HPP_
#define CONTROLLER_CORE_CONTROLLER_HPP_

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

#endif // CONTROLLER_CORE_CONTROLLER_HPP_
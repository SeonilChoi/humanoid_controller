#ifndef ROBOT_ROBOTS_OLAF_HPP_
#define ROBOT_ROBOTS_OLAF_HPP_

#include "robot/core/robot.hpp"

namespace olaf {

class Olaf : public robot::Robot {
public:
    explicit Olaf(const std::string& config_file)
    : robot::Robot(config_file) {}

    virtual ~Olaf() = default;

    const std::vector<double>& observation() override;

    void control() override;
};

} // namespace olaf

#endif // ROBOT_ROBOTS_OLAF_HPP_
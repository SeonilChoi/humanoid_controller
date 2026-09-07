#ifndef ROBOT_ROBOTS_OLAF_HPP_
#define ROBOT_ROBOTS_OLAF_HPP_

#include "robot/core/robot.hpp"

namespace olaf {

class Olaf : public Robot {
public:
    explicit Olaf(const std::string& config_file)
    : robot::Robot(config_file)

    virtual ~Olaf();

    void start() override;

    void stop() override;

    void observation() override;

    void control() override;
};

} // namespace olaf

#endif // ROBOT_ROBOTS_OLAF_HPP_
#ifndef JOY_PLAYSTATION_DUALSENSE_HPP_
#define JOY_PLAYSTATION_DUALSENSE_HPP_

#include <mutex>
#include <string>
#include <cstdint>

#include "joy/core/joy.hpp"
#include "joy/core/joy_interface.hpp"

namespace playstation {

class DualSense : public joy_interface::Joy {
public:
    explicit DualSense(const std::string& device, uint32_t period, const std::string& axis_layout)
    : joy_interface::Joy(device, period, axis_layout) {}

    virtual ~DualSense();

    void initialize() override;
    
    void update() override;

    void shutdown() override;

    void read(joy_interface::joy_data_t& data) override;

private:
    joy_interface::joy_data_t data_;

    int fd_{-1};

    std::mutex mutex_;

    bool initialized_{false};
};

} // namespace playstation

#endif // JOY_PLAYSTATION_DUALSENSE_HPP_
#ifndef JOY_CORE_JOY_HPP_
#define JOY_CORE_JOY_HPP_

#include <string>

#include "joy/core/joy_interface.hpp"

namespace joy_interface {

class Joy {
public:
    explicit Joy(const std::string& device, uint32_t period)
    : device_(device), period_(period) {}

    virtual ~Joy() = default;

    virtual void initialize() = 0;

    virtual void update() = 0;

    virtual void shutdown() = 0;

    virtual void read(joy_data_t& data) = 0;

    const std::string device() const { return device_; }

    uint32_t period() const { return period_; }

protected:
    const std::string device_;

    const uint32_t period_;
};

} // namespace joy_interface

#endif // JOY_CORE_JOY_HPP_
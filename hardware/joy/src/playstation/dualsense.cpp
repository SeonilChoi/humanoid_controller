#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <linux/joystick.h>

#include <cstring>
#include <iostream>
#include <stdexcept>

#include "joy/playstation/dualsense.hpp"

namespace {
constexpr auto RECONNECT_PERIOD = std::chrono::seconds(1);
}

float deadzone(float v, float threshold = 0.1f) {
    if (v > -threshold && v < threshold) return 0.0f;
    return v;
}

playstation::DualSense::~DualSense() {
    shutdown();
}

void playstation::DualSense::initialize() {
    if (initialized_) return;

    fd_ = open(device_.c_str(), O_RDONLY | O_NONBLOCK);
    if (fd_ < 0) throw std::runtime_error("[DualSense::initialize] Failed to open device: " + device_);

    initialized_ = true;
}

void playstation::DualSense::disconnect(int error) {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
    {
        // a dropped pad sends no release events: clear everything so the robot gets the zero command (stand)
        std::lock_guard<std::mutex> lock(mutex_);
        data_ = joy_interface::joy_data_t{};
    }
    next_open_ = std::chrono::steady_clock::now() + RECONNECT_PERIOD;
    std::cerr << "[DualSense] " << device_ << " lost (" << std::strerror(error)
              << "): command set to zero, reconnecting..." << std::endl;
}

void playstation::DualSense::try_reconnect() {
    const auto now = std::chrono::steady_clock::now();
    if (now < next_open_) return;
    next_open_ = now + RECONNECT_PERIOD;
    fd_ = open(device_.c_str(), O_RDONLY | O_NONBLOCK);
    // the kernel replays the current button / axis state as JS_EVENT_INIT events, read below
    if (fd_ >= 0) std::cerr << "[DualSense] " << device_ << " reconnected." << std::endl;
}

void playstation::DualSense::update() {
    if (!initialized_) return;
    if (fd_ < 0) {
        try_reconnect();
        if (fd_ < 0) return;
    }

    js_event event{};
    while (true) {
        const ssize_t n = ::read(fd_, &event, sizeof(event));
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) break;
            if (errno == EINTR) continue;
            disconnect(errno);
            return;
        }
        if (n != static_cast<ssize_t>(sizeof(event))) break;

        const uint8_t type = event.type & ~JS_EVENT_INIT;

        std::lock_guard<std::mutex> lock(mutex_);
        if (type == JS_EVENT_BUTTON) {
            const bool pressed = event.value != 0;
            switch (event.number) {
                case 0: data_.cross = pressed; break;
                case 1: data_.circle = pressed; break;
                case 2: data_.triangle = pressed; break;
                case 3: data_.square = pressed; break;
                case 4: data_.l1 = pressed; break;
                case 5: data_.r1 = pressed; break;
                case 6: data_.l2 = pressed; break;
                case 7: data_.r2 = pressed; break;
                case 8: data_.select = pressed; break;
                case 9: data_.start = pressed; break;
                case 10: data_.center = pressed; break;
                default: break;
            }
        } else if (type == JS_EVENT_AXIS) {
            const float v = static_cast<float>(event.value) / 32767.0f;
            if (axis_layout_ == "generic") {
                switch (event.number) {
                    case 0: data_.stick_lx = deadzone(v); break;
                    case 1: data_.stick_ly = deadzone(-v); break;
                    case 2: data_.stick_rx = deadzone(v); break;
                    case 3: data_.l2_analog = deadzone(v); break;
                    case 4: data_.r2_analog = deadzone(v); break;
                    case 5: data_.stick_ry = deadzone(-v); break;
                    case 6: data_.dpad_x = v; break;
                    case 7: data_.dpad_y = -v; break;
                    default: break;
                }
            } else if (axis_layout_ == "playstation") {
                switch (event.number) {
                    case 0: data_.stick_lx = deadzone(v); break;
                    case 1: data_.stick_ly = deadzone(-v); break;
                    case 2: data_.l2_analog = deadzone(v); break;
                    case 3: data_.stick_rx = deadzone(v); break;
                    case 4: data_.stick_ry = deadzone(-v); break;
                    case 5: data_.r2_analog = deadzone(v); break;
                    case 6: data_.dpad_x = v; break;
                    case 7: data_.dpad_y = -v; break;
                    default: break;
                }
            }
        }
    }
}

void playstation::DualSense::shutdown() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }

    initialized_ = false;
}

void playstation::DualSense::read(joy_interface::joy_data_t& data) {
    std::lock_guard<std::mutex> lock(mutex_);
    data = data_;
}
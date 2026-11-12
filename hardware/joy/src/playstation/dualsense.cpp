#include <fcntl.h>
#include <cstring>
#include <iostream>
#include <errno.h>
#include <unistd.h>
#include <linux/joystick.h>

#include "joy/playstation/dualsense.hpp"

float deadzone(float v, float threshold = 0.1f) {
    if (v > -threshold && v < threshold) return 0.0f;
    return v;
}

playstation::DualSense::~DualSense() {
    shutdown();
}

void playstation::DualSense::initialize() {
    if (initialized_) return;

    // A missing pad is not fatal: start disconnected (all inputs zero) and let update() retry every second, the same
    // as after a drop, so the robot can start before the pad is paired / powered.
    fd_ = open(device_.c_str(), O_RDONLY | O_NONBLOCK);
    if (fd_ < 0) {
        std::cerr << "[DualSense] " << device_ << " not found (" << std::strerror(errno)
                  << "): all inputs zero, retrying every second" << std::endl;
        connected_.store(false);
        next_retry_ = std::chrono::steady_clock::now() + std::chrono::seconds(1);
    } else {
        connected_.store(true);
    }
    initialized_ = true;
}

void playstation::DualSense::disconnect(const char* reason) {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
    {
        std::lock_guard<std::mutex> lock(mutex_);
        data_ = joy_interface::joy_data_t{};
    }
    if (connected_.exchange(false)) {
        std::cerr << "[DualSense] " << device_ << " lost (" << reason << "): all inputs zero, reconnecting" << std::endl;
    }
    next_retry_ = std::chrono::steady_clock::now() + std::chrono::seconds(1);
}

void playstation::DualSense::update() {
    if (!initialized_) return;

    if (fd_ < 0) {  // lost: retry once a second; the joystick driver replays the current state on open (JS_EVENT_INIT)
        if (std::chrono::steady_clock::now() < next_retry_) return;
        fd_ = open(device_.c_str(), O_RDONLY | O_NONBLOCK);
        if (fd_ < 0) {
            next_retry_ = std::chrono::steady_clock::now() + std::chrono::seconds(1);
            return;
        }
        connected_.store(true);
        std::cerr << "[DualSense] " << device_ << " reconnected" << std::endl;
    }

    js_event event{};
    while (true) {
        const ssize_t n = ::read(fd_, &event, sizeof(event));
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) break;
            if (errno == EINTR) continue;
            disconnect(std::strerror(errno));
            return;
        }
        if (n == 0) {
            disconnect("end of file");
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
    data = connected_.load() ? data_ : joy_interface::joy_data_t{};
}
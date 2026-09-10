#include <fcntl.h>
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

    fd_ = open(device_.c_str(), O_RDONLY | O_NONBLOCK);
    if (fd_ < 0) throw std::runtime_error("[DualSense::initialize] Failed to open device: " + device_);

    initialized_ = true;
}

void playstation::DualSense::update() {
    if (!initialized_) return;

    js_event event{};
    while (true) {
        const ssize_t n = ::read(fd_, &event, sizeof(event));
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) break;
            throw std::runtime_error("[DualSense::update] read failed.");
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
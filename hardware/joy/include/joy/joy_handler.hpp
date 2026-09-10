#ifndef JOY_JOY_HANDLER_HPP_
#define JOY_JOY_HANDLER_HPP_

#include <atomic>
#include <thread>
#include <memory>
#include <string>
#include <cstdint>

#include "joy/core/joy.hpp"
#include "joy/core/joy_interface.hpp"

namespace joy_handler {

class JoyHandler {
public:
    explicit JoyHandler(const std::string& config_file) {
        load(config_file);
        initialize();
    }

    virtual ~JoyHandler();

    void initialize();

    void start();

    void stop();

    void read(joy_interface::joy_data_t& data);

private:
    void load(const std::string& config_file);

    void run();

    std::unique_ptr<joy_interface::Joy> joy_;

    std::atomic<bool> running_{false};

    std::thread thread_;
};

} // namespace joy_handler

#endif // JOY_JOY_HANDLER_HPP_
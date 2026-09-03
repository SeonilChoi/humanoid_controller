#ifndef MOTOR_MASTER_UNITREE_MASTER_HPP_
#define MOTOR_MASTER_UNITREE_MASTER_HPP_

#include <cerrno>
#include <string>
#include <chrono>
#include <memory>
#include <cstring>
#include <cstdint>
#include <cstddef>
#include <stdexcept>

#include <poll.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>

#include "motor/core/motor_master.hpp"
#include "motor/driver/unitree_driver.hpp"

namespace unitree {

constexpr speed_t UNITREE_BAUDRATE = B4000000;
constexpr std::size_t TX_PACKET_SIZE = 17;
constexpr std::size_t RX_PACKET_SIZE = 16;
constexpr auto TIMEOUT = std::chrono::microseconds(200);

class UnitreeMaster : public motor_interface::MotorMaster {
public:
    UnitreeMaster(uint32_t period, uint8_t number_of_motors, const std::string& device)
    : motor_interface::MotorMaster(period, number_of_motors), device_(device) {}

    virtual ~UnitreeMaster() {
        if (fd_ >= 0) {
            ::close(fd_);
            fd_ = -1;
        }
    }

    void add_motor(uint8_t id, double gear_ratio, double zero_offset) override {
        drivers_[id] = std::make_unique<unitree::UnitreeDriver>(gear_ratio, zero_offset);
    }

    void initialize() override {
        fd_ = ::open(device_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd_ < 0) throw std::runtime_error("[UnitreeMaster::initialize] Failed to open serial port.");

        termios tio{};
        tio.c_cflag = UNITREE_BAUDRATE | CS8 | CLOCAL | CREAD;
        tio.c_iflag = IGNPAR;
        tio.c_oflag = 0;
        tio.c_lflag = 0;
        tio.c_cc[VTIME] = 0;
        tio.c_cc[VMIN] = 1;

        if (::tcflush(fd_, TCIOFLUSH) != 0) throw std::runtime_error("[UnitreeMaster::initialize] Failed to flush serial port.");

        if (::tcsetattr(fd_, TCSANOW, &tio) != 0) throw std::runtime_error("[UnitreeMaster::initialize] Failed to configure serial port.");
    }

    void update(uint8_t id, const motor_interface::motor_command_t& command, motor_interface::motor_state_t& status) override {
        auto& driver = *drivers_.at(id);

        uint8_t tx[TX_PACKET_SIZE]{};
        const std::size_t tx_size = driver.encode(command, tx, sizeof(tx));

        send_packet(tx, tx_size);

        uint8_t rx[RX_PACKET_SIZE]{};
        const std::size_t rx_size = receive_packet(rx, sizeof(rx));

        driver.decode(rx, rx_size, status);
    }

private:
    void send_packet(const uint8_t* tx, std::size_t size) {
        std::size_t total = 0;

        const auto deadline = std::chrono::steady_clock::now() + TIMEOUT;

        while (total < size) {
            if (std::chrono::steady_clock::now() >= deadline) throw std::runtime_error("[Unitree::send_packet] timeout.");

            const ssize_t written = ::write(fd_, tx + total, size - total);

            if (written > 0) {
                total += static_cast<std::size_t>(written);
                continue;
            }

            if (written < 0) {
                if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) continue;
            }
            throw std::runtime_error("[UnitreeMaster::send_packet] Failed to write serial data");
        }
    }

    std::size_t receive_packet(uint8_t* rx, std::size_t size) {
        std::size_t total = 0;

        const auto deadline = std::chrono::steady_clock::now() + TIMEOUT;

        while (total < size) {
            const auto now = std::chrono::steady_clock::now();

            if (now >= deadline) throw std::runtime_error("[UnitreeMaster::receive_packet] timeout.");

            const auto remaining = std::chrono::duration_cast<std::chrono::nanoseconds>(deadline - now);

            timespec ts{};
            ts.tv_sec = static_cast<time_t>(remaining.count() / 1'000'000'000LL);
            ts.tv_nsec = static_cast<long>(remaining.count() % 1'000'000'000LL);

            pollfd pfd{};
            pfd.fd = fd_;
            pfd.events = POLLIN;

            const int ret = ::ppoll(&pfd, 1, &ts, nullptr);
            if (ret < 0) {
                if (errno == EINTR) continue;
                throw std::runtime_error("[UnitreeMaster::receive_packet] ppoll failed.");
            }

            if (ret == 0) throw std::runtime_error("[UnitreeMaster::receive_packet] timeout.");

            if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) throw std::runtime_error("[UnitreeMaster::receive_packet] serial error.");

            const ssize_t received = ::read(fd_, rx + total, size - total);

            if (received > 0) {
                total += static_cast<std::size_t>(received);
                continue;
            }

            if (received < 0) {
                if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) continue;
                throw std::runtime_error("[UnitreeMaster::receive_packet] read failed.");
            }
        }

        return total;
    }

    std::string device_;

    int fd_{-1};
};

}

#endif // MOTOR_MASTER_UNITREE_MASTER_HPP_
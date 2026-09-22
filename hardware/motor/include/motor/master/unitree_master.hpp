#ifndef MOTOR_MASTER_UNITREE_MASTER_HPP_
#define MOTOR_MASTER_UNITREE_MASTER_HPP_

#include <cerrno>
#include <string>
#include <chrono>
#include <memory>
#include <cstring>
#include <cstdint>
#include <cstddef>
#include <iostream>
#include <stdexcept>

#include <poll.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>

#include "motor/core/motor_master.hpp"
#include "motor/driver/unitree_driver.hpp"

namespace unitree {

constexpr speed_t UNITREE_BAUDRATE = B4000000;
constexpr auto TIMEOUT = std::chrono::microseconds(10000);

class UnitreeMaster : public motor_interface::MotorMaster {
public:
    UnitreeMaster(uint32_t period, const std::string& device)
    : motor_interface::MotorMaster(period), device_(device) {}

    virtual ~UnitreeMaster() {
        if (fd_ >= 0) {
            ::close(fd_);
            fd_ = -1;
        }
    }

    void add_motor(uint8_t id, double gear_ratio, double zero_offset, uint32_t pulse_per_revolution, double min, double max) override {
        drivers_[id] = std::make_unique<unitree::UnitreeDriver>(id, gear_ratio, zero_offset, pulse_per_revolution, min, max);
        ids_[n_ids_++] = id;
    }

    void initialize() override {
        fd_ = ::open(device_.c_str(), O_RDWR | O_NOCTTY);
        if (fd_ < 0) throw std::runtime_error("[UnitreeMaster::initialize] Failed to open serial port.");

        termios tio{};
        if (::tcgetattr(fd_, &tio) != 0) throw std::runtime_error("[UnitreeMaster::initialize] tcgetattr failed.");

        ::cfmakeraw(&tio);

        tio.c_cflag |= CLOCAL | CREAD;
        tio.c_cflag &= ~PARENB;
        tio.c_cflag &= ~CSTOPB;
        tio.c_cflag &= ~CSIZE;
        tio.c_cflag |= CS8;

        if (::cfsetispeed(&tio, UNITREE_BAUDRATE) != 0 ||
            ::cfsetospeed(&tio, UNITREE_BAUDRATE) != 0) {
            throw std::runtime_error("[UnitreeMaster::initialize] Failed to set baudrate.");
        }

        tio.c_cc[VMIN] = 0;
        tio.c_cc[VTIME] = 0;

        if (::tcflush(fd_, TCIOFLUSH) != 0) throw std::runtime_error("[UnitreeMaster::initialize] Failed to flush serial port.");

        if (::tcsetattr(fd_, TCSANOW, &tio) != 0) throw std::runtime_error("[UnitreeMaster::initialize] Failed to configure serial port.");
    }

    void shutdown() override {
        if (fd_ >= 0) ::tcflush(fd_, TCIOFLUSH);

        for (auto& [id, driver] : drivers_) {
            try {
                uint8_t tx[TX_PACKET_SIZE]{};
                const std::size_t tx_size = driver->disable(tx, sizeof(tx));

                send_packet(tx, tx_size);

                uint8_t rx[RX_PACKET_SIZE]{};
                const std::size_t rx_size = receive_packet(rx, sizeof(rx));

                motor_interface::motor_state_t status{};
                driver->decode(rx, rx_size, status);
            } catch (const std::exception& e) {
                std::cerr << "[UnitreeMaster::shutdown] id " << static_cast<int>(id)
                          << ": " << e.what() << std::endl;
            }
        }
    }

    void update(const motor_interface::motor_command_t* commands, motor_interface::motor_state_t* status) override {
        for (uint8_t i = 0; i < n_ids_; ++i) {
            const uint8_t id = ids_[i];
            auto& driver = *drivers_.at(id);

            uint8_t tx[TX_PACKET_SIZE]{};
            const std::size_t tx_size = driver.encode(commands[i], tx, sizeof(tx));

            send_packet(tx, tx_size);

            uint8_t rx[RX_PACKET_SIZE]{};
            const std::size_t rx_size = receive_packet(rx, sizeof(rx));

            driver.decode(rx, rx_size, status[i]);
        }
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

        ::tcdrain(fd_);
    }

    void wait_readable(std::chrono::steady_clock::time_point deadline) {
        while (true) {
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

            if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) {
                throw std::runtime_error("[UnitreeMaster::receive_packet] serial error.");
            }

            return;
        }
    }

    std::size_t read_bytes(uint8_t* buffer, std::size_t size, std::chrono::steady_clock::time_point deadline) {
        std::size_t total = 0;

        while (total < size) {
            wait_readable(deadline);

            const ssize_t received = ::read(fd_, buffer + total, size - total);

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

    std::size_t receive_packet(uint8_t* rx, std::size_t size) {
        if (size < 2) throw std::runtime_error("[UnitreeMaster::receive_packet] Invalid buffer size.");

        const auto deadline = std::chrono::steady_clock::now() + TIMEOUT;

        bool saw_header = false;
        while (!saw_header) {
            uint8_t byte{};
            read_bytes(&byte, 1, deadline);

            if (byte != 0xFD) continue;

            read_bytes(&byte, 1, deadline);
            if (byte == 0xEE) {
                saw_header = true;
            }
        }

        rx[0] = 0xFD;
        rx[1] = 0xEE;
        read_bytes(rx + 2, size - 2, deadline);

        return size;
    }

    std::string device_;

    int fd_{-1};
};

}

#endif // MOTOR_MASTER_UNITREE_MASTER_HPP_
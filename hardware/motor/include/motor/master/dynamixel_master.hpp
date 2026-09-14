#ifndef MOTOR_MASTER_DYNAMIXEL_MASTER_HPP_
#define MOTOR_MASTER_DYNAMIXEL_MASTER_HPP_

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
#include "motor/driver/dynamixel_driver.hpp"

namespace dynamixel {

constexpr auto DYNAMIXEL_BAUDRATE = B1000000;
constexpr auto DYNAMIXEL_TIMEOUT = std::chrono::milliseconds(20);

constexpr uint8_t BROADCAST_ID = 0xFE;
constexpr uint8_t INST_WRITE = 0x03;
constexpr uint8_t INST_SYNC_WRITE = 0x83;
constexpr uint8_t INST_SYNC_READ = 0x82;
constexpr uint8_t INST_STATUS = 0x55;

constexpr std::size_t MAX_PACKET_SIZE = 256;

class DynamixelMaster : public motor_interface::MotorMaster {
public:
    DynamixelMaster(uint32_t period, const std::string& device)
    : motor_interface::MotorMaster(period), device_(device) {}

    virtual ~DynamixelMaster() {
        if (fd_ >= 0) {
            ::close(fd_);
            fd_ = -1;
        }
    }

    void add_motor(uint8_t id, double gear_ratio, double zero_offset, uint32_t pulse_per_revolution) override {
        drivers_[id] = std::make_unique<dynamixel::DynamixelDriver>(id, gear_ratio, zero_offset, pulse_per_revolution);
        ids_[n_ids_++] = id;
    }
    
    void initialize() override {
        fd_ = ::open(device_.c_str(), O_RDWR | O_NOCTTY);
        if (fd_ < 0) throw std::runtime_error("[DynamixelMaster::initialize] Failed to open serial port.");

        termios tio{};
        if (::tcgetattr(fd_, &tio) != 0) {
            throw std::runtime_error("[DynamixelMaster::initialize] tcgetattr failed.");
        }

        ::cfmakeraw(&tio);

        tio.c_cflag |= CLOCAL | CREAD;
        tio.c_cflag &= ~PARENB;
        tio.c_cflag &= ~CSTOPB;
        tio.c_cflag &= ~CSIZE;
        tio.c_cflag |= CS8;

        if (::cfsetispeed(&tio, DYNAMIXEL_BAUDRATE) != 0 ||
            ::cfsetospeed(&tio, DYNAMIXEL_BAUDRATE) != 0) {
            throw std::runtime_error("[DynamixelMaster::initialize] Failed to set baudrate.");
        }

        tio.c_cc[VMIN] = 0;
        tio.c_cc[VTIME] = 0;

        if (::tcflush(fd_, TCIOFLUSH) != 0) {
            throw std::runtime_error("[DynamixelMaster::initialize] Failed to flush serial port.");
        }
        if (::tcsetattr(fd_, TCSANOW, &tio) != 0) {
            throw std::runtime_error("[DynamixelMaster::initialize] Failed to configure serial port.");
        }

        for (uint8_t i = 0; i < n_ids_; ++i) {
            const uint8_t id = ids_[i];
            auto& driver = *drivers_.at(id);

            uint8_t data[64];
            data[0] = static_cast<uint8_t>(ADDR_TORQUE_ENABLE);
            data[1] = static_cast<uint8_t>(ADDR_TORQUE_ENABLE >> 8);
            const std::size_t n = driver.enable(data + 2, sizeof(data) - 2);

            uint8_t tx[MAX_PACKET_SIZE]{};
            const std::size_t tx_size = make_packet(tx, sizeof(tx), id, INST_WRITE, data, n + 2);
            ::tcflush(fd_, TCIOFLUSH);
            send_packet(tx, tx_size);

            uint8_t rx[MAX_PACKET_SIZE]{};
            receive_packet(rx, sizeof(rx));
        }
    }

    void shutdown() override {
        uint8_t data[4 + motor_interface::MAX_MOTORS * 2]{};
        data[0] = static_cast<uint8_t>(ADDR_TORQUE_ENABLE);
        data[1] = static_cast<uint8_t>(ADDR_TORQUE_ENABLE >> 8);
        data[2] = 1;
        data[3] = 0;

        std::size_t n = 4;
        for (uint8_t i = 0; i < n_ids_; ++i) {
            uint8_t payload[8]{};
            const std::size_t k = drivers_.at(ids_[i])->disable(payload, sizeof(payload));
            data[n++] = ids_[i];
            std::memcpy(data + n, payload, k);
            n += k;
        }

        uint8_t tx[MAX_PACKET_SIZE]{};
        const std::size_t tx_size = make_packet(tx, sizeof(tx), BROADCAST_ID, INST_SYNC_WRITE, data, n);
        ::tcflush(fd_, TCIOFLUSH);
        send_packet(tx, tx_size);
    }

    virtual void update(const motor_interface::motor_command_t* commands, motor_interface::motor_state_t* status) override {
        uint8_t data[4 + motor_interface::MAX_MOTORS * (1 + TX_DATA_SIZE)]{};
        data[0] = static_cast<uint8_t>(ADDR_GOAL_POSITION);
        data[1] = static_cast<uint8_t>(ADDR_GOAL_POSITION >> 8);
        data[2] = static_cast<uint8_t>(TX_DATA_SIZE);
        data[3] = static_cast<uint8_t>(TX_DATA_SIZE >> 8);

        std::size_t n = 4;
        for (uint8_t i = 0; i < n_ids_; ++i) {
            uint8_t payload[TX_DATA_SIZE]{};
            drivers_.at(ids_[i])->encode(commands[i], payload, sizeof(payload));
            data[n++] = ids_[i];
            std::memcpy(data + n, payload, TX_DATA_SIZE);
            n += TX_DATA_SIZE;
        }

        uint8_t tx[MAX_PACKET_SIZE]{};
        std::size_t tx_size = make_packet(tx, sizeof(tx), BROADCAST_ID, INST_SYNC_WRITE, data, n);
        ::tcflush(fd_, TCIOFLUSH);
        send_packet(tx, tx_size);

        data[0] = static_cast<uint8_t>(ADDR_PRESENT_CURRENT);
        data[1] = static_cast<uint8_t>(ADDR_PRESENT_CURRENT >> 8);
        data[2] = static_cast<uint8_t>(RX_DATA_SIZE);
        data[3] = static_cast<uint8_t>(RX_DATA_SIZE >> 8);
        std::memcpy(data + 4, ids_, n_ids_);

        tx_size = make_packet(tx, sizeof(tx), BROADCAST_ID, INST_SYNC_READ, data, 4 + n_ids_);
        ::tcflush(fd_, TCIOFLUSH);
        send_packet(tx, tx_size);

        for (uint8_t i = 0; i < n_ids_; ++i) {
            uint8_t rx[MAX_PACKET_SIZE]{};
            receive_packet(rx, sizeof(rx));

            const uint8_t id = rx[4];
            const uint8_t error = rx[8];

            uint8_t idx = i;
            for (uint8_t k = 0; k < n_ids_; ++k) {
                if (ids_[k] == id) {
                    idx = k;
                    break;
                }
            }

            drivers_.at(id)->decode(rx + 9, RX_DATA_SIZE, status[idx]);
            status[idx].error = error;
        }
    }

private:
    std::size_t make_packet(uint8_t* packet, std::size_t max_packet_size, uint8_t id, uint8_t inst, const uint8_t* data, std::size_t data_size) {
        const uint16_t length = static_cast<uint16_t>(data_size + 3);
        const std::size_t total = 10 + data_size;
        if (total > max_packet_size) {
            throw std::runtime_error("[DynamixelMaster::make_packet] packet too large.");
        }

        packet[0] = 0xFF;
        packet[1] = 0xFF;
        packet[2] = 0xFD;
        packet[3] = 0x00;
        packet[4] = id;
        packet[5] = static_cast<uint8_t>(length);
        packet[6] = static_cast<uint8_t>(length >> 8);
        packet[7] = inst;
        
        std::memcpy(packet + 8, data, data_size);

        const uint16_t crc = crc_ccitt(packet, total - 2);
        packet[total - 2] = static_cast<uint8_t>(crc);
        packet[total - 1] = static_cast<uint8_t>(crc >> 8);
        return total;
    }

    void send_packet(const uint8_t* tx, std::size_t size) {
        std::size_t total = 0;
        const auto timeout = std::chrono::steady_clock::now() + DYNAMIXEL_TIMEOUT;

        while (total < size) {
            if (std::chrono::steady_clock::now() >= timeout) {
                throw std::runtime_error("[DynamixelMaster::send_packet] timeout.");
            }

            const ssize_t written = ::write(fd_, tx + total, size - total);
            if (written > 0) {
                total += static_cast<std::size_t>(written);
                continue;
            }

            if (written < 0 && (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK)) continue;

            throw std::runtime_error("[DynamixelMaster::send_packet] Failed to write serial data.");
        }
    }

    std::size_t receive_packet(uint8_t* rx, std::size_t size) {
        std::size_t total = 0;
        std::size_t wait_length = 11;
        const auto timeout = std::chrono::steady_clock::now() + DYNAMIXEL_TIMEOUT;

        while (true) {
            while (total < wait_length) {
                const auto now = std::chrono::steady_clock::now();
                if (now >= timeout) {
                    throw std::runtime_error("[DynamixelMaster::receive_packet] timeout.");
                }

                const auto remaining = std::chrono::duration_cast<std::chrono::nanoseconds>(timeout - now);
                timespec ts{};
                ts.tv_sec = static_cast<time_t>(remaining.count() / 1'000'000'000LL);
                ts.tv_nsec = static_cast<long>(remaining.count() % 1'000'000'000LL);

                pollfd pfd{};
                pfd.fd = fd_;
                pfd.events = POLLIN;

                const int ret = ::ppoll(&pfd, 1, &ts, nullptr);
                if (ret < 0) {
                    if (errno == EINTR) continue;
                    throw std::runtime_error("[DynamixelMaster::receive_packet] ppoll failed.");
                } else if (ret == 0) {
                    throw std::runtime_error("[DynamixelMaster::receive_packet] timeout.");
                }
                
                if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) {
                    throw std::runtime_error("[DynamixelMaster::receive_packet] serial error.");
                }

                const ssize_t received = ::read(fd_, rx + total, wait_length - total);
                if (received > 0) {
                    total += static_cast<std::size_t>(received);
                    continue;
                } else if (received < 0) {
                    if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) continue;
                    throw std::runtime_error("[DynamixelMaster::receive_packet] read failed.");
                }
            }

            std::size_t idx = 0;
            for (; idx + 3 < total; ++idx) {
                if (rx[idx] == 0xFF && rx[idx + 1] == 0xFF && rx[idx + 2] == 0xFD && rx[idx + 3] != 0xFD) break;
            }

            if (idx != 0) {
                if (idx + 3 >= total) {
                    throw std::runtime_error("[DynamixelMaster::receive_packet] header not found.");
                }

                std::memmove(rx, rx + idx, total - idx);
                total -= idx;
                continue;
            }

            const uint16_t length = static_cast<uint16_t>(rx[5] | (rx[6] << 8));
            if (rx[3] != 0x00 || rx[7] != INST_STATUS || static_cast<std::size_t>(length) + 7 > size) {
                std::memmove(rx, rx + 1, total - 1);
                total -= 1;
                wait_length = 11;
                continue;
            }

            if (wait_length != static_cast<std::size_t>(length) + 7) {
                wait_length = static_cast<std::size_t>(length) + 7;
                continue;
            }

            const uint16_t crc = static_cast<uint16_t>(rx[wait_length - 2] | (rx[wait_length - 1] << 8));
            if (crc_ccitt(rx, wait_length - 2) != crc) {
                throw std::runtime_error("[DynamixelMaster::receive_packet] CRC mismatch.");
            }

            return wait_length;
        }
    }

    static uint16_t crc_ccitt(const uint8_t* data, std::size_t n) {
        static const uint16_t table[256] = {
            0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
            0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
            0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
            0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
            0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
            0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
            0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
            0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
            0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
            0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
            0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
            0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
            0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
            0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
            0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
            0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
            0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
            0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
            0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
            0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
            0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
            0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
            0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
            0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
            0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
            0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
            0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
            0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
            0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
            0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
            0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
            0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202
        };

        uint16_t crc = 0;
        for (std::size_t i = 0; i < n; ++i) {
            const uint16_t idx = static_cast<uint16_t>((crc >> 8) ^ data[i]) & 0xFF;
            crc = static_cast<uint16_t>(crc << 8) ^ table[idx];
        }

        return crc;
    }

    std::string device_;

    int fd_{-1};
};

} // namespace dynamixel

#endif // MOTOR_MASTER_DYNAMIXEL_MASTER_HPP_
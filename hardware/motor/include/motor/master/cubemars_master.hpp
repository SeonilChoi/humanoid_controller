#ifndef MOTOR_MASTER_CUBEMARS_MASTER_HPP_
#define MOTOR_MASTER_CUBEMARS_MASTER_HPP_

#include <string>
#include <cstdint>
#include <cstring>
#include <stdexcept>

#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#include "motor/core/motor_master.hpp"
#include "motor/driver/cubemars_driver.hpp"

namespace cubemars {

class CubemarsMaster : public motor_interface::MotorMaster {
public:
    CubemarsMaster(uint32_t period, const std::string& device)
    : motor_interface::MotorMaster(period), device_(device) {}

    virtual ~CubemarsMaster() {
        if (socket_ >= 0) {
            ::close(socket_);
            socket_ = -1;
        }
    }

    void add_motor(uint8_t id, double gear_ratio, double zero_offset, uint32_t pulse_per_revolution) override {
        drivers_[id] = std::make_unique<cubemars::CubemarsDriver>(id, gear_ratio, zero_offset, pulse_per_revolution);
    }

    void initialize() override {
        socket_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);

        if (socket_ < 0) throw std::runtime_error("[CubemarsMaster::initialize] Failed to create CAN socket.");

        ifreq ifr{};
        std::strncpy(ifr.ifr_name, device_.c_str(), IFNAMSIZ - 1);

        if (::ioctl(socket_, SIOCGIFINDEX, &ifr) < 0) {
            throw std::runtime_error("[CubemarsMaster::initialize] Failed to get interface index.");
        }

        sockaddr_can addr{};
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;

        if (::bind(socket_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
            throw std::runtime_error("[CubemarsMaster::initialize] Failed to bind CAN socket.");
        }

        for (auto& [id, driver] : drivers_) {
            uint8_t buffer[TX_PACKET_SIZE]{};

            const std::size_t size = driver->enable(buffer, sizeof(buffer));

            can_frame frame{};
            frame.can_id = id;
            frame.can_dlc = static_cast<__u8>(size);

            std::memcpy(frame.data, buffer, size);

            send_frame(frame);
        }
    }

    void update(uint8_t id, const motor_interface::motor_command_t& command, motor_interface::motor_state_t& status) override {
        auto& driver = *drivers_.at(id);

        uint8_t tx_buffer[TX_PACKET_SIZE]{};

        const std::size_t tx_size = driver.encode(command, tx_buffer, sizeof(tx_buffer));

        can_frame tx{};
        tx.can_id = id;
        tx.can_dlc = static_cast<__u8>(tx_size);
        
        std::memcpy(tx.data, tx_buffer, tx_size);
        send_frame(tx);

        can_frame rx{};
        receive_frame(id, rx);

        driver.decode(rx.data, rx.can_dlc, status);
    }

    void shutdown() override {
        for (auto& [id, driver] : drivers_) {
            uint8_t buffer[TX_PACKET_SIZE]{};

            const std::size_t size = driver->disable(buffer, sizeof(buffer));

            can_frame frame{};
            frame.can_id = id;
            frame.can_dlc = static_cast<__u8>(size);

            std::memcpy(frame.data, buffer, size);

            send_frame(frame);
        }
    }

private:
    void send_frame(const can_frame& frame) {
        const ssize_t written = ::write(socket_, &frame, sizeof(frame));
        if (written != sizeof(frame)) throw std::runtime_error("[CubemarsMaster::send_frame] Failed to send CAN frame.");
    }

    void receive_frame(uint8_t id, can_frame& frame) {
        while (true) {
            const ssize_t received = ::read(socket_, &frame, sizeof(frame));

            if (received < 0) {
                if (errno == EINTR) continue;
                throw std::runtime_error("[Cubemars::receive_frame] read failed.");
            }

            if (received != sizeof(frame)) continue;

            const uint32_t received_id = frame.can_id & CAN_SFF_MASK;
            if (received_id == id) return;
        }
    }

    std::string device_;

    int socket_{-1};
};

} // namespace cubemars

#endif // MOTOR_MASTER_CUBEMARS_MASTER_HPP_
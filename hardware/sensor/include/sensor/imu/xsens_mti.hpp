#ifndef SENSOR_IMU_XSENS_MTI_HPP_
#define SENSOR_IMU_XSENS_MTI_HPP_

#include <mutex>
#include <string>
#include <cstdint>

#include "sensor/core/imu.hpp"

#include <xstypes/xsdatapacket.h>
#include <xscontroller/xsscanner.h>
#include <xscontroller/xscallback.h>
#include <xscontroller/xsdevice_public.h>
#include <xscontroller/xscontrol_public.h>

namespace xsens_mti {

class XsensCallback : public XsCallback {
public:
    XsensCallback() = default;

    ~XsensCallback() override = default;

    bool read(XsDataPacket& packet) {
        std::lock_guard<std::mutex> lock(mutex_);

        if (!available_) return false;

        packet = packet_;
        available_ = false;

        return true;
    }

protected:
    void onLiveDataAvailable(XsDevice*, const XsDataPacket* packet) override {
        if (packet == nullptr) return;

        std::lock_guard<std::mutex> lock(mutex_);

        packet_ = *packet;
        available_ = true;
    }

private:
    std::mutex mutex_;

    XsDataPacket packet_{};

    bool available_{false};
};


class XsensMti : public sensor_interface::Imu {
public:
    XsensMti(uint32_t period, const std::string& device, uint32_t baudrate)
    : sensor_interface::Imu(period, device, baudrate) {}

    ~XsensMti() override;

    void initialize() override;

    void update() override;

    void shutdown() override;

    void read(sensor_interface::imu_data_t& data) override;

private:
    XsensCallback callback_;

    XsControl* control_{nullptr};

    XsDevice* device_handle_{nullptr};

    sensor_interface::imu_data_t data_{};

    std::mutex mutex_;

    bool initialized_{false};
};

} // namespace imu

#endif // SENSOR_IMU_XSENS_MTI_HPP_
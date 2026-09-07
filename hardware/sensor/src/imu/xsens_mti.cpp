#include <iostream>
#include <stdexcept>

#include "sensor/imu/xsens_mti.hpp"

#include <xscommon/journaller.h>

Journaller* gJournal = nullptr;

xsens_mti::XsensMti::~XsensMti() {
    shutdown();
}

void xsens_mti::XsensMti::initialize() {
    if (initialized_) return;

    control_ = XsControl::construct();

    if (control_ == nullptr) throw std::runtime_error("[XsensMti::initialize] Failed to construct XsControl.");

    XsPortInfo port_info = XsScanner::scanPort(XsString(device_), static_cast<XsBaudRate>(baudrate_));

    if (port_info.empty()) {
        control_->destruct();
        control_ = nullptr;

        throw std::runtime_error("[XsensMti::initialize] xsens device not found.");
    }

    if (!control_->openPort(port_info.portName(), port_info.baudrate())) {
        control_->destruct();
        control_ = nullptr;

        throw std::runtime_error("[XsensMti::initialize] Failed to open port.");    
    }

    device_handle_ = control_->device(port_info.deviceId());

    if (device_handle_ == nullptr) {
        control_->close();
        control_->destruct();
        control_ = nullptr;

        throw std::runtime_error("[XsensMti::initialize] Failed to get device handle.");
    }

    device_handle_->addCallbackHandler(&callback_);

    if (!device_handle_->gotoMeasurement()) {
        device_handle_->removeCallbackHandler(&callback_);
        device_handle_ = nullptr;

        control_->close();
        control_->destruct();
        control_ = nullptr;

        throw std::runtime_error("[XsensMti::initialize] Failed to enter mesurement mode.");
    }

    initialized_ = true;
}

void xsens_mti::XsensMti::update() {
    if (!initialized_) return;

    XsDataPacket packet;

    if (!callback_.read(packet)) return;

    sensor_interface::imu_data_t data{};

    if (packet.containsOrientation()) {
        const XsQuaternion quaternion = packet.orientationQuaternion();

        data.orientation[0] = quaternion.w();
        data.orientation[1] = quaternion.x();
        data.orientation[2] = quaternion.y();
        data.orientation[3] = quaternion.z();
    }

    if (packet.containsCalibratedGyroscopeData()) {
        const XsVector angular_velocity = packet.calibratedGyroscopeData();

        data.angular_velocity[0] = angular_velocity[0];
        data.angular_velocity[1] = angular_velocity[1];
        data.angular_velocity[2] = angular_velocity[2];
    }

    if (packet.containsCalibratedAcceleration()) {
        const XsVector linear_acceleration = packet.calibratedAcceleration();

        data.linear_acceleration[0] = linear_acceleration[0];
        data.linear_acceleration[1] = linear_acceleration[1];
        data.linear_acceleration[2] = linear_acceleration[2];
    }

    if (packet.containsTemperature()) {
        data.temperature = packet.temperature();
    }

    {
        std::lock_guard<std::mutex> lock(mutex_);
        data_ = data;
    }
}

void xsens_mti::XsensMti::read(sensor_interface::imu_data_t& data) {
    std::lock_guard<std::mutex> lock(mutex_);
    data = data_;
}

void xsens_mti::XsensMti::shutdown() {
    if (!initialized_ && control_ == nullptr) return;

    if (device_handle_ != nullptr) {
        device_handle_->removeCallbackHandler(&callback_);
        device_handle_ = nullptr;
    }

    if (control_ != nullptr) {
        control_->close();
        control_->destruct();
        control_ = nullptr;
    }

    initialized_ = false;
}
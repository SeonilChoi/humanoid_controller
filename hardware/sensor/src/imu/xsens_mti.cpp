#include <stdexcept>

#include "sensor/imu/xsens_mti.hpp"

#include <xstypes/xsbaudrate.h>
#include <xstypes/xsdataidentifier.h>
#include <xstypes/xsoutputconfigurationarray.h>
#include <xscommon/journaller.h>

Journaller* gJournal = nullptr;

XsBaudRate to_xsbaudrate(uint32_t baudrate) {
    switch (baudrate) {
	case 19200:
	    return XBR_19k2;
	case 38400:
	    return XBR_38k4;
	case 57600:
	    return XBR_57k6;
	case 115200:
	    return XBR_115k2;
	case 230400:
	    return XBR_230k4;
	case 460800:
	    return XBR_460k8;
	case 921600:
	    return XBR_921k6;
	default:
	    throw std::runtime_error("[to_xsbaudrate] Invalied baudrate.");
    }
}

xsens_mti::XsensMti::~XsensMti() {
    shutdown();
}

void xsens_mti::XsensMti::initialize() {
    if (initialized_) return;

    control_ = XsControl::construct();

    if (control_ == nullptr) throw std::runtime_error("[XsensMti::initialize] Failed to construct XsControl.");

    XsPortInfo port_info = XsScanner::scanPort(XsString(device_), to_xsbaudrate(baudrate_));


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

    if (!device_handle_->gotoConfig()) {
        device_handle_ = nullptr;

        control_->close();
        control_->destruct();
        control_ = nullptr;

        throw std::runtime_error("[XsensMti::initialize] Failed to enter config mode.");
    }

    constexpr uint16_t kMaxFrequency = 0xFFFF;

    XsOutputConfigurationArray output_config;
    output_config.push_back(XsOutputConfiguration(XDI_Quaternion, kMaxFrequency));
    output_config.push_back(XsOutputConfiguration(XDI_RateOfTurn, kMaxFrequency));
    output_config.push_back(XsOutputConfiguration(XDI_Acceleration, kMaxFrequency));

    if (!device_handle_->setOutputConfiguration(output_config)) {
        device_handle_ = nullptr;

        control_->close();
        control_->destruct();
        control_ = nullptr;

        throw std::runtime_error("[XsensMti::initialize] Failed to set output configuration.");
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

    sensor_interface::imu_data_t data;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        data = data_;
    }

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

#pragma once

#include "config/config.hh"
#include "rclcpp/rclcpp.hpp"
#include <LPMS-NAV3/LpmsIG1.hh>
#include <LPMS-NAV3/SensorData.hh>
#include <atomic>
#include <iostream>
#include <memory>
#include <ostream>
#include <rclcpp/node.hpp>
#include <spdlog/spdlog.h>
#include <string>

namespace fan {
class Visual : public rclcpp::Node {
public:
    explicit Visual()
        : rclcpp::Node("visual") {
        spdlog::info("Visual initializing...");
        ig1_ = std::make_unique<IG1>();
        ig1_->init();
        const auto port = fan::node({"serialPort"}).as<std::string>();
        const auto baudRate = fan::node({ "baudRate"}).as<int>();
        if (ig1_->connect(port, baudRate)) {
            spdlog::info("Connected to IMU device");
            if (ig1_->hasInfo()) {
                IG1Info info;
                ig1_->getInfo(info);
                spdlog::info("IMU deviceName: {}", info.deviceName);
                spdlog::info("IMU firmware: {}", info.firmwareInfo);
                spdlog::info("IMU filterVersion: {}", info.filterVersion);
                spdlog::info("IMU iapCheckStatus: {}", info.iapCheckStatus);
                spdlog::info("IMU serialNumber: {}", info.serialNumber);
            }
        } else
            spdlog::error("Failed to connect to IMU device");
        spdlog::info("Visual initialized");
    };

    void start() {
        running_.store(true, std::memory_order::relaxed);
        while (running_.load(std::memory_order::relaxed)) {
            if (ig1_->hasImuData()) {
                IG1ImuData imuData;
                if (ig1_->getImuData(imuData)) {
                    spdlog::info(
                        "IMU data: {} {} {}", imuData.euler.data[0], imuData.euler.data[1],
                        imuData.euler.data[2]);
                }
            } else {
                spdlog::info("No IMU data");
            }
            usleep(100000);
        }
        ig1_->disconnect();
    }

    ~Visual() {
        if (running_.load(std::memory_order::relaxed)) {
            running_.store(false, std::memory_order::relaxed);
        }
    }

private:
    std::atomic<bool> running_{false};
    std::unique_ptr<IG1> ig1_;
};
} // namespace fan
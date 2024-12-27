#include "config/config.hh"
// #include "visual/visual.hh"
#include "imunode/imunode.hh"
#include <atomic>
#include <csignal>
#include <filesystem>
#include <memory>
#include <rclcpp/executors.hpp>
#include <rclcpp/utilities.hpp>
#include <string>

int main(int argc, char** argv) {
    static std::atomic<bool> running(true);
    std::signal(SIGEV_SIGNAL, [](int sig) { running = false; });

    auto configPath = std::filesystem::canonical("/proc/self/exe")
                          .parent_path()
                          .parent_path()
                          .parent_path()
                          .string();
    configPath += "/include/config/config.yaml";
    config::load(configPath);

    rclcpp::init(argc, argv);

    const auto port          = config::value<std::string>({"serialPort"}, "/dev/ttyUSB0");
    const auto baudRate      = config::value<int>({"baudRate"}, 115200);
    const auto autoReconnect = config::value<bool>({"autoReconnect"}, true);
    const auto frameId       = config::value<std::string>({"frameId"}, "imu");
    const auto updateRate    = config::value<int>({"updateRate"}, 200);
    auto imu =
        std::make_shared<imu::LpNAV3Proxy>(port, baudRate, autoReconnect, frameId, updateRate);
    imu->run();
    rclcpp::spin(imu);
    rclcpp::shutdown();

    return 0;
}
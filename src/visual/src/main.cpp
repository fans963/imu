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
#include <thread>

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

    const auto port          = config::node({"serialPort"}).as<std::string>();
    const auto baudRate      = config::node({"baudRate"}).as<int>();
    const auto autoReconnect = config::node({"autoReconnect"}).as<bool>();
    const auto frameId       = config::node({"frameId"}).as<std::string>();
    const auto updateRate    = config::node({"updateRate"}).as<int>();
    auto imu =
        std::make_shared<imu::LpNAV3Proxy>(port, baudRate, autoReconnect, frameId, updateRate);
    imu->run();
    rclcpp::spin(imu);
    rclcpp::shutdown();

    return 0;
}
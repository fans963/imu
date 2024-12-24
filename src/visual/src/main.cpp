#include "config/config.hh"
// #include "visual/visual.hh"
#include <atomic>
#include <csignal>
#include "visual/visual.hh"
#include <filesystem>
#include <memory>
#include "LPMS-NAV3/LpmsIG1.hh"
#include <rclcpp/executors.hpp>
#include <rclcpp/utilities.hpp>
#include <thread>

int main(int argc, char **argv) {
    static std::atomic<bool> running(true);
    std::signal(SIGEV_SIGNAL, [](int sig) { running = false; });

    auto configPath = std::filesystem::canonical("/proc/self/exe")
                          .parent_path()
                          .parent_path()
                          .parent_path()
                          .string();
    configPath += "/include/config/config.yaml";
    fan::load(configPath);

    rclcpp::init(argc, argv);
    auto visualNode = std::make_shared<fan::Visual>();
    std::thread visualThread([=]() { visualNode->start(); });
    visualThread.detach();

    rclcpp::spin(visualNode);
    rclcpp::shutdown();

    return 0;
}
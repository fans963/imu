#pragma once

#include "config/config.hh"
#include "rclcpp/rclcpp.hpp"
#include <atomic>
#include <memory>
#include <rclcpp/node.hpp>

namespace visual {
class Visual : public rclcpp::Node {
public:
    explicit Visual()
        : rclcpp::Node("visual") {
    };

    ~Visual() {
        if (running_.load(std::memory_order::relaxed)) {
            running_.store(false, std::memory_order::relaxed);
        }
    }

private:
    std::atomic<bool> running_{false};
};
} // namespace visual
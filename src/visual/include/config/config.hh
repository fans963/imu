#pragma once

#include "spdlog/spdlog.h"
#include "yaml-cpp/yaml.h"
#include <cstddef>
#include <iostream>
#include <memory>
#include <mutex>
#include <ostream>
#include <thread>
#include <yaml-cpp/node/node.h>

namespace config {
static std::unique_ptr<YAML::Node> root_ = std::make_unique<YAML::Node>();
static std::mutex mutex_;

static inline void load(const std::string& configFile) {
    std::lock_guard<std::mutex> locker(mutex_);
    *root_ = YAML::LoadFile(configFile);
    spdlog::info("Config file loaded from {}", configFile);
}

template <size_t N>
static inline auto node(const char* (&&keys)[N]) {
    std::lock_guard<std::mutex> locker(mutex_);
    auto node = YAML::Clone(*root_);
    for (const auto& key : keys) {
        node = node[key];
    }
    return node;
}

} // namespace fan
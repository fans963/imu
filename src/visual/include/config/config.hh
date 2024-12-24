#pragma once

#include "spdlog/spdlog.h"
#include "yaml-cpp/yaml.h"
#include <cstddef>
#include <iostream>
#include <memory>
#include <ostream>
#include <thread>
#include <yaml-cpp/node/node.h>

namespace fan {
static YAML::Node root_;

static inline void load(const std::string& configFile) {
    root_ = YAML::LoadFile(configFile);
    spdlog::info("Config file loaded from {}", configFile);
}

template <size_t N>
static inline auto node(const char* (&&keys)[N]) {
    auto node = YAML::Clone(root_);
    for (const auto& key : keys) {
        node = node[key];
    }
    return node;
}

} // namespace fan
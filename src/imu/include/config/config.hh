#pragma once

#include "spdlog/spdlog.h"
#include "yaml-cpp/yaml.h"
#include <cstddef>
#include <yaml-cpp/node/node.h>

namespace config {

static YAML::Node config_;

static inline void load(const std::string& path) {
    if ((config_ = YAML::LoadFile(path)))
        spdlog::info("Config file load successfully");
}

template <size_t N>
static inline YAML::Node node(const char* (&&key)[N]) {
    auto node = config_;
    for (size_t i = 0; i < N; ++i) {
        node = node[key[i]];
    }
    return node;
}

} // namespace config
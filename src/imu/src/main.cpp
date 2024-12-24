
#include "config/config.hh"
#include "LPMS-NAV3/SerialPort.hh"
#include "LPMS-NAV3/LpmsIG1.hh"
#include <filesystem>
#include <iostream>

int main() {
    auto configPath =
        std::filesystem::canonical("/proc/self/exe").parent_path().parent_path().parent_path().string();
    configPath += "/include/config/config.yaml";
    config::load(configPath);

    IG1 ig1_;
    ig1_.init();
    ig1_.connect("/dev/ttyUSB0", 115200);

    return 0;
}
//
// Created by tobias on 9/14/25.
// Split from Types.h during architecture restructuring.
//
#pragma once

#include <cstdint>
#include <chrono>
#include <string>

namespace wombat
{
    struct Configuration
    {
        struct Spi
        {
            std::string devicePath{"/dev/spidev0.0"};
            uint32_t speedHz{20'000'000};
            uint8_t mode{0};
            uint8_t bitsPerWord{8};
            uint8_t maxRetryAttempts{3};
            uint8_t protocolVersion{1};
        } spi;

        struct Logging
        {
            enum class Level { Debug, Info, Warn, Error };

            Level logLevel{Level::Info};
            std::string pattern{"[%H:%M:%S.%e] [%l] %v"};
        } logging;

        std::chrono::milliseconds mainLoopDelay{1};
    };
}

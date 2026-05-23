#pragma once

#include "wombat/core/Result.h"
#include "wombat/core/Logger.h"
#include "wombat/core/Configuration.h"
#include <chrono>
#include <memory>
#include <string>
#include <optional>

namespace wombat
{
    class UartMonitor
    {
    public:
        UartMonitor(std::shared_ptr<Logger> logger, const Configuration::Uart& config);

        Result<void> initialize();
        Result<void> processUpdate();
        Result<void> shutdown();

        // Pump UART output for the given duration, logging all received lines.
        Result<void> drainFor(std::chrono::milliseconds duration);

        void noteLoopTime(std::chrono::steady_clock::time_point now);
        bool heartbeatEverSeen() const;
        std::optional<std::chrono::steady_clock::time_point> lastHeartbeatTime() const;

    private:
        void processLine(const std::string& line);

        std::shared_ptr<Logger> logger_;
        Configuration::Uart config_;

        int fd_{-1};
        bool isOpen_{false};
        std::string lineBuffer_;
        std::chrono::steady_clock::time_point lastLoopTime_{};
        std::optional<std::chrono::steady_clock::time_point> lastHeartbeatTime_{};
    };
}

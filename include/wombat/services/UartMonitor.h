#pragma once

#include "wombat/core/Result.h"
#include "wombat/core/Logger.h"
#include "wombat/core/Configuration.h"
#include <memory>
#include <string>

namespace wombat
{
    class UartMonitor
    {
    public:
        UartMonitor(std::shared_ptr<Logger> logger, const Configuration::Uart& config);

        Result<void> initialize();
        Result<void> processUpdate();
        Result<void> shutdown();

    private:
        std::shared_ptr<Logger> logger_;
        Configuration::Uart config_;

        int fd_{-1};
        bool isOpen_{false};
        std::string lineBuffer_;
    };
}
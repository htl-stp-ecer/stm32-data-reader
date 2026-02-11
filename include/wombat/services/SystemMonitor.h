#pragma once

#include "wombat/core/Result.h"
#include "wombat/core/Logger.h"
#include "wombat/messaging/LcmBroker.h"
#include <memory>
#include <chrono>

namespace wombat
{
    class SystemMonitor
    {
    public:
        SystemMonitor(std::shared_ptr<LcmBroker> broker, std::shared_ptr<Logger> logger);

        Result<void> updateCpuTemperature(std::chrono::milliseconds publishInterval = std::chrono::milliseconds(1000));

    private:
        std::shared_ptr<LcmBroker> broker_;
        std::shared_ptr<Logger> logger_;

        std::chrono::steady_clock::time_point lastCpuTempPublishTime_{std::chrono::steady_clock::now()};
        float lastPublishedCpuTemperature_{0.0f};

        Result<float> readCpuTemperature();
        Result<void> publishCpuTemperature(float temperature);
    };
}
//
// Created by tobias on 9/14/25.
//
#pragma once

#include "wombat/core/DeviceTypes.h"
#include "wombat/core/Configuration.h"
#include "wombat/core/Result.h"
#include "wombat/core/Logger.h"
#include "wombat/messaging/LcmBroker.h"
#include "wombat/services/DeviceController.h"
#include "wombat/services/DataPublisher.h"
#include "wombat/services/CommandSubscriber.h"
#include "wombat/services/SystemMonitor.h"
#include "wombat/services/UartMonitor.h"
#include <memory>
#include <atomic>

namespace wombat
{
    class Application
    {
    public:
        explicit Application(const Configuration& config);
        ~Application();

        // Non-copyable, non-movable
        Application(const Application&) = delete;
        Application& operator=(const Application&) = delete;

        Result<void> initialize();
        Result<void> run();
        Result<void> shutdown();

        void requestShutdown();

        std::shared_ptr<Logger> getLogger() const { return logger_; }

    private:
        const Configuration config_;

        // Core services
        std::shared_ptr<Logger> logger_;
        std::shared_ptr<LcmBroker> messageBroker_;
        std::shared_ptr<DeviceController> deviceController_;
        std::shared_ptr<DataPublisher> dataPublisher_;
        std::shared_ptr<CommandSubscriber> commandSubscriber_;
        std::unique_ptr<SystemMonitor> systemMonitor_;
        std::unique_ptr<UartMonitor> uartMonitor_;

        std::atomic<bool> shouldShutdown_{false};
        bool isInitialized_{false};

        Result<void> createServices();
        Result<void> initializeServices();
        Result<void> shutdownServices();

        Result<void> processMainLoop();
        Result<void> publishCurrentData();

        Timestamp lastPublishedTimestamp_{0};

        // BEMF change detection for timing instrumentation
        int32_t lastBemf_[4]{0, 0, 0, 0};
    };
} // namespace wombat
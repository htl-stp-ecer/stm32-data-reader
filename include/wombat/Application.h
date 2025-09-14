//
// Created by tobias on 9/14/25.
//
#pragma once

#include "wombat/core/Types.h"
#include "wombat/core/Result.h"
#include "wombat/core/Logger.h"
#include "wombat/messaging/LcmBroker.h"
#include "wombat/services/DeviceController.h"
#include "wombat/services/DataPublisher.h"
#include "wombat/services/CommandSubscriber.h"
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

        std::atomic<bool> shouldShutdown_{false};
        bool isInitialized_{false};

        Result<void> createServices();
        Result<void> initializeServices();
        Result<void> shutdownServices();

        Result<void> processMainLoop();
        Result<void> publishCurrentData();

        Timestamp lastPublishedTimestamp_{0};
    };
} // namespace wombat

#pragma once

#include "wombat/core/Result.h"
#include "wombat/core/Logger.h"
#include <raccoon/Concepts.h>
#include <raccoon/Options.h>
#include <memory>
#include <functional>

namespace wombat
{
    using raccoon::LcmMessage;

    class LcmBroker
    {
    public:
        explicit LcmBroker(std::shared_ptr<Logger> logger);
        ~LcmBroker();

        Result<void> initialize();
        Result<void> shutdown();
        Result<void> processMessages();
        [[nodiscard]] bool isHealthy() const;

        template <LcmMessage T>
        Result<void> publish(const std::string& channel, const T& message);

        template <LcmMessage T>
        Result<void> publishForce(const std::string& channel, const T& message);

        template <LcmMessage T>
        Result<void> publishRetained(const std::string& channel, const T& message);

        template <LcmMessage T>
        Result<void> subscribe(const std::string& channel, std::function<void(const T &)> handler,
                               const raccoon::SubscribeOptions& options = {});

    private:
        class Impl;
        std::unique_ptr<Impl> impl_;
    };
}
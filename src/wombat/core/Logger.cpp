//
// Created by tobias on 9/14/25.
//
#include "wombat/core/Logger.h"
#include "wombat/core/Channels.h"
#include "wombat/messaging/LcmBroker.h"
#include <exlcm/string_t.hpp>
#include <spdlog/spdlog.h>
#include <chrono>

namespace wombat
{
    class SpdlogLogger final : public Logger
    {
    public:
        explicit SpdlogLogger(const Configuration::Logging& config)
        {
            spdlog::set_pattern(config.pattern);
            spdlog::set_level(convertLogLevel(config.logLevel));
        }

        explicit SpdlogLogger(const Configuration::Logging& config, std::shared_ptr<LcmBroker> lcmBroker)
            : lcmBroker_{std::move(lcmBroker)}
        {
            spdlog::set_pattern(config.pattern);
            spdlog::set_level(convertLogLevel(config.logLevel));
        }

        void info(const std::string& message) override
        {
            spdlog::info(message);
        }

        void warn(const std::string& message) override
        {
            spdlog::warn(message);
            publishErrorToLcm("[WARN] " + message);
        }

        void error(const std::string& message) override
        {
            spdlog::error(message);
            publishErrorToLcm(message);
        }

        void debug(const std::string& message) override
        {
            spdlog::debug(message);
        }

        void setLcmBroker(std::shared_ptr<LcmBroker> lcmBroker) override
        {
            lcmBroker_ = std::move(lcmBroker);
        }

    private:
        std::shared_ptr<LcmBroker> lcmBroker_;

        void publishErrorToLcm(const std::string& message)
        {
            if (!lcmBroker_)
            {
                return; // No broker available
            }

            exlcm::string_t errorMsg;
            errorMsg.timestamp = std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count();
            errorMsg.value = message;
            lcmBroker_->publish(Channels::ERROR_MESSAGES, errorMsg);
        }

        static spdlog::level::level_enum convertLogLevel(Configuration::Logging::Level level)
        {
            switch (level)
            {
            case Configuration::Logging::Level::Debug: return spdlog::level::debug;
            case Configuration::Logging::Level::Info: return spdlog::level::info;
            case Configuration::Logging::Level::Warn: return spdlog::level::warn;
            case Configuration::Logging::Level::Error: return spdlog::level::err;
            default: return spdlog::level::info;
            }
        }
    };

    std::unique_ptr<Logger> Logger::create(const Configuration::Logging& config)
    {
        return std::make_unique<SpdlogLogger>(config);
    }

    std::unique_ptr<Logger> Logger::create(const Configuration::Logging& config, std::shared_ptr<LcmBroker> lcmBroker)
    {
        return std::make_unique<SpdlogLogger>(config, std::move(lcmBroker));
    }
}
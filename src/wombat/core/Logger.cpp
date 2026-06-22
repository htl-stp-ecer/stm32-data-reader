//
// Created by tobias on 9/14/25.
//
#include "wombat/core/Logger.h"
#include <raccoon/Channels.h>
#include "wombat/messaging/LcmBroker.h"
#include <raccoon/string_t.hpp>
#include <spdlog/spdlog.h>
#include <spdlog/async.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <chrono>
#include <mutex>

namespace wombat
{
    namespace Channels = raccoon::Channels;

    class SpdlogLogger final : public Logger
    {
    public:
        explicit SpdlogLogger(const Configuration::Logging& config)
        {
            ensureAsyncDefaultLogger(config);
        }

        explicit SpdlogLogger(const Configuration::Logging& config, std::shared_ptr<LcmBroker> lcmBroker)
            : lcmBroker_{std::move(lcmBroker)}
        {
            ensureAsyncDefaultLogger(config);
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

            raccoon::string_t errorMsg;
            errorMsg.timestamp = std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count();
            errorMsg.value = message;
            lcmBroker_->publishRetained(Channels::ERROR_MESSAGES, errorMsg);
        }

        // Install an *asynchronous* default logger exactly once. The reader runs
        // under systemd, so spdlog's default synchronous stdout sink writes
        // straight into journald — and a journald back-pressure event (rate
        // limit, or an fsync of /var/log/journal to the SD card) blocks the
        // calling thread. With synchronous logging that caller is the robot's
        // control loop, which is how a single ~500 ms standstill appears with
        // no command reordering. An async logger moves the actual write to a
        // background thread; the control loop only enqueues. ``overrun_oldest``
        // means a full queue drops the oldest line instead of blocking — losing
        // a log line is always better than stalling the robot.
        static void ensureAsyncDefaultLogger(const Configuration::Logging& config)
        {
            static std::once_flag once;
            std::call_once(once, []
            {
                spdlog::init_thread_pool(8192, 1);
                auto sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
                auto logger = std::make_shared<spdlog::async_logger>(
                    "reader", sink, spdlog::thread_pool(),
                    spdlog::async_overflow_policy::overrun_oldest);
                spdlog::set_default_logger(logger);
            });
            spdlog::set_pattern(config.pattern);
            spdlog::set_level(convertLogLevel(config.logLevel));
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
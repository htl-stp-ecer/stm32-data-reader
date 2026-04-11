#include "wombat/messaging/LcmBroker.h"
#include <raccoon/Transport.h>
#include <raccoon/Options.h>
#include <raccoon/vector3f_t.hpp>
#include <raccoon/quaternion_t.hpp>
#include <raccoon/scalar_f_t.hpp>
#include <raccoon/scalar_i32_t.hpp>
#include <raccoon/scalar_i8_t.hpp>
#include <raccoon/string_t.hpp>
#include <raccoon/orientation_matrix_t.hpp>
#include <raccoon/kinematics_config_t.hpp>
#include <chrono>

namespace wombat
{
    class LcmBroker::Impl
    {
    public:
        explicit Impl(std::shared_ptr<Logger> logger)
            : logger_{std::move(logger)}
        {
        }

        Result<void> initialize()
        {
            transport_ = std::make_unique<raccoon::Transport>(raccoon::Transport::create());

            logger_->info("LCM message broker initialized successfully (via raccoon::Transport)");
            return Result<void>::success();
        }

        Result<void> shutdown()
        {
            if (transport_)
            {
                transport_->stop();
                transport_.reset();
            }
            logger_->info("LCM message broker shut down");
            return Result<void>::success();
        }

        Result<void> processMessages()
        {
            if (!transport_)
            {
                return Result<void>::failure("Transport not initialized");
            }

            // Drain all pending messages instead of just one
            int messagesProcessed = 0;
            while (true)
            {
                const int result = transport_->spinOnce(5);
                if (result < 0)
                {
                    return Result<void>::failure("Failed to process LCM messages");
                }
                if (result == 0)
                {
                    break; // No more pending messages
                }
                ++messagesProcessed;
            }

            const auto now = std::chrono::steady_clock::now();
            ++processCallCount_;

            intervalMessagesProcessed_ += messagesProcessed;

            if (lastProcessTime_.time_since_epoch().count() > 0)
            {
                const auto dt = std::chrono::duration<double>(now - lastProcessTime_).count();
                const double hz = (dt > 0.0) ? 1.0 / dt : 0.0;

                // Log every 100 calls to avoid spamming
                if (processCallCount_ % 100 == 0)
                {
                    const double avgMsgsPerCall = static_cast<double>(intervalMessagesProcessed_) / 100.0;
                    const auto stats = transport_->getAndResetStats();
                    std::string latencyStr = "no msgs";
                    if (stats.latency.count > 0)
                    {
                        latencyStr = "min=" + std::to_string(stats.latency.minUs / 1000) + "ms"
                            + " avg=" + std::to_string(stats.latency.avgUs / 1000) + "ms"
                            + " max=" + std::to_string(stats.latency.maxUs / 1000) + "ms"
                            + " (n=" + std::to_string(stats.latency.count) + ")";
                    }
                    std::string dedupStr;
                    if (stats.publishesDeduplicated > 0)
                    {
                        dedupStr = ", dedup=" + std::to_string(stats.publishesDeduplicated);
                    }
                    logger_->info("LCM processMessages: avg=" + std::to_string(avgMsgsPerCall).substr(0, 5)
                        + " msgs/call, rate=" + std::to_string(hz).substr(0, 5) + "Hz"
                        + ", total calls=" + std::to_string(processCallCount_)
                        + ", total msgs=" + std::to_string(totalMessagesProcessed_ + messagesProcessed)
                        + ", latency: " + latencyStr
                        + dedupStr);
                    intervalMessagesProcessed_ = 0;
                }
            }

            totalMessagesProcessed_ += messagesProcessed;
            lastProcessTime_ = now;

            return Result<void>::success();
        }

        bool isHealthy() const
        {
            return transport_ != nullptr;
        }

        template <typename MessageType>
        Result<void> publishIfChanged(const std::string& channel, const MessageType& message,
                                      const raccoon::PublishOptions& extraOptions = {})
        {
            if (!transport_)
            {
                return Result<void>::failure("Transport not initialized");
            }

            raccoon::PublishOptions options = extraOptions;
            options.deduplicate = true;

            if (!transport_->publish(channel, message, options))
            {
                return Result<void>::failure("Failed to publish message on channel: " + channel);
            }

            logger_->debug("Published " + std::string(MessageType::getTypeName())
                + " on channel: " + channel
                + (options.retained ? " (retained, dedup)" : " (dedup)"));
            return Result<void>::success();
        }

        template <typename MessageType>
        Result<void> publishForce(const std::string& channel, const MessageType& message,
                                  const raccoon::PublishOptions& options = {})
        {
            if (!transport_)
            {
                return Result<void>::failure("Transport not initialized");
            }

            if (!transport_->publish(channel, message, options))
            {
                return Result<void>::failure("Failed to publish message on channel: " + channel);
            }

            logger_->debug("Published " + std::string(MessageType::getTypeName())
                + " on channel: " + channel
                + (options.retained ? " (forced, retained)" : " (forced)"));
            return Result<void>::success();
        }

        template <LcmMessage T>
        Result<void> subscribe(const std::string& channel,
                               std::function<void(const T &)> handler,
                               const raccoon::SubscribeOptions& options = {})
        {
            if (!transport_)
            {
                return Result<void>::failure("Transport not initialized");
            }

            transport_->subscribe<T>(channel, std::move(handler), options);

            logger_->debug("Subscribed to " + std::string(T::getTypeName()) + " channel: " + channel);
            return Result<void>::success();
        }

    private:
        std::shared_ptr<Logger> logger_;
        std::unique_ptr<raccoon::Transport> transport_;
        std::chrono::steady_clock::time_point lastProcessTime_{};
        uint64_t processCallCount_{0};
        uint64_t totalMessagesProcessed_{0};
        uint64_t intervalMessagesProcessed_{0};
    };

    // Public interface implementation
    LcmBroker::LcmBroker(std::shared_ptr<Logger> logger)
        : impl_{std::make_unique<Impl>(std::move(logger))}
    {
    }

    LcmBroker::~LcmBroker() = default;

    Result<void> LcmBroker::initialize()
    {
        return impl_->initialize();
    }

    Result<void> LcmBroker::shutdown()
    {
        return impl_->shutdown();
    }

    Result<void> LcmBroker::processMessages()
    {
        return impl_->processMessages();
    }

    bool LcmBroker::isHealthy() const
    {
        return impl_->isHealthy();
    }

    // Explicit template instantiations — publish (change-detected, plain)
    template <>
    Result<void> LcmBroker::publish<raccoon::vector3f_t>(const std::string& ch, const raccoon::vector3f_t& m)
    {
        return impl_->publishIfChanged(ch, m);
    }

    template <>
    Result<void> LcmBroker::publish<raccoon::quaternion_t>(const std::string& ch, const raccoon::quaternion_t& m)
    {
        return impl_->publishIfChanged(ch, m);
    }

    template <>
    Result<void> LcmBroker::publish<raccoon::scalar_f_t>(const std::string& ch, const raccoon::scalar_f_t& m)
    {
        return impl_->publishIfChanged(ch, m);
    }

    template <>
    Result<void> LcmBroker::publish<raccoon::scalar_i32_t>(const std::string& ch, const raccoon::scalar_i32_t& m)
    {
        return impl_->publishIfChanged(ch, m);
    }

    template <>
    Result<void> LcmBroker::publish<raccoon::scalar_i8_t>(const std::string& ch, const raccoon::scalar_i8_t& m)
    {
        return impl_->publishIfChanged(ch, m);
    }

    template <>
    Result<void> LcmBroker::publish<raccoon::string_t>(const std::string& ch, const raccoon::string_t& m)
    {
        return impl_->publishIfChanged(ch, m);
    }

    // Explicit template instantiations — publishForce (plain, no change detection)
    template <>
    Result<void> LcmBroker::publishForce<raccoon::scalar_i8_t>(const std::string& ch, const raccoon::scalar_i8_t& m)
    {
        return impl_->publishForce(ch, m);
    }

    template <>
    Result<void> LcmBroker::publishForce<raccoon::scalar_i32_t>(const std::string& ch, const raccoon::scalar_i32_t& m)
    {
        return impl_->publishForce(ch, m);
    }

    // Explicit template instantiations — publishRetained (change-detected + retained flag)
    static const raccoon::PublishOptions retainedOpts{.retained = true};

    template <>
    Result<void> LcmBroker::publishRetained<raccoon::scalar_i8_t>(const std::string& ch, const raccoon::scalar_i8_t& m)
    {
        return impl_->publishIfChanged(ch, m, retainedOpts);
    }

    template <>
    Result<void> LcmBroker::publishRetained<raccoon::scalar_i32_t>(const std::string& ch,
                                                                   const raccoon::scalar_i32_t& m)
    {
        return impl_->publishIfChanged(ch, m, retainedOpts);
    }

    template <>
    Result<void> LcmBroker::publishRetained<raccoon::scalar_f_t>(const std::string& ch,
                                                                 const raccoon::scalar_f_t& m)
    {
        return impl_->publishIfChanged(ch, m, retainedOpts);
    }

    // Explicit template instantiations — subscribe
    template <>
    Result<void> LcmBroker::subscribe<raccoon::vector3f_t>(const std::string& ch,
                                                           std::function<void(const raccoon::vector3f_t &)> h,
                                                           const raccoon::SubscribeOptions& opts)
    {
        return impl_->subscribe<raccoon::vector3f_t>(ch, std::move(h), opts);
    }

    template <>
    Result<void> LcmBroker::subscribe<raccoon::quaternion_t>(const std::string& ch,
                                                             std::function<void(const raccoon::quaternion_t &)> h,
                                                             const raccoon::SubscribeOptions& opts)
    {
        return impl_->subscribe<raccoon::quaternion_t>(ch, std::move(h), opts);
    }

    template <>
    Result<void> LcmBroker::subscribe<raccoon::scalar_i32_t>(const std::string& ch,
                                                             std::function<void(const raccoon::scalar_i32_t &)> h,
                                                             const raccoon::SubscribeOptions& opts)
    {
        return impl_->subscribe<raccoon::scalar_i32_t>(ch, std::move(h), opts);
    }

    template <>
    Result<void> LcmBroker::subscribe<raccoon::scalar_f_t>(const std::string& ch,
                                                           std::function<void(const raccoon::scalar_f_t &)> h,
                                                           const raccoon::SubscribeOptions& opts)
    {
        return impl_->subscribe<raccoon::scalar_f_t>(ch, std::move(h), opts);
    }

    template <>
    Result<void> LcmBroker::subscribe<raccoon::scalar_i8_t>(const std::string& ch,
                                                            std::function<void(const raccoon::scalar_i8_t &)> h,
                                                            const raccoon::SubscribeOptions& opts)
    {
        return impl_->subscribe<raccoon::scalar_i8_t>(ch, std::move(h), opts);
    }

    template <>
    Result<void> LcmBroker::subscribe<raccoon::orientation_matrix_t>(const std::string& ch,
                                                                     std::function<void(
                                                                     const raccoon::orientation_matrix_t&)
    >
    h
    ,
    const raccoon::SubscribeOptions& opts
    )
    {
        return impl_->subscribe<raccoon::orientation_matrix_t>(ch, std::move(h), opts);
    }

    template <>
    Result<void> LcmBroker::subscribe<raccoon::kinematics_config_t>(const std::string& ch,
                                                                    std::function<void(
const raccoon::kinematics_config_t &)> h,
                                                                    const raccoon::SubscribeOptions& opts)
    {
        return impl_->subscribe<raccoon::kinematics_config_t>(ch, std::move(h), opts);
    }
} // namespace wombat
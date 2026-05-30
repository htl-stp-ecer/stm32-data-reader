#include "wombat/messaging/LcmBroker.h"
#include <raccoon/Transport.h>
#include <raccoon/Options.h>
#include <raccoon/vector3f_t.hpp>
#include <raccoon/quaternion_t.hpp>
#include <raccoon/scalar_f_t.hpp>
#include <raccoon/scalar_i32_t.hpp>
#include <raccoon/scalar_i8_t.hpp>
#include <raccoon/string_t.hpp>
#include <raccoon/Channels.h>
#include <raccoon/orientation_matrix_t.hpp>
#include <raccoon/kinematics_config_t.hpp>
#include <algorithm>
#include <chrono>
#include <map>
#include <sstream>
#include <vector>

namespace wombat
{
    class LcmBroker::Impl
    {
    public:
        static constexpr int64_t kLatencyAlertThresholdUs = 10'000;

        struct ChannelTimingStats
        {
            uint64_t intervalCount{0};
            uint64_t totalCount{0};
            uint64_t intervalTotalUs{0};
            uint64_t totalTotalUs{0};
            uint64_t intervalMaxUs{0};
            uint64_t totalMaxUs{0};
        };

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
                // 1 ms spin slice — rrb_reader_recv is a cheap atomic
                // load, so polling at 1 ms gives ~1 ms p50 inbound
                // latency. Was 5 ms back when the iceoryx2 backend made
                // each spin call ~hundreds of us; rrb costs ~hundreds
                // of nanoseconds per channel.
                const int result = transport_->spinOnce(1);
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
                            + " p99=" + std::to_string(stats.latency.p99Us / 1000) + "ms"
                            + " max=" + std::to_string(stats.latency.maxUs / 1000) + "ms"
                            + " (n=" + std::to_string(stats.latency.count) + ")";

                        handleLatencyAlert(stats.latency);
                    }
                    std::string dedupStr;
                    if (stats.publishesDeduplicated > 0)
                    {
                        dedupStr = ", dedup=" + std::to_string(stats.publishesDeduplicated);
                    }
                    const std::string subscriberStats = formatChannelStatsReport();
                    logger_->info("LCM processMessages: avg=" + std::to_string(avgMsgsPerCall).substr(0, 5)
                        + " msgs/call, rate=" + std::to_string(hz).substr(0, 5) + "Hz"
                        + ", total calls=" + std::to_string(processCallCount_)
                        + ", total msgs=" + std::to_string(totalMessagesProcessed_ + messagesProcessed)
                        + ", latency: " + latencyStr
                        + dedupStr);
                    if (!subscriberStats.empty())
                    {
                        logger_->info("LCM subscriber stats: " + subscriberStats);
                    }
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

            auto instrumentedHandler = [this, channel, handler = std::move(handler)](const T& message)
            {
                const auto started = std::chrono::steady_clock::now();
                handler(message);
                const auto elapsedUs = static_cast<uint64_t>(
                    std::chrono::duration_cast<std::chrono::microseconds>(
                        std::chrono::steady_clock::now() - started).count());
                auto& stats = channelTimingStats_[channel];
                ++stats.intervalCount;
                ++stats.totalCount;
                stats.intervalTotalUs += elapsedUs;
                stats.totalTotalUs += elapsedUs;
                stats.intervalMaxUs = std::max(stats.intervalMaxUs, elapsedUs);
                stats.totalMaxUs = std::max(stats.totalMaxUs, elapsedUs);
            };

            transport_->subscribe<T>(channel, std::move(instrumentedHandler), options);

            logger_->debug("Subscribed to " + std::string(T::getTypeName()) + " channel: " + channel);
            return Result<void>::success();
        }

    private:
        void handleLatencyAlert(const raccoon::TransportStats::Latency& latency)
        {
            if (latency.p99Us > kLatencyAlertThresholdUs)
            {
                ++latencyAlertConsecutiveCount_;
                const std::string message =
                    "stm32_data_reader inbound LCM latency p99="
                    + std::to_string(latency.p99Us / 1000)
                    + "ms exceeds 10ms"
                    + " (avg=" + std::to_string(latency.avgUs / 1000)
                    + "ms, max=" + std::to_string(latency.maxUs / 1000)
                    + "ms, n=" + std::to_string(latency.count) + ")";

                if (!latencyAlertActive_ || latencyAlertConsecutiveCount_ % 10 == 1)
                {
                    logger_->error(message);
                    raccoon::string_t errorMsg{};
                    errorMsg.timestamp = std::chrono::duration_cast<std::chrono::microseconds>(
                        std::chrono::system_clock::now().time_since_epoch()).count();
                    errorMsg.value = message;
                    const auto publishResult = publishForce<raccoon::string_t>(
                        raccoon::Channels::ERROR_MESSAGES, errorMsg);
                    if (publishResult.isFailure())
                    {
                        logger_->error("Failed to publish latency alert to raccoon/errors: "
                            + publishResult.error());
                    }
                }

                latencyAlertActive_ = true;
                return;
            }

            if (latencyAlertActive_)
            {
                logger_->info("stm32_data_reader inbound LCM latency recovered: p99="
                    + std::to_string(latency.p99Us / 1000) + "ms");
            }
            latencyAlertActive_ = false;
            latencyAlertConsecutiveCount_ = 0;
        }

        std::string formatChannelStatsReport()
        {
            struct Entry
            {
                std::string channel;
                uint64_t intervalCount{0};
                uint64_t totalCount{0};
                uint64_t intervalTotalUs{0};
                uint64_t totalTotalUs{0};
                uint64_t intervalMaxUs{0};
                uint64_t totalMaxUs{0};
            };

            std::vector<Entry> entries;
            entries.reserve(channelTimingStats_.size());
            for (auto& [channel, stats] : channelTimingStats_)
            {
                if (stats.intervalCount == 0)
                {
                    continue;
                }
                entries.push_back(Entry{
                    .channel = channel,
                    .intervalCount = stats.intervalCount,
                    .totalCount = stats.totalCount,
                    .intervalTotalUs = stats.intervalTotalUs,
                    .totalTotalUs = stats.totalTotalUs,
                    .intervalMaxUs = stats.intervalMaxUs,
                    .totalMaxUs = stats.totalMaxUs,
                });
                stats.intervalCount = 0;
                stats.intervalTotalUs = 0;
                stats.intervalMaxUs = 0;
            }

            if (entries.empty())
            {
                return {};
            }

            uint64_t intervalTotalUsAllChannels = 0;
            for (const auto& entry : entries)
            {
                intervalTotalUsAllChannels += entry.intervalTotalUs;
            }

            std::sort(entries.begin(), entries.end(), [](const Entry& a, const Entry& b)
            {
                if (a.intervalCount != b.intervalCount)
                {
                    return a.intervalCount > b.intervalCount;
                }
                return a.intervalTotalUs > b.intervalTotalUs;
            });

            std::ostringstream out;
            const size_t limit = std::min<size_t>(entries.size(), 8);
            for (size_t i = 0; i < limit; ++i)
            {
                const auto& entry = entries[i];
                const double intervalAvgUs = static_cast<double>(entry.intervalTotalUs)
                    / static_cast<double>(entry.intervalCount);
                if (i > 0)
                {
                    out << " | ";
                }
                const double sharePct = intervalTotalUsAllChannels > 0
                                            ? (100.0 * static_cast<double>(entry.intervalTotalUs)
                                                / static_cast<double>(intervalTotalUsAllChannels))
                                            : 0.0;
                out << entry.channel
                    << ": n=" << entry.intervalCount
                    << ", avg=" << static_cast<uint64_t>(intervalAvgUs)
                    << "us"
                    << ", max=" << entry.intervalMaxUs << "us"
                    << ", share=" << std::to_string(sharePct).substr(0, 4) << "%"
                    << ", total=" << entry.totalCount;
            }
            return out.str();
        }

        std::shared_ptr<Logger> logger_;
        std::unique_ptr<raccoon::Transport> transport_;
        std::map<std::string, ChannelTimingStats> channelTimingStats_;
        std::chrono::steady_clock::time_point lastProcessTime_{};
        uint64_t processCallCount_{0};
        uint64_t totalMessagesProcessed_{0};
        uint64_t intervalMessagesProcessed_{0};
        bool latencyAlertActive_{false};
        uint64_t latencyAlertConsecutiveCount_{0};
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

    template <>
    Result<void> LcmBroker::publishRetained<raccoon::string_t>(const std::string& ch,
                                                               const raccoon::string_t& m)
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

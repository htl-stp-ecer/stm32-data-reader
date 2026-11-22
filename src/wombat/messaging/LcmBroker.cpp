#include "wombat/messaging/LcmBroker.h"
#include <typeinfo>
#include <raccoon/Transport.h>
#include <raccoon/Options.h>
#include <raccoon/vector3f_t.hpp>
#include <raccoon/quaternion_t.hpp>
#include <raccoon/scalar_f_t.hpp>
#include <raccoon/scalar_i32_t.hpp>
#include <raccoon/scalar_i8_t.hpp>
#include <raccoon/message_t.hpp>
#include "wombat/core/Channels.h"
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
        static constexpr int64_t kLatencyAlertThresholdUs = 20'000;

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
                const int result = transport_->spin_once(1);
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
                    const auto stats = transport_->get_and_reset_stats();
                    std::string latencyStr = "no msgs";
                    if (stats.latency.count > 0)
                    {
                        latencyStr = "min=" + std::to_string(stats.latency.min_us / 1000) + "ms"
                            + " avg=" + std::to_string(stats.latency.avg_us / 1000) + "ms"
                            + " p99=" + std::to_string(stats.latency.p99_us / 1000) + "ms"
                            + " max=" + std::to_string(stats.latency.max_us / 1000) + "ms"
                            + " (n=" + std::to_string(stats.latency.count) + ")";

                        handleLatencyAlert(stats.latency);
                    }
                    std::string dedupStr;
                    if (stats.publishes_deduplicated > 0)
                    {
                        dedupStr = ", dedup=" + std::to_string(stats.publishes_deduplicated);
                    }
                    const std::string subscriberStats = formatChannelStatsReport();
                    logger_->debug("LCM processMessages: avg=" + std::to_string(avgMsgsPerCall).substr(0, 5)
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

            const auto published = transport_->publish(channel, message, options);
            if (published != raccoon::PublishResult::Ok)
            {
                return Result<void>::failure("Failed to publish message on channel: " + channel
                    + " (" + raccoon::to_string(published) + ")");
            }

            logger_->debug("Published " + std::string(typeid(MessageType).name())
                + " on channel: " + channel + " (dedup)");
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

            const auto published = transport_->publish(channel, message, options);
            if (published != raccoon::PublishResult::Ok)
            {
                return Result<void>::failure("Failed to publish message on channel: " + channel
                    + " (" + raccoon::to_string(published) + ")");
            }

            logger_->debug("Published " + std::string(typeid(MessageType).name())
                + " on channel: " + channel + " (forced)");
            return Result<void>::success();
        }

        template <TransportMessage T>
        Result<void> subscribe(const std::string& channel,
                               std::function<void(const T &)> handler)
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

            if (transport_->subscribe<T>(channel, std::move(instrumentedHandler)) == 0)
            {
                return Result<void>::failure("Cannot subscribe to " + channel
                    + " (not in the channel table, or not of type " + T::type_name + ")");
            }

            logger_->debug("Subscribed to " + std::string(typeid(T).name()) + " channel: " + channel);
            return Result<void>::success();
        }

    private:
        void handleLatencyAlert(const raccoon::TransportStats::Latency& latency)
        {
            if (latency.p99_us > kLatencyAlertThresholdUs)
            {
                ++latencyAlertConsecutiveCount_;
                const std::string content =
                    "Messages reach the reader late: 99% arrive within "
                    + std::to_string(latency.p99_us / 1000) + " ms (limit 20 ms). "
                    + "Average " + std::to_string(latency.avg_us / 1000) + " ms, worst "
                    + std::to_string(latency.max_us / 1000) + " ms, over "
                    + std::to_string(latency.count) + " messages.";

                if (!latencyAlertActive_ || latencyAlertConsecutiveCount_ % 10 == 1)
                {
                    // To the journal and to messages/error.
                    logger_->error("Commands arrive slowly", content);
                }

                latencyAlertActive_ = true;
                return;
            }

            if (latencyAlertActive_)
            {
                logger_->info("stm32_data_reader inbound LCM latency recovered: p99="
                    + std::to_string(latency.p99_us / 1000) + "ms");
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
    Result<void> LcmBroker::publish<raccoon::message_t>(const std::string& ch, const raccoon::message_t& m)
    {
        return impl_->publishIfChanged(ch, m);
    }

    // Explicit template instantiations — publishForce (plain, no change detection)
    template <>
    Result<void> LcmBroker::publishForce<raccoon::vector3f_t>(const std::string& ch, const raccoon::vector3f_t& m)
    {
        return impl_->publishForce(ch, m);
    }

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

    template <>
    Result<void> LcmBroker::publishForce<raccoon::message_t>(const std::string& ch, const raccoon::message_t& m)
    {
        return impl_->publishForce(ch, m);
    }

    // Explicit template instantiations — subscribe
    template <>
    Result<void> LcmBroker::subscribe<raccoon::vector3f_t>(const std::string& ch,
                                                           std::function<void(const raccoon::vector3f_t &)> h)
    {
        return impl_->subscribe<raccoon::vector3f_t>(ch, std::move(h));
    }

    template <>
    Result<void> LcmBroker::subscribe<raccoon::quaternion_t>(const std::string& ch,
                                                             std::function<void(const raccoon::quaternion_t &)> h)
    {
        return impl_->subscribe<raccoon::quaternion_t>(ch, std::move(h));
    }

    template <>
    Result<void> LcmBroker::subscribe<raccoon::scalar_i32_t>(const std::string& ch,
                                                             std::function<void(const raccoon::scalar_i32_t &)> h)
    {
        return impl_->subscribe<raccoon::scalar_i32_t>(ch, std::move(h));
    }

    template <>
    Result<void> LcmBroker::subscribe<raccoon::scalar_f_t>(const std::string& ch,
                                                           std::function<void(const raccoon::scalar_f_t &)> h)
    {
        return impl_->subscribe<raccoon::scalar_f_t>(ch, std::move(h));
    }

    template <>
    Result<void> LcmBroker::subscribe<raccoon::scalar_i8_t>(const std::string& ch,
                                                            std::function<void(const raccoon::scalar_i8_t &)> h)
    {
        return impl_->subscribe<raccoon::scalar_i8_t>(ch, std::move(h));
    }

    template <>
    Result<void> LcmBroker::subscribe<raccoon::orientation_matrix_t>(const std::string& ch,
                                                                     std::function<void(
                                                                     const raccoon::orientation_matrix_t&)
    >
    h
    )
    {
        return impl_->subscribe<raccoon::orientation_matrix_t>(ch, std::move(h));
    }

    template <>
    Result<void> LcmBroker::subscribe<raccoon::kinematics_config_t>(const std::string& ch,
                                                                    std::function<void(
const raccoon::kinematics_config_t &)> h)
    {
        return impl_->subscribe<raccoon::kinematics_config_t>(ch, std::move(h));
    }
} // namespace wombat

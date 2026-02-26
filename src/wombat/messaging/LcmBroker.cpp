#include "wombat/messaging/LcmBroker.h"
#include <lcm/lcm-cpp.hpp>
#include <unordered_map>
#include <vector>
#include <cstring>
#include <chrono>
#include <algorithm>
#include <limits>
#include <mutex>

namespace wombat
{
    namespace
    {
        int64_t nowUsec()
        {
            return std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count();
        }

        struct LatencyStats
        {
            std::mutex mtx;
            int64_t minUs{std::numeric_limits<int64_t>::max()};
            int64_t maxUs{0};
            int64_t sumUs{0};
            uint64_t count{0};

            void record(int64_t latencyUs)
            {
                std::lock_guard<std::mutex> lock(mtx);
                if (latencyUs < 0) return; // clock skew, ignore
                minUs = std::min(minUs, latencyUs);
                maxUs = std::max(maxUs, latencyUs);
                sumUs += latencyUs;
                ++count;
            }

            struct Snapshot
            {
                int64_t minUs, maxUs, avgUs;
                uint64_t count;
            };

            Snapshot resetAndSnapshot()
            {
                std::lock_guard<std::mutex> lock(mtx);
                Snapshot s{};
                s.count = count;
                if (count > 0)
                {
                    s.minUs = minUs;
                    s.maxUs = maxUs;
                    s.avgUs = static_cast<int64_t>(sumUs / count);
                }
                minUs = std::numeric_limits<int64_t>::max();
                maxUs = 0;
                sumUs = 0;
                count = 0;
                return s;
            }
        };
    } // anonymous namespace

    class LcmBroker::Impl
    {
    public:
        explicit Impl(std::shared_ptr<Logger> logger)
            : logger_{std::move(logger)}
        {
        }

        Result<void> initialize()
        {
            lcm_ = std::make_unique<lcm::LCM>();

            if (!lcm_->good())
            {
                return Result<void>::failure("Failed to initialize LCM");
            }

            logger_->info("LCM message broker initialized successfully");
            return Result<void>::success();
        }

        Result<void> shutdown()
        {
            lcm_.reset();
            logger_->info("LCM message broker shut down");
            return Result<void>::success();
        }

        Result<void> processMessages()
        {
            if (!lcm_ || !lcm_->good())
            {
                return Result<void>::failure("LCM not initialized or unhealthy");
            }

            // Drain all pending messages instead of just one
            int messagesProcessed = 0;
            while (true)
            {
                const int result = lcm_->handleTimeout(0);
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
                    const auto latency = latencyStats_.resetAndSnapshot();
                    std::string latencyStr = "no msgs";
                    if (latency.count > 0)
                    {
                        latencyStr = "min=" + std::to_string(latency.minUs / 1000) + "ms"
                            + " avg=" + std::to_string(latency.avgUs / 1000) + "ms"
                            + " max=" + std::to_string(latency.maxUs / 1000) + "ms"
                            + " (n=" + std::to_string(latency.count) + ")";
                    }
                    logger_->info("LCM processMessages: avg=" + std::to_string(avgMsgsPerCall).substr(0, 5)
                        + " msgs/call, rate=" + std::to_string(hz).substr(0, 5) + "Hz"
                        + ", total calls=" + std::to_string(processCallCount_)
                        + ", total msgs=" + std::to_string(totalMessagesProcessed_ + messagesProcessed)
                        + ", latency: " + latencyStr);
                    intervalMessagesProcessed_ = 0;
                }
            }

            totalMessagesProcessed_ += messagesProcessed;
            lastProcessTime_ = now;

            return Result<void>::success();
        }

        bool isHealthy() const
        {
            return lcm_ && lcm_->good();
        }

        template <typename MessageType>
        Result<void> publishIfChanged(const std::string& channel, const MessageType& message)
        {
            if (!lcm_ || !lcm_->good())
            {
                return Result<void>::failure("LCM not initialized or unhealthy");
            }

            if (!hasMessageChanged(channel, message))
            {
                return Result<void>::success(); // No change, skip publishing
            }

            const int result = lcm_->publish(channel, &message);
            if (result != 0)
            {
                return Result<void>::failure("Failed to publish message on channel: " + channel);
            }

            return Result<void>::success();
        }

        template <typename MessageType>
        Result<void> publishForce(const std::string& channel, const MessageType& message)
        {
            if (!lcm_ || !lcm_->good())
            {
                return Result<void>::failure("LCM not initialized or unhealthy");
            }

            const int result = lcm_->publish(channel, &message);
            if (result != 0)
            {
                return Result<void>::failure("Failed to publish message on channel: " + channel);
            }

            return Result<void>::success();
        }

        template <LcmMessage T>
        Result<void> subscribe(const std::string& channel,
                               std::function<void(const T &)> handler)
        {
            if (!lcm_ || !lcm_->good())
            {
                return Result<void>::failure("LCM not initialized or unhealthy");
            }

            std::function < void(const lcm::ReceiveBuffer *, const std::string &, const T *) > lcmHandler =
                [handler, this](const lcm::ReceiveBuffer*, const std::string&, const T* msg)
                {
                    if (msg)
                    {
                        latencyStats_.record(nowUsec() - msg->timestamp);
                        handler(*msg);
                    }
                };
            lcm_->subscribe<T>(channel, lcmHandler);

            logger_->debug("Subscribed to " + std::string(T::getTypeName()) + " channel: " + channel);
            return Result<void>::success();
        }

    private:
        std::shared_ptr<Logger> logger_;
        std::unique_ptr<lcm::LCM> lcm_;
        std::unordered_map<std::string, std::vector<uint8_t>> messageCache_;
        std::chrono::steady_clock::time_point lastProcessTime_{};
        uint64_t processCallCount_{0};
        uint64_t totalMessagesProcessed_{0};
        uint64_t intervalMessagesProcessed_{0};
        LatencyStats latencyStats_;

        template <typename MessageType>
        bool hasMessageChanged(const std::string& channel, const MessageType& message)
        {
            const auto* messageBytes = reinterpret_cast<const uint8_t*>(&message);
            const size_t messageSize = sizeof(MessageType);

            auto& cachedMessage = messageCache_[channel];
            if (cachedMessage.size() == messageSize &&
                std::memcmp(cachedMessage.data(), messageBytes, messageSize) == 0)
            {
                return false; // No change
            }

            cachedMessage.assign(messageBytes, messageBytes + messageSize);
            return true; // Changed
        }
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

    // Explicit template instantiations — publish
    template <>
    Result<void> LcmBroker::publish<exlcm::vector3f_t>(const std::string& ch, const exlcm::vector3f_t& m)
    {
        return impl_->publishIfChanged(ch, m);
    }

    template <>
    Result<void> LcmBroker::publish<exlcm::quaternion_t>(const std::string& ch, const exlcm::quaternion_t& m)
    {
        return impl_->publishIfChanged(ch, m);
    }

    template <>
    Result<void> LcmBroker::publish<exlcm::scalar_f_t>(const std::string& ch, const exlcm::scalar_f_t& m)
    {
        return impl_->publishIfChanged(ch, m);
    }

    template <>
    Result<void> LcmBroker::publish<exlcm::scalar_i32_t>(const std::string& ch, const exlcm::scalar_i32_t& m)
    {
        return impl_->publishIfChanged(ch, m);
    }

    template <>
    Result<void> LcmBroker::publish<exlcm::scalar_i8_t>(const std::string& ch, const exlcm::scalar_i8_t& m)
    {
        return impl_->publishIfChanged(ch, m);
    }

    template <>
    Result<void> LcmBroker::publish<exlcm::string_t>(const std::string& ch, const exlcm::string_t& m)
    {
        return impl_->publishIfChanged(ch, m);
    }

    // Explicit template instantiations — publishForce
    template <>
    Result<void> LcmBroker::publishForce<exlcm::scalar_i8_t>(const std::string& ch, const exlcm::scalar_i8_t& m)
    {
        return impl_->publishForce(ch, m);
    }

    template <>
    Result<void> LcmBroker::publishForce<exlcm::scalar_i32_t>(const std::string& ch, const exlcm::scalar_i32_t& m)
    {
        return impl_->publishForce(ch, m);
    }

    // Explicit template instantiations — subscribe
    template <>
    Result<void> LcmBroker::subscribe<exlcm::vector3f_t>(const std::string& ch,
                                                         std::function<void(const exlcm::vector3f_t &)> h)
    {
        return impl_->subscribe<exlcm::vector3f_t>(ch, std::move(h));
    }

    template <>
    Result<void> LcmBroker::subscribe<exlcm::quaternion_t>(const std::string& ch,
                                                           std::function<void(const exlcm::quaternion_t &)> h)
    {
        return impl_->subscribe<exlcm::quaternion_t>(ch, std::move(h));
    }

    template <>
    Result<void> LcmBroker::subscribe<exlcm::scalar_i32_t>(const std::string& ch,
                                                           std::function<void(const exlcm::scalar_i32_t &)> h)
    {
        return impl_->subscribe<exlcm::scalar_i32_t>(ch, std::move(h));
    }

    template <>
    Result<void> LcmBroker::subscribe<exlcm::scalar_f_t>(const std::string& ch,
                                                         std::function<void(const exlcm::scalar_f_t &)> h)
    {
        return impl_->subscribe<exlcm::scalar_f_t>(ch, std::move(h));
    }

    template <>
    Result<void> LcmBroker::subscribe<exlcm::scalar_i8_t>(const std::string& ch,
                                                          std::function<void(const exlcm::scalar_i8_t &)> h)
    {
        return impl_->subscribe<exlcm::scalar_i8_t>(ch, std::move(h));
    }

    template <>
    Result<void> LcmBroker::subscribe<exlcm::orientation_matrix_t>(const std::string& ch,
                                                                   std::function<void(
const exlcm::orientation_matrix_t &)> h)
    {
        return impl_->subscribe<exlcm::orientation_matrix_t>(ch, std::move(h));
    }
} // namespace wombat
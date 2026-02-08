#include "wombat/messaging/LcmBroker.h"
#include <lcm/lcm-cpp.hpp>
#include <unordered_map>
#include <vector>
#include <cstring>
#include <chrono>
#include <algorithm>
#include <limits>
#include <mutex>

namespace wombat {

namespace {

int64_t nowUsec() {
    return std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
}

struct LatencyStats {
    std::mutex mtx;
    int64_t minUs{std::numeric_limits<int64_t>::max()};
    int64_t maxUs{0};
    int64_t sumUs{0};
    uint64_t count{0};

    void record(int64_t latencyUs) {
        std::lock_guard<std::mutex> lock(mtx);
        if (latencyUs < 0) return; // clock skew, ignore
        minUs = std::min(minUs, latencyUs);
        maxUs = std::max(maxUs, latencyUs);
        sumUs += latencyUs;
        ++count;
    }

    struct Snapshot {
        int64_t minUs, maxUs, avgUs;
        uint64_t count;
    };

    Snapshot resetAndSnapshot() {
        std::lock_guard<std::mutex> lock(mtx);
        Snapshot s{};
        s.count = count;
        if (count > 0) {
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

class LcmBroker::Impl {
public:
    explicit Impl(std::shared_ptr<Logger> logger)
        : logger_{std::move(logger)} {}

    Result<void> initialize() {
        lcm_ = std::make_unique<lcm::LCM>();

        if (!lcm_->good()) {
            return Result<void>::failure("Failed to initialize LCM");
        }

        logger_->info("LCM message broker initialized successfully");
        return Result<void>::success();
    }

    Result<void> shutdown() {
        lcm_.reset();
        logger_->info("LCM message broker shut down");
        return Result<void>::success();
    }

    Result<void> processMessages() {
        if (!lcm_ || !lcm_->good()) {
            return Result<void>::failure("LCM not initialized or unhealthy");
        }

        // Drain all pending messages instead of just one
        int messagesProcessed = 0;
        while (true) {
            const int result = lcm_->handleTimeout(0);
            if (result < 0) {
                return Result<void>::failure("Failed to process LCM messages");
            }
            if (result == 0) {
                break; // No more pending messages
            }
            ++messagesProcessed;
        }

        const auto now = std::chrono::steady_clock::now();
        ++processCallCount_;

        intervalMessagesProcessed_ += messagesProcessed;

        if (lastProcessTime_.time_since_epoch().count() > 0) {
            const auto dt = std::chrono::duration<double>(now - lastProcessTime_).count();
            const double hz = (dt > 0.0) ? 1.0 / dt : 0.0;

            // Log every 100 calls to avoid spamming
            if (processCallCount_ % 100 == 0) {
                const double avgMsgsPerCall = static_cast<double>(intervalMessagesProcessed_) / 100.0;
                const auto latency = latencyStats_.resetAndSnapshot();
                std::string latencyStr = "no msgs";
                if (latency.count > 0) {
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

    bool isHealthy() const {
        return lcm_ && lcm_->good();
    }

    template<typename MessageType>
    Result<void> publishIfChanged(const std::string& channel, const MessageType& message) {
        if (!lcm_ || !lcm_->good()) {
            return Result<void>::failure("LCM not initialized or unhealthy");
        }

        if (!hasMessageChanged(channel, message)) {
            return Result<void>::success(); // No change, skip publishing
        }

        const int result = lcm_->publish(channel, &message);
        if (result != 0) {
            return Result<void>::failure("Failed to publish message on channel: " + channel);
        }

        return Result<void>::success();
    }

    template<typename MessageType>
    Result<void> publishForce(const std::string& channel, const MessageType& message) {
        if (!lcm_ || !lcm_->good()) {
            return Result<void>::failure("LCM not initialized or unhealthy");
        }

        const int result = lcm_->publish(channel, &message);
        if (result != 0) {
            return Result<void>::failure("Failed to publish message on channel: " + channel);
        }

        return Result<void>::success();
    }

    Result<void> subscribeVector3f(const std::string& channel,
                                  std::function<void(const exlcm::vector3f_t&)> handler) {
        if (!lcm_ || !lcm_->good()) {
            return Result<void>::failure("LCM not initialized or unhealthy");
        }

        std::function<void(const lcm::ReceiveBuffer*, const std::string&, const exlcm::vector3f_t*)> lcmHandler =
            [handler, this](const lcm::ReceiveBuffer*, const std::string&, const exlcm::vector3f_t* msg) {
                if (msg) {
                    latencyStats_.record(nowUsec() - msg->timestamp);
                    handler(*msg);
                }
            };
        lcm_->subscribe<exlcm::vector3f_t>(channel, lcmHandler);

        logger_->debug("Subscribed to Vector3f channel: " + channel);
        return Result<void>::success();
    }

    Result<void> subscribeQuaternion(const std::string& channel,
                                     std::function<void(const exlcm::quaternion_t&)> handler) {
        if (!lcm_ || !lcm_->good()) {
            return Result<void>::failure("LCM not initialized or unhealthy");
        }

        std::function<void(const lcm::ReceiveBuffer*, const std::string&, const exlcm::quaternion_t*)> lcmHandler =
            [handler, this](const lcm::ReceiveBuffer*, const std::string&, const exlcm::quaternion_t* msg) {
                if (msg) {
                    latencyStats_.record(nowUsec() - msg->timestamp);
                    handler(*msg);
                }
            };
        lcm_->subscribe<exlcm::quaternion_t>(channel, lcmHandler);

        logger_->debug("Subscribed to Quaternion channel: " + channel);
        return Result<void>::success();
    }

    Result<void> subscribeScalarI32(const std::string& channel,
                                   std::function<void(const exlcm::scalar_i32_t&)> handler) {
        if (!lcm_ || !lcm_->good()) {
            return Result<void>::failure("LCM not initialized or unhealthy");
        }

        std::function<void(const lcm::ReceiveBuffer*, const std::string&, const exlcm::scalar_i32_t*)> lcmHandler =
            [handler, this](const lcm::ReceiveBuffer*, const std::string&, const exlcm::scalar_i32_t* msg) {
                if (msg) {
                    latencyStats_.record(nowUsec() - msg->timestamp);
                    handler(*msg);
                }
            };
        lcm_->subscribe<exlcm::scalar_i32_t>(channel, lcmHandler);

        logger_->debug("Subscribed to ScalarI32 channel: " + channel);
        return Result<void>::success();
    }

    Result<void> subscribeScalarF(const std::string& channel,
                                 std::function<void(const exlcm::scalar_f_t&)> handler) {
        if (!lcm_ || !lcm_->good()) {
            return Result<void>::failure("LCM not initialized or unhealthy");
        }

        std::function<void(const lcm::ReceiveBuffer*, const std::string&, const exlcm::scalar_f_t*)> lcmHandler =
            [handler, this](const lcm::ReceiveBuffer*, const std::string&, const exlcm::scalar_f_t* msg) {
                if (msg) {
                    latencyStats_.record(nowUsec() - msg->timestamp);
                    handler(*msg);
                }
            };
        lcm_->subscribe<exlcm::scalar_f_t>(channel, lcmHandler);

        logger_->debug("Subscribed to ScalarF channel: " + channel);
        return Result<void>::success();
    }

    Result<void> subscribeScalarI8(const std::string& channel,
                                  std::function<void(const exlcm::scalar_i8_t&)> handler) {
        if (!lcm_ || !lcm_->good()) {
            return Result<void>::failure("LCM not initialized or unhealthy");
        }

        std::function<void(const lcm::ReceiveBuffer*, const std::string&, const exlcm::scalar_i8_t*)> lcmHandler =
            [handler, this](const lcm::ReceiveBuffer*, const std::string&, const exlcm::scalar_i8_t* msg) {
                if (msg) {
                    latencyStats_.record(nowUsec() - msg->timestamp);
                    handler(*msg);
                }
            };
        lcm_->subscribe<exlcm::scalar_i8_t>(channel, lcmHandler);

        logger_->debug("Subscribed to ScalarI8 channel: " + channel);
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

    template<typename MessageType>
    bool hasMessageChanged(const std::string& channel, const MessageType& message) {
        const auto* messageBytes = reinterpret_cast<const uint8_t*>(&message);
        const size_t messageSize = sizeof(MessageType);

        auto& cachedMessage = messageCache_[channel];
        if (cachedMessage.size() == messageSize &&
            std::memcmp(cachedMessage.data(), messageBytes, messageSize) == 0) {
            return false; // No change
        }

        cachedMessage.assign(messageBytes, messageBytes + messageSize);
        return true; // Changed
    }
};

// Public interface implementation
LcmBroker::LcmBroker(std::shared_ptr<Logger> logger)
    : impl_{std::make_unique<Impl>(std::move(logger))} {}

LcmBroker::~LcmBroker() = default;

Result<void> LcmBroker::initialize() {
    return impl_->initialize();
}

Result<void> LcmBroker::shutdown() {
    return impl_->shutdown();
}

Result<void> LcmBroker::processMessages() {
    return impl_->processMessages();
}

bool LcmBroker::isHealthy() const {
    return impl_->isHealthy();
}

Result<void> LcmBroker::publishVector3f(const std::string& channel, const exlcm::vector3f_t& message) {
    return impl_->publishIfChanged(channel, message);
}

Result<void> LcmBroker::publishQuaternion(const std::string& channel, const exlcm::quaternion_t& message) {
    return impl_->publishIfChanged(channel, message);
}

Result<void> LcmBroker::publishScalarF(const std::string& channel, const exlcm::scalar_f_t& message) {
    return impl_->publishIfChanged(channel, message);
}

Result<void> LcmBroker::publishScalarI32(const std::string& channel, const exlcm::scalar_i32_t& message) {
    return impl_->publishIfChanged(channel, message);
}

Result<void> LcmBroker::publishScalarI8(const std::string& channel, const exlcm::scalar_i8_t& message) {
    return impl_->publishIfChanged(channel, message);
}

Result<void> LcmBroker::publishScalarI8Force(const std::string& channel, const exlcm::scalar_i8_t& message) {
    return impl_->publishForce(channel, message);
}

Result<void> LcmBroker::publishScalarI32Force(const std::string& channel, const exlcm::scalar_i32_t& message) {
    return impl_->publishForce(channel, message);
}

Result<void> LcmBroker::publishString(const std::string& channel, const exlcm::string_t& message) {
    return impl_->publishIfChanged(channel, message);
}

Result<void> LcmBroker::subscribeVector3f(const std::string& channel,
                                         std::function<void(const exlcm::vector3f_t&)> handler) {
    return impl_->subscribeVector3f(channel, std::move(handler));
}

Result<void> LcmBroker::subscribeQuaternion(const std::string& channel,
                                            std::function<void(const exlcm::quaternion_t&)> handler) {
    return impl_->subscribeQuaternion(channel, std::move(handler));
}

Result<void> LcmBroker::subscribeScalarI32(const std::string& channel,
                                          std::function<void(const exlcm::scalar_i32_t&)> handler) {
    return impl_->subscribeScalarI32(channel, std::move(handler));
}

Result<void> LcmBroker::subscribeScalarF(const std::string& channel,
                                        std::function<void(const exlcm::scalar_f_t&)> handler) {
    return impl_->subscribeScalarF(channel, std::move(handler));
}

Result<void> LcmBroker::subscribeScalarI8(const std::string& channel,
                                         std::function<void(const exlcm::scalar_i8_t&)> handler) {
    return impl_->subscribeScalarI8(channel, std::move(handler));
}

} // namespace wombat

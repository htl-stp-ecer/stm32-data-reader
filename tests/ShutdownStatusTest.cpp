// The hardware latch stops the motors only. The status that the reader publishes
// on Channels::SHUTDOWN_STATUS must say exactly that: a user shutdown is the motor
// bit alone, a watchdog shutdown adds the watchdog source bit, and bit 0 (servo) is
// never set.

#include "wombat/core/ShutdownFlags.h"
#include "wombat/messaging/LcmBroker.h"
#include "wombat/services/CommandSubscriber.h"
#include "wombat/services/DataPublisher.h"
#include "wombat/services/DeviceController.h"
#include "wombat/services/MotorWatchdog.h"
#include "mocks/MockLogger.h"
#include "mocks/MockSpi.h"

#include "wombat/core/Channels.h"
#include <raccoon/raccoon_ring.h>
#include <raccoon/scalar_i32_t.hpp>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <thread>
#include <unistd.h>

using namespace wombat;
using namespace wombat::test;
using ::testing::_;
using ::testing::NiceMock;
using ::testing::Return;

namespace
{
    void removeRing(const std::string& channel)
    {
        char path[512]{};
        if (rrb_channel_to_path(channel.c_str(), path, sizeof(path)) == 0) (void)::unlink(path);
    }

    int64_t nowUs()
    {
        return std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
    }
}

class ShutdownStatusTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        // Rings outlive processes; frames from an earlier run must not be replayed.
        removeRing(wombat::Channels::SHUTDOWN_CMD);
        removeRing(wombat::Channels::SHUTDOWN_STATUS);

        logger_ = std::make_shared<MockLogger>();
        broker_ = std::make_shared<LcmBroker>(logger_);
        ASSERT_TRUE(broker_->initialize().isSuccess());

        auto spi = std::make_unique<NiceMock<MockSpi>>();
        spi_ = spi.get();
        ON_CALL(*spi_, initialize()).WillByDefault(Return(Result<void>::success()));
        ON_CALL(*spi_, shutdown()).WillByDefault(Return(Result<void>::success()));
        ON_CALL(*spi_, setShutdown(_)).WillByDefault(Return(Result<void>::success()));
        ON_CALL(*spi_, setMotorState(_, _)).WillByDefault(Return(Result<void>::success()));
        ON_CALL(*spi_, setServoState(_, _)).WillByDefault(Return(Result<void>::success()));
        ON_CALL(*spi_, getServoState(_))
            .WillByDefault(Return(Result<ServoState>::success(ServoState{ServoMode::Disabled, 0.0f})));

        controller_ = std::make_shared<DeviceController>(std::move(spi), logger_);
        ASSERT_TRUE(controller_->initialize().isSuccess());
        publisher_ = std::make_shared<DataPublisher>(broker_, logger_);
        watchdog_ = std::make_unique<MotorWatchdog>();
        subscriber_ = std::make_unique<CommandSubscriber>(broker_, controller_, publisher_, logger_, *watchdog_);
        ASSERT_TRUE(subscriber_->initialize().isSuccess());

        ASSERT_TRUE(broker_->subscribe<raccoon::scalar_i32_t>(
            wombat::Channels::SHUTDOWN_STATUS,
            [this](const raccoon::scalar_i32_t& status) {
                lastStatus_ = status.value;
                ++statusCount_;
            }).isSuccess());
    }

    void TearDown() override
    {
        subscriber_.reset();
        broker_->shutdown();
    }

    // Send a shutdown command and wait for the status the reader publishes in answer.
    int waitForStatus(bool enabled)
    {
        const int before = statusCount_.load();
        raccoon::scalar_i32_t cmd{};
        cmd.timestamp = nowUs();
        cmd.value = enabled ? 1 : 0;
        EXPECT_TRUE(broker_->publish(wombat::Channels::SHUTDOWN_CMD, cmd).isSuccess());
        for (int i = 0; i < 100 && statusCount_.load() == before; ++i)
        {
            broker_->processMessages();
            if (statusCount_.load() == before)
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        EXPECT_GT(statusCount_.load(), before) << "no shutdown status was published";
        return lastStatus_.load();
    }

    std::shared_ptr<MockLogger> logger_;
    std::shared_ptr<LcmBroker> broker_;
    NiceMock<MockSpi>* spi_ = nullptr;
    std::shared_ptr<DeviceController> controller_;
    std::shared_ptr<DataPublisher> publisher_;
    std::unique_ptr<MotorWatchdog> watchdog_;
    std::unique_ptr<CommandSubscriber> subscriber_;
    std::atomic<int> lastStatus_{-1};
    std::atomic<int> statusCount_{0};
};

TEST_F(ShutdownStatusTest, UserShutdownLatchesMotorsOnly)
{
    // The command latches the motors; tearing the controller down latches them once more.
    EXPECT_CALL(*spi_, setShutdown(true)).Times(::testing::AtLeast(1));

    const int status = waitForStatus(true);

    EXPECT_EQ(status, shutdown_flags::kMotor);
    EXPECT_EQ(status & 0x01, 0) << "servo bit must never be set";
    EXPECT_EQ(status & shutdown_flags::kSourceWatchdog, 0) << "a user shutdown has no watchdog source";
}

TEST_F(ShutdownStatusTest, ReleaseClearsTheStatus)
{
    waitForStatus(true);
    EXPECT_EQ(waitForStatus(false), shutdown_flags::kClear);
}

TEST(ShutdownFlagsTest, WatchdogLatchIsMotorPlusSource)
{
    EXPECT_EQ(shutdown_flags::kWatchdogLatch, 0x06);
    EXPECT_EQ(shutdown_flags::kUserLatch, 0x02);
    EXPECT_EQ(shutdown_flags::kWatchdogLatch & 0x01, 0);
}

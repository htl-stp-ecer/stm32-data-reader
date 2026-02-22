#include "wombat/services/DataPublisher.h"
#include "wombat/messaging/LcmBroker.h"
#include "wombat/core/Channels.h"
#include "mocks/MockLogger.h"
#include <gtest/gtest.h>
#include <atomic>
#include <thread>
#include <chrono>

using namespace wombat;
using namespace wombat::test;

class DataPublisherTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        logger_ = std::make_shared<MockLogger>();
        broker_ = std::make_shared<LcmBroker>(logger_);
        auto initResult = broker_->initialize();
        ASSERT_TRUE(initResult.isSuccess());
        publisher_ = std::make_unique<DataPublisher>(broker_, logger_);
    }

    void TearDown() override
    {
        broker_->shutdown();
    }

    // Drain LCM messages with retries to handle loopback delivery latency
    void drainMessages(const std::atomic<bool>& flag, int maxRetries = 20)
    {
        for (int i = 0; i < maxRetries && !flag.load(); ++i)
        {
            broker_->processMessages();
            if (!flag.load())
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }

    std::shared_ptr<MockLogger> logger_;
    std::shared_ptr<LcmBroker> broker_;
    std::unique_ptr<DataPublisher> publisher_;
};

TEST_F(DataPublisherTest, PublishSensorDataSucceeds)
{
    SensorData data{};
    data.gyro = {1.0f, 2.0f, 3.0f};
    data.accelerometer = {0.0f, 0.0f, 1.0f};
    data.magnetometer = {0.3f, 0.1f, 0.5f};
    data.orientation = {1.0f, 0.0f, 0.0f, 0.0f};
    data.temperature = 25.0f;
    data.batteryVoltage = 12.0f;

    auto result = publisher_->publishSensorData(data);
    EXPECT_TRUE(result.isSuccess());
}

TEST_F(DataPublisherTest, PublishSensorDataWithSubscription)
{
    std::atomic<bool> received{false};
    exlcm::vector3f_t receivedMsg{};

    broker_->subscribe<exlcm::vector3f_t>(Channels::GYRO,
                                          [&](const exlcm::vector3f_t& msg)
                                          {
                                              receivedMsg = msg;
                                              received = true;
                                          });

    SensorData data{};
    data.gyro = {1.5f, 2.5f, 3.5f};

    auto result = publisher_->publishSensorData(data);
    EXPECT_TRUE(result.isSuccess());

    drainMessages(received);

    EXPECT_TRUE(received.load());
    EXPECT_FLOAT_EQ(receivedMsg.x, 1.5f);
    EXPECT_FLOAT_EQ(receivedMsg.y, 2.5f);
    EXPECT_FLOAT_EQ(receivedMsg.z, 3.5f);
}

TEST_F(DataPublisherTest, PublishMotorStateSucceeds)
{
    MotorState state{MotorControlMode::Pwm, 200, 0, 50, 1000, false};

    auto result = publisher_->publishMotorState(0, state);
    EXPECT_TRUE(result.isSuccess());
}

TEST_F(DataPublisherTest, PublishMotorStateInvalidPort)
{
    MotorState state{};
    auto result = publisher_->publishMotorState(MAX_MOTOR_PORTS, state);
    EXPECT_TRUE(result.isFailure());
}

TEST_F(DataPublisherTest, PublishMotorStateWithSubscription)
{
    std::atomic<bool> receivedPower{false};
    int32_t powerValue = 0;

    broker_->subscribe<exlcm::scalar_i32_t>(Channels::motorPower(0),
                                            [&](const exlcm::scalar_i32_t& msg)
                                            {
                                                powerValue = msg.value;
                                                receivedPower = true;
                                            });

    MotorState state{MotorControlMode::Pwm, 200, 0, 50, 1000, false};
    publisher_->publishMotorState(0, state);
    drainMessages(receivedPower);

    EXPECT_TRUE(receivedPower.load());
    EXPECT_EQ(powerValue, 200); // target value published directly
}

TEST_F(DataPublisherTest, PublishMotorStateNegativeTarget)
{
    std::atomic<bool> receivedPower{false};
    int32_t powerValue = 0;

    broker_->subscribe<exlcm::scalar_i32_t>(Channels::motorPower(1),
                                            [&](const exlcm::scalar_i32_t& msg)
                                            {
                                                powerValue = msg.value;
                                                receivedPower = true;
                                            });

    MotorState state{MotorControlMode::Pwm, -150, 0, 0, 0, false};
    publisher_->publishMotorState(1, state);
    drainMessages(receivedPower);

    EXPECT_TRUE(receivedPower.load());
    EXPECT_EQ(powerValue, -150); // Negative target published directly
}

TEST_F(DataPublisherTest, PublishServoStateSucceeds)
{
    ServoState state{ServoMode::Enabled, 1500};

    auto result = publisher_->publishServoState(0, state);
    EXPECT_TRUE(result.isSuccess());
}

TEST_F(DataPublisherTest, PublishServoStateInvalidPort)
{
    ServoState state{};
    auto result = publisher_->publishServoState(MAX_SERVO_PORTS, state);
    EXPECT_TRUE(result.isFailure());
}

TEST_F(DataPublisherTest, PublishMotorPositionSucceeds)
{
    auto result = publisher_->publishMotorPosition(0, 5000);
    EXPECT_TRUE(result.isSuccess());
}

TEST_F(DataPublisherTest, PublishMotorPositionInvalidPort)
{
    auto result = publisher_->publishMotorPosition(MAX_MOTOR_PORTS, 5000);
    EXPECT_TRUE(result.isFailure());
}

TEST_F(DataPublisherTest, PublishMotorDoneSucceeds)
{
    auto result = publisher_->publishMotorDone(0, true);
    EXPECT_TRUE(result.isSuccess());
}

TEST_F(DataPublisherTest, PublishMotorDoneInvalidPort)
{
    auto result = publisher_->publishMotorDone(MAX_MOTOR_PORTS, true);
    EXPECT_TRUE(result.isFailure());
}

TEST_F(DataPublisherTest, PublishShutdownStatus)
{
    auto result = publisher_->publishShutdownStatus(0x03);
    EXPECT_TRUE(result.isSuccess());
    EXPECT_FALSE(logger_->infoMessages.empty());
}

TEST_F(DataPublisherTest, AccuracyPublishedOnFirstCall)
{
    SensorData data{};
    data.accuracy = {3, 3, 3, 3, 3};

    publisher_->publishSensorData(data);

    // First call should log initial accuracy
    bool foundInitial = false;
    for (const auto& msg : logger_->infoMessages)
    {
        if (msg.find("initial") != std::string::npos)
        {
            foundInitial = true;
            break;
        }
    }
    EXPECT_TRUE(foundInitial);
}

TEST_F(DataPublisherTest, AccuracyNotPublishedWhenUnchanged)
{
    SensorData data{};
    data.accuracy = {3, 3, 3, 3, 3};

    publisher_->publishSensorData(data);
    logger_->clear();

    // Second call with same accuracy should not log
    publisher_->publishSensorData(data);

    bool foundAccuracy = false;
    for (const auto& msg : logger_->infoMessages)
    {
        if (msg.find("accuracy") != std::string::npos)
        {
            foundAccuracy = true;
            break;
        }
    }
    EXPECT_FALSE(foundAccuracy);
}

TEST_F(DataPublisherTest, AccuracyPublishedOnChange)
{
    SensorData data{};
    data.accuracy = {3, 3, 3, 3, 3};
    publisher_->publishSensorData(data);
    logger_->clear();

    data.accuracy = {2, 3, 3, 3, 3};
    publisher_->publishSensorData(data);

    bool foundChanged = false;
    for (const auto& msg : logger_->infoMessages)
    {
        if (msg.find("changed") != std::string::npos)
        {
            foundChanged = true;
            break;
        }
    }
    EXPECT_TRUE(foundChanged);
}
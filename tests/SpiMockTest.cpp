#include "wombat/hardware/SpiMock.h"
#include "mocks/MockLogger.h"
#include <gtest/gtest.h>
#include <cmath>

using namespace wombat;
using namespace wombat::test;

class SpiMockTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        logger_ = std::make_shared<MockLogger>();
        Configuration::Spi cfg{};
        spiMock_ = std::make_unique<SpiMock>(cfg, logger_);
    }

    void initializeSpi()
    {
        auto result = spiMock_->initialize();
        ASSERT_TRUE(result.isSuccess());
    }

    std::shared_ptr<MockLogger> logger_;
    std::unique_ptr<SpiMock> spiMock_;
};

TEST_F(SpiMockTest, InitializeSuccess)
{
    auto result = spiMock_->initialize();
    EXPECT_TRUE(result.isSuccess());
}

TEST_F(SpiMockTest, ShutdownSuccess)
{
    initializeSpi();
    auto result = spiMock_->shutdown();
    EXPECT_TRUE(result.isSuccess());
}

TEST_F(SpiMockTest, ReadSensorDataFailsWhenNotInitialized)
{
    auto result = spiMock_->readSensorData();
    EXPECT_TRUE(result.isFailure());
}

TEST_F(SpiMockTest, ReadSensorDataProducesValidData)
{
    initializeSpi();

    auto result = spiMock_->readSensorData();
    ASSERT_TRUE(result.isSuccess());

    const auto& data = result.value();

    // Accelerometer Z should be approximately 1g
    EXPECT_NEAR(data.accelerometer.z, 1.0f, 0.05f);

    // Temperature should be around 28 degrees
    EXPECT_NEAR(data.temperature, 28.0f, 1.0f);

    // Battery voltage should be reasonable
    EXPECT_GT(data.batteryVoltage, 10.0f);
    EXPECT_LT(data.batteryVoltage, 13.0f);

    // IMU accuracy should be calibrated (3)
    EXPECT_EQ(data.accuracy.gyro, 3);
    EXPECT_EQ(data.accuracy.accelerometer, 3);
    EXPECT_EQ(data.accuracy.linearAcceleration, 3);
    EXPECT_EQ(data.accuracy.compass, 3);
}

TEST_F(SpiMockTest, AnalogValuesAreInRange)
{
    initializeSpi();

    auto result = spiMock_->readSensorData();
    ASSERT_TRUE(result.isSuccess());

    for (uint8_t i = 0; i < MAX_ANALOG_PORTS; ++i)
    {
        EXPECT_GE(result.value().analogValues[i], 0);
        EXPECT_LE(result.value().analogValues[i], 4095);
    }
}

TEST_F(SpiMockTest, QuaternionIsNormalized)
{
    initializeSpi();

    auto result = spiMock_->readSensorData();
    ASSERT_TRUE(result.isSuccess());

    const auto& q = result.value().dmpOrientation;
    float norm = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    EXPECT_NEAR(norm, 1.0f, 0.01f);
}

// --- Motor mode tracking ---

TEST_F(SpiMockTest, SetMotorPwmSuccess)
{
    initializeSpi();

    auto result = spiMock_->setMotorPwm(0, 200);
    EXPECT_TRUE(result.isSuccess());

    auto getResult = spiMock_->getMotorState(0);
    ASSERT_TRUE(getResult.isSuccess());
    EXPECT_EQ(getResult.value().controlMode, MotorControlMode::Pwm);
    EXPECT_EQ(getResult.value().target, 200);
}

TEST_F(SpiMockTest, SetMotorOffSuccess)
{
    initializeSpi();

    spiMock_->setMotorPwm(0, 200);
    auto result = spiMock_->setMotorOff(0);
    EXPECT_TRUE(result.isSuccess());

    auto getResult = spiMock_->getMotorState(0);
    ASSERT_TRUE(getResult.isSuccess());
    EXPECT_EQ(getResult.value().controlMode, MotorControlMode::Off);
    EXPECT_EQ(getResult.value().target, 0);
}

TEST_F(SpiMockTest, SetMotorBrakeSuccess)
{
    initializeSpi();

    auto result = spiMock_->setMotorBrake(0);
    EXPECT_TRUE(result.isSuccess());

    auto getResult = spiMock_->getMotorState(0);
    ASSERT_TRUE(getResult.isSuccess());
    EXPECT_EQ(getResult.value().controlMode, MotorControlMode::PassiveBrake);
}

TEST_F(SpiMockTest, SetMotorPwmInvalidPort)
{
    auto result = spiMock_->setMotorPwm(MAX_MOTOR_PORTS, 100);
    EXPECT_TRUE(result.isFailure());
}

TEST_F(SpiMockTest, SetMotorVelocitySetsControlMode)
{
    initializeSpi();

    auto result = spiMock_->setMotorVelocity(1, -300);
    EXPECT_TRUE(result.isSuccess());

    auto getResult = spiMock_->getMotorState(1);
    ASSERT_TRUE(getResult.isSuccess());
    EXPECT_EQ(getResult.value().controlMode, MotorControlMode::MoveAtVelocity);
    EXPECT_EQ(getResult.value().target, -300);
}

TEST_F(SpiMockTest, SetMotorPositionSetsControlMode)
{
    initializeSpi();

    auto result = spiMock_->setMotorPosition(2, 100, 5000);
    EXPECT_TRUE(result.isSuccess());

    auto getResult = spiMock_->getMotorState(2);
    ASSERT_TRUE(getResult.isSuccess());
    EXPECT_EQ(getResult.value().controlMode, MotorControlMode::MoveToPosition);
    EXPECT_EQ(getResult.value().target, 100);
    EXPECT_EQ(getResult.value().goalPosition, 5000);
}

TEST_F(SpiMockTest, GetMotorDoneAllDone)
{
    initializeSpi();

    auto result = spiMock_->getMotorDone();
    ASSERT_TRUE(result.isSuccess());
    EXPECT_EQ(result.value(), 0x0F); // All 4 motors done
}

// --- Servo state tracking ---

TEST_F(SpiMockTest, SetServoStateSuccess)
{
    initializeSpi();

    ServoState state{ServoMode::Enabled, 1500};
    auto result = spiMock_->setServoState(0, state);
    EXPECT_TRUE(result.isSuccess());

    auto getResult = spiMock_->getServoState(0);
    ASSERT_TRUE(getResult.isSuccess());
    EXPECT_EQ(getResult.value().mode, ServoMode::Enabled);
    EXPECT_EQ(getResult.value().position, 1500u);
}

TEST_F(SpiMockTest, SetServoStateInvalidPort)
{
    auto result = spiMock_->setServoState(MAX_SERVO_PORTS, ServoState{});
    EXPECT_TRUE(result.isFailure());
}

// --- BEMF ---

TEST_F(SpiMockTest, BemfProportionalToTarget)
{
    initializeSpi();

    spiMock_->setMotorPwm(0, 400);

    // Trigger a sensor read to update BEMF values
    spiMock_->readSensorData();

    auto getResult = spiMock_->getMotorState(0);
    ASSERT_TRUE(getResult.isSuccess());
    // BEMF should be target/4 = 100
    EXPECT_EQ(getResult.value().backEmf, 100);
}

TEST_F(SpiMockTest, ResetMotorPosition)
{
    initializeSpi();

    spiMock_->setMotorPwm(0, 400);
    spiMock_->readSensorData();

    auto result = spiMock_->resetMotorPosition(0);
    EXPECT_TRUE(result.isSuccess());

    // After reset, position should be 0
    auto getResult = spiMock_->getMotorState(0);
    ASSERT_TRUE(getResult.isSuccess());
    EXPECT_EQ(getResult.value().position, 0);
}

TEST_F(SpiMockTest, ForceUpdateSuccess)
{
    initializeSpi();
    auto result = spiMock_->forceUpdate();
    EXPECT_TRUE(result.isSuccess());
}

TEST_F(SpiMockTest, SetShutdownLogs)
{
    initializeSpi();
    auto result = spiMock_->setShutdown(true);
    EXPECT_TRUE(result.isSuccess());
    EXPECT_FALSE(logger_->infoMessages.empty());
}

// Port validation for motor operations

TEST_F(SpiMockTest, GetMotorPositionInvalidPort)
{
    auto result = spiMock_->getMotorPosition(MAX_MOTOR_PORTS);
    EXPECT_TRUE(result.isFailure());
}

TEST_F(SpiMockTest, SetMotorVelocityInvalidPort)
{
    auto result = spiMock_->setMotorVelocity(MAX_MOTOR_PORTS, 100);
    EXPECT_TRUE(result.isFailure());
}

TEST_F(SpiMockTest, ResetMotorPositionInvalidPort)
{
    auto result = spiMock_->resetMotorPosition(MAX_MOTOR_PORTS);
    EXPECT_TRUE(result.isFailure());
}

TEST_F(SpiMockTest, SetMotorPidInvalidPort)
{
    auto result = spiMock_->setMotorPid(MAX_MOTOR_PORTS, 1.0f, 0.5f, 0.1f);
    EXPECT_TRUE(result.isFailure());
}
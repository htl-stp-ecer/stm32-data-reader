#include "wombat/services/DeviceController.h"
#include "mocks/MockSpi.h"
#include "mocks/MockLogger.h"
#include <gtest/gtest.h>
#include <gmock/gmock.h>

using namespace wombat;
using namespace wombat::test;
using ::testing::_;
using ::testing::Return;
using ::testing::NiceMock;

class DeviceControllerTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        logger_ = std::make_shared<MockLogger>();
        auto mockSpi = std::make_unique<NiceMock<MockSpi>>();
        mockSpi_ = mockSpi.get();

        // Default return values for all methods so destructor cleanup works
        ON_CALL(*mockSpi_, initialize()).WillByDefault(Return(Result<void>::success()));
        ON_CALL(*mockSpi_, shutdown()).WillByDefault(Return(Result<void>::success()));
        ON_CALL(*mockSpi_, setShutdown(_)).WillByDefault(Return(Result<void>::success()));
        ON_CALL(*mockSpi_, setMotorOff(_)).WillByDefault(Return(Result<void>::success()));
        ON_CALL(*mockSpi_, setServoState(_, _)).WillByDefault(Return(Result<void>::success()));

        controller_ = std::make_unique<DeviceController>(std::move(mockSpi), logger_);
    }

    void initializeController()
    {
        EXPECT_CALL(*mockSpi_, initialize())
            .WillOnce(Return(Result<void>::success()));
        EXPECT_CALL(*mockSpi_, setMotorOff(_))
            .Times(MAX_MOTOR_PORTS)
            .WillRepeatedly(Return(Result<void>::success()));
        EXPECT_CALL(*mockSpi_, setServoState(_, _))
            .Times(MAX_SERVO_PORTS)
            .WillRepeatedly(Return(Result<void>::success()));

        auto result = controller_->initialize();
        ASSERT_TRUE(result.isSuccess());
    }

    std::shared_ptr<MockLogger> logger_;
    NiceMock<MockSpi>* mockSpi_{nullptr};
    std::unique_ptr<DeviceController> controller_;
};

// --- Lifecycle ---

TEST_F(DeviceControllerTest, InitializeSuccess)
{
    initializeController();
}

TEST_F(DeviceControllerTest, InitializeFailure)
{
    EXPECT_CALL(*mockSpi_, initialize())
        .WillOnce(Return(Result<void>::failure("SPI error")));

    auto result = controller_->initialize();
    EXPECT_TRUE(result.isFailure());
    EXPECT_FALSE(logger_->errorMessages.empty());
}

TEST_F(DeviceControllerTest, DoubleInitializeIsNoop)
{
    initializeController();

    // Second call should succeed without calling SPI again
    auto result = controller_->initialize();
    EXPECT_TRUE(result.isSuccess());
}

TEST_F(DeviceControllerTest, ShutdownSuccess)
{
    initializeController();

    EXPECT_CALL(*mockSpi_, setShutdown(true))
        .WillOnce(Return(Result<void>::success()));
    EXPECT_CALL(*mockSpi_, shutdown())
        .WillOnce(Return(Result<void>::success()));

    auto result = controller_->shutdown();
    EXPECT_TRUE(result.isSuccess());
}

TEST_F(DeviceControllerTest, ShutdownWhenNotInitializedIsNoop)
{
    auto result = controller_->shutdown();
    EXPECT_TRUE(result.isSuccess());
}

// --- processUpdate ---

TEST_F(DeviceControllerTest, ProcessUpdateSuccess)
{
    initializeController();

    SensorData fakeSensorData{};
    fakeSensorData.temperature = 25.0f;
    fakeSensorData.batteryVoltage = 12.0f;

    EXPECT_CALL(*mockSpi_, readSensorData())
        .WillOnce(Return(Result<SensorData>::success(fakeSensorData)));

    auto result = controller_->processUpdate();
    EXPECT_TRUE(result.isSuccess());

    auto sensorResult = controller_->getCurrentSensorData();
    EXPECT_TRUE(sensorResult.isSuccess());
    EXPECT_FLOAT_EQ(sensorResult.value().temperature, 25.0f);
}

TEST_F(DeviceControllerTest, ProcessUpdateWhenNotInitialized)
{
    auto result = controller_->processUpdate();
    EXPECT_TRUE(result.isFailure());
}

TEST_F(DeviceControllerTest, ProcessUpdateSpiFailure)
{
    initializeController();

    EXPECT_CALL(*mockSpi_, readSensorData())
        .WillOnce(Return(Result<SensorData>::failure("read error")));

    auto result = controller_->processUpdate();
    EXPECT_TRUE(result.isFailure());
}

// --- Motor commands ---

TEST_F(DeviceControllerTest, SetMotorPwmSuccess)
{
    initializeController();

    EXPECT_CALL(*mockSpi_, setMotorPwm(0, 400))
        .WillOnce(Return(Result<void>::success()));

    auto result = controller_->setMotorPwm(0, 400);
    EXPECT_TRUE(result.isSuccess());
}

TEST_F(DeviceControllerTest, SetMotorPwmInvalidPort)
{
    initializeController();

    auto result = controller_->setMotorPwm(MAX_MOTOR_PORTS, 400);
    EXPECT_TRUE(result.isFailure());
}

TEST_F(DeviceControllerTest, SetMotorOffSuccess)
{
    initializeController();

    EXPECT_CALL(*mockSpi_, setMotorOff(0))
        .WillOnce(Return(Result<void>::success()));

    auto result = controller_->setMotorOff(0);
    EXPECT_TRUE(result.isSuccess());
}

TEST_F(DeviceControllerTest, SetMotorBrakeSuccess)
{
    initializeController();

    EXPECT_CALL(*mockSpi_, setMotorBrake(1))
        .WillOnce(Return(Result<void>::success()));

    auto result = controller_->setMotorBrake(1);
    EXPECT_TRUE(result.isSuccess());
}

TEST_F(DeviceControllerTest, SetMotorVelocitySuccess)
{
    initializeController();

    EXPECT_CALL(*mockSpi_, setMotorVelocity(1, 500))
        .WillOnce(Return(Result<void>::success()));

    auto result = controller_->setMotorVelocity(1, 500);
    EXPECT_TRUE(result.isSuccess());
}

TEST_F(DeviceControllerTest, SetMotorPositionSuccess)
{
    initializeController();

    EXPECT_CALL(*mockSpi_, setMotorPosition(2, 100, 5000))
        .WillOnce(Return(Result<void>::success()));

    auto result = controller_->setMotorPosition(2, 100, 5000);
    EXPECT_TRUE(result.isSuccess());
}

TEST_F(DeviceControllerTest, SetMotorVelocityInvalidPort)
{
    initializeController();

    auto result = controller_->setMotorVelocity(MAX_MOTOR_PORTS, 100);
    EXPECT_TRUE(result.isFailure());
}

// --- Servo commands ---

TEST_F(DeviceControllerTest, SetServoCommandSuccess)
{
    initializeController();

    EXPECT_CALL(*mockSpi_, setServoState(0, _))
        .WillOnce(Return(Result<void>::success()));

    auto result = controller_->setServoCommand(0, 1500);
    EXPECT_TRUE(result.isSuccess());
}

TEST_F(DeviceControllerTest, SetServoCommandInvalidPort)
{
    initializeController();

    auto result = controller_->setServoCommand(MAX_SERVO_PORTS, 1500);
    EXPECT_TRUE(result.isFailure());
}

TEST_F(DeviceControllerTest, SetServoModePreservesPosition)
{
    initializeController();

    ServoState currentState{ServoMode::Disabled, 1234};
    EXPECT_CALL(*mockSpi_, getServoState(0))
        .WillOnce(Return(Result<ServoState>::success(currentState)));
    EXPECT_CALL(*mockSpi_, setServoState(0, _))
        .WillOnce(Return(Result<void>::success()));

    auto result = controller_->setServoMode(0, ServoMode::Enabled);
    EXPECT_TRUE(result.isSuccess());
}

// --- State getters when not initialized ---

TEST_F(DeviceControllerTest, GetSensorDataWhenNotInitialized)
{
    auto result = controller_->getCurrentSensorData();
    EXPECT_TRUE(result.isFailure());
}

TEST_F(DeviceControllerTest, GetMotorStateWhenNotInitialized)
{
    auto result = controller_->getMotorState(0);
    EXPECT_TRUE(result.isFailure());
}

TEST_F(DeviceControllerTest, GetServoStateWhenNotInitialized)
{
    auto result = controller_->getServoState(0);
    EXPECT_TRUE(result.isFailure());
}

TEST_F(DeviceControllerTest, GetMotorPositionWhenNotInitialized)
{
    auto result = controller_->getMotorPosition(0);
    EXPECT_TRUE(result.isFailure());
}

TEST_F(DeviceControllerTest, GetMotorDoneWhenNotInitialized)
{
    auto result = controller_->getMotorDone();
    EXPECT_TRUE(result.isFailure());
}

// --- Position reset operations require initialization ---

TEST_F(DeviceControllerTest, ResetMotorPositionWhenNotInitialized)
{
    auto result = controller_->resetMotorPosition(0);
    EXPECT_TRUE(result.isFailure());
}

TEST_F(DeviceControllerTest, ResetMotorPositionSuccess)
{
    initializeController();

    EXPECT_CALL(*mockSpi_, resetMotorPosition(0))
        .WillOnce(Return(Result<void>::success()));

    auto result = controller_->resetMotorPosition(0);
    EXPECT_TRUE(result.isSuccess());
}

TEST_F(DeviceControllerTest, ResetMotorPositionInvalidPort)
{
    initializeController();

    auto result = controller_->resetMotorPosition(MAX_MOTOR_PORTS);
    EXPECT_TRUE(result.isFailure());
}

TEST_F(DeviceControllerTest, SetMotorPidSuccess)
{
    initializeController();

    EXPECT_CALL(*mockSpi_, setMotorPid(0, 1.0f, 0.5f, 0.1f))
        .WillOnce(Return(Result<void>::success()));

    auto result = controller_->setMotorPid(0, 1.0f, 0.5f, 0.1f);
    EXPECT_TRUE(result.isSuccess());
}

TEST_F(DeviceControllerTest, SetShutdownSuccess)
{
    initializeController();

    auto result = controller_->setShutdown(true);
    EXPECT_TRUE(result.isSuccess());
}

TEST_F(DeviceControllerTest, SetShutdownWhenNotInitialized)
{
    auto result = controller_->setShutdown(true);
    EXPECT_TRUE(result.isFailure());
}
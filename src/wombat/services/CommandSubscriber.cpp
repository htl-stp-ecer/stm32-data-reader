#include "wombat/services/CommandSubscriber.h"

namespace wombat
{
    CommandSubscriber::CommandSubscriber(std::shared_ptr<LcmBroker> broker,
                                         std::shared_ptr<DeviceController> deviceController,
                                         std::shared_ptr<DataPublisher> dataPublisher,
                                         std::shared_ptr<Logger> logger)
        : broker_{std::move(broker)},
          deviceController_{std::move(deviceController)},
          dataPublisher_{std::move(dataPublisher)},
          logger_{std::move(logger)}
    {
    }

    Result<void> CommandSubscriber::initialize()
    {
        if (isInitialized_)
        {
            return Result<void>::success();
        }

        // Subscribe to motor power commands
        for (uint8_t i = 0; i < MAX_MOTOR_PORTS; ++i)
        {
            logger_->info("Subscribing to motor power command channel: " + Channels::motorPowerCommand(i));
            auto motorResult = broker_->subscribeScalarI32(
                Channels::motorPowerCommand(i),
                [this, i](const exlcm::scalar_i32_t& cmd) { onMotorPowerCommand(i, cmd); }
            );
            if (motorResult.isFailure())
            {
                return Result<void>::failure("Failed to subscribe to motor commands: " + motorResult.error());
            }
        }

        // Subscribe to servo position commands
        const auto servoResult = broker_->subscribeScalarI32(
            Channels::servoPositionCommand(0),
            [this](const exlcm::scalar_i32_t& cmd) { onServoPositionCommand(cmd); }
        );
        if (servoResult.isFailure())
        {
            return Result<void>::failure("Failed to subscribe to servo commands: " + servoResult.error());
        }

        // Subscribe to data dump request command
        logger_->info("Subscribing to data dump request channel: " + std::string(Channels::DATA_DUMP_REQUEST));
        const auto dumpResult = broker_->subscribeScalarI32(
            Channels::DATA_DUMP_REQUEST,
            [this](const exlcm::scalar_i32_t& cmd) { onDataDumpRequest(cmd); }
        );
        if (dumpResult.isFailure())
        {
            return Result<void>::failure("Failed to subscribe to data dump request: " + dumpResult.error());
        }

        isInitialized_ = true;
        logger_->info("Command subscriber initialized successfully");
        return Result<void>::success();
    }

    Result<void> CommandSubscriber::shutdown()
    {
        if (!isInitialized_)
        {
            return Result<void>::success();
        }

        isInitialized_ = false;
        logger_->info("Command subscriber shut down");
        return Result<void>::success();
    }

    void CommandSubscriber::onMotorPowerCommand(const PortId port, const exlcm::scalar_i32_t& command) const
    {
        logger_->debug("onMotorPowerCommand called with value: " + std::to_string(command.value));
        if (!isInitialized_)
        {
            logger_->warn("Received motor command while not initialized");
            return;
        }

        const int32_t powerValue = command.value;
        const auto direction = powerValue >= 0 ? MotorDirection::Clockwise : MotorDirection::CounterClockwise;
        const auto speed = static_cast<MotorSpeed>(std::abs(powerValue));

        const auto result = deviceController_->setMotorCommand(port, direction, speed);
        if (result.isFailure())
        {
            logger_->error("Failed to set motor command: " + result.error());
            return;
        }

        // Force hardware update
        auto forceResult = deviceController_->processUpdate();
        if (forceResult.isFailure())
        {
            logger_->error("Failed to force device update after motor command: " + forceResult.error());
        }

        logger_->info("Received motor power_cmd: " + std::to_string(powerValue));
    }

    void CommandSubscriber::onServoPositionCommand(const exlcm::scalar_i32_t& command)
    {
        if (!isInitialized_)
        {
            logger_->warn("Received servo command while not initialized");
            return;
        }

        const auto position = static_cast<ServoPosition>(command.value);

        auto result = deviceController_->setServoCommand(0, position);
        if (result.isFailure())
        {
            logger_->error("Failed to set servo command: " + result.error());
            return;
        }

        // Force hardware update
        auto forceResult = deviceController_->processUpdate();
        if (forceResult.isFailure())
        {
            logger_->error("Failed to force device update after servo command: " + forceResult.error());
        }

        logger_->info("Received servo position_cmd: " + std::to_string(command.value));
    }

    void CommandSubscriber::onDataDumpRequest(const exlcm::scalar_i32_t& command) const
    {
        if (!isInitialized_)
        {
            logger_->warn("Received data dump request while not initialized");
            return;
        }

        logger_->info("Received data dump request, publishing all current values");

        // Get current sensor data and publish
        auto sensorDataResult = deviceController_->getCurrentSensorData();
        if (sensorDataResult.isSuccess())
        {
            dataPublisher_->publishSensorData(sensorDataResult.value());
        }
        else
        {
            logger_->warn("Failed to get sensor data for dump: " + sensorDataResult.error());
        }

        // Publish all motor states
        for (PortId port = 0; port < MAX_MOTOR_PORTS; ++port)
        {
            auto motorStateResult = deviceController_->getMotorState(port);
            if (motorStateResult.isSuccess())
            {
                dataPublisher_->publishMotorState(port, motorStateResult.value());
            }
        }

        // Publish all servo states
        for (PortId port = 0; port < MAX_SERVO_PORTS; ++port)
        {
            auto servoStateResult = deviceController_->getServoState(port);
            if (servoStateResult.isSuccess())
            {
                dataPublisher_->publishServoState(port, servoStateResult.value());
            }
        }

        logger_->info("Data dump completed");
    }
} // namespace wombat

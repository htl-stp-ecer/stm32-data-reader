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

            logger_->info("Subscribing to motor stop command channel: " + Channels::motorStopCommand(i));
            auto stopResult = broker_->subscribeScalarI32(
                Channels::motorStopCommand(i),
                [this, i](const exlcm::scalar_i32_t& cmd) { onMotorStopCommand(i, cmd); }
            );
            if (stopResult.isFailure())
            {
                return Result<void>::failure("Failed to subscribe to motor stop commands: " + stopResult.error());
            }
        }

        // Subscribe to BEMF reset commands
        for (uint8_t i = 0; i < MAX_MOTOR_PORTS; ++i)
        {
            logger_->info("Subscribing to BEMF reset command channel: " + Channels::bemfResetCommand(i));
            auto bemfResetResult = broker_->subscribeScalarI32(
                Channels::bemfResetCommand(i),
                [this, i](const exlcm::scalar_i32_t& cmd) { onBemfResetCommand(i, cmd); }
            );
            if (bemfResetResult.isFailure())
            {
                return Result<void>::failure("Failed to subscribe to BEMF reset commands: " + bemfResetResult.error());
            }

            // Subscribe to BEMF scale commands
            logger_->info("Subscribing to BEMF scale command channel: " + Channels::bemfScaleCommand(i));
            auto bemfScaleResult = broker_->subscribeScalarF(
                Channels::bemfScaleCommand(i),
                [this, i](const exlcm::scalar_f_t& cmd) { onBemfScaleCommand(i, cmd); }
            );
            if (bemfScaleResult.isFailure())
            {
                return Result<void>::failure("Failed to subscribe to BEMF scale commands: " + bemfScaleResult.error());
            }

            // Subscribe to BEMF offset commands
            logger_->info("Subscribing to BEMF offset command channel: " + Channels::bemfOffsetCommand(i));
            auto bemfOffsetResult = broker_->subscribeScalarF(
                Channels::bemfOffsetCommand(i),
                [this, i](const exlcm::scalar_f_t& cmd) { onBemfOffsetCommand(i, cmd); }
            );
            if (bemfOffsetResult.isFailure())
            {
                return Result<void>::failure("Failed to subscribe to BEMF offset commands: " + bemfOffsetResult.error());
            }
        }

        // Subscribe to BEMF nominal voltage command
        logger_->info("Subscribing to BEMF nominal voltage command channel: " + std::string(Channels::BEMF_NOMINAL_VOLTAGE_CMD));
        auto nominalVoltageResult = broker_->subscribeScalarI32(
            Channels::BEMF_NOMINAL_VOLTAGE_CMD,
            [this](const exlcm::scalar_i32_t& cmd) { onBemfNominalVoltageCommand(cmd); }
        );
        if (nominalVoltageResult.isFailure())
        {
            return Result<void>::failure("Failed to subscribe to BEMF nominal voltage command: " + nominalVoltageResult.error());
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

        logger_->info("Received motor power_cmd on port " + std::to_string(port) + ": " + std::to_string(command.value));
    }

    void CommandSubscriber::onMotorStopCommand(const PortId port, const exlcm::scalar_i32_t& command) const
    {
        if (!isInitialized_)
        {
            logger_->warn("Received motor stop command while not initialized");
            return;
        }

        const bool engageStop = command.value != 0;
        auto result = deviceController_->setMotorStop(port, engageStop);
        if (result.isFailure())
        {
            logger_->error("Failed to apply motor stop command on port " + std::to_string(port) + ": " + result.error());
            return;
        }

        // Force hardware update so stop/wake applies immediately
        auto forceResult = deviceController_->processUpdate();
        if (forceResult.isFailure())
        {
            logger_->error("Failed to force device update after motor stop command: " + forceResult.error());
        }

        const std::string action = engageStop ? "stop engaged" : "wake-up received";
        logger_->info("Motor " + std::to_string(port) + " " + action);
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

    void CommandSubscriber::onBemfResetCommand(const PortId port, const exlcm::scalar_i32_t& command) const
    {
        if (!isInitialized_)
        {
            logger_->warn("Received BEMF reset command while not initialized");
            return;
        }

        if (command.value == 0)
        {
            logger_->debug("Ignoring BEMF reset command with zero value for motor " + std::to_string(port));
            return;
        }

        const auto result = deviceController_->resetBemfSum(port);
        if (result.isFailure())
        {
            logger_->error("Failed to reset BEMF sum for motor " + std::to_string(port) + ": " + result.error());
            return;
        }

        logger_->info("Reset BEMF sum for motor " + std::to_string(port));
    }

    void CommandSubscriber::onBemfScaleCommand(const PortId port, const exlcm::scalar_f_t& command) const
    {
        if (!isInitialized_)
        {
            logger_->warn("Received BEMF scale command while not initialized");
            return;
        }

        const auto result = deviceController_->setBemfScale(port, command.value);
        if (result.isFailure())
        {
            logger_->error("Failed to set BEMF scale for motor " + std::to_string(port) + ": " + result.error());
            return;
        }

        // Force hardware update
        auto forceResult = deviceController_->processUpdate();
        if (forceResult.isFailure())
        {
            logger_->error("Failed to force device update after BEMF scale command: " + forceResult.error());
        }

        logger_->info("Set BEMF scale for motor " + std::to_string(port) + " to " + std::to_string(command.value));
    }

    void CommandSubscriber::onBemfOffsetCommand(const PortId port, const exlcm::scalar_f_t& command) const
    {
        if (!isInitialized_)
        {
            logger_->warn("Received BEMF offset command while not initialized");
            return;
        }

        const auto result = deviceController_->setBemfOffset(port, command.value);
        if (result.isFailure())
        {
            logger_->error("Failed to set BEMF offset for motor " + std::to_string(port) + ": " + result.error());
            return;
        }

        // Force hardware update
        auto forceResult = deviceController_->processUpdate();
        if (forceResult.isFailure())
        {
            logger_->error("Failed to force device update after BEMF offset command: " + forceResult.error());
        }

        logger_->info("Set BEMF offset for motor " + std::to_string(port) + " to " + std::to_string(command.value));
    }

    void CommandSubscriber::onBemfNominalVoltageCommand(const exlcm::scalar_i32_t& command) const
    {
        if (!isInitialized_)
        {
            logger_->warn("Received BEMF nominal voltage command while not initialized");
            return;
        }

        const auto result = deviceController_->setBemfNominalVoltage(static_cast<int16_t>(command.value));
        if (result.isFailure())
        {
            logger_->error("Failed to set BEMF nominal voltage: " + result.error());
            return;
        }

        // Force hardware update
        auto forceResult = deviceController_->processUpdate();
        if (forceResult.isFailure())
        {
            logger_->error("Failed to force device update after BEMF nominal voltage command: " + forceResult.error());
        }

        logger_->info("Set BEMF nominal voltage ADC to " + std::to_string(command.value));
    }
} // namespace wombat

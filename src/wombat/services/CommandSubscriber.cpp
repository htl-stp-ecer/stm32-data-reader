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

        // Subscribe to servo position and mode commands for all ports
        for (uint8_t i = 0; i < MAX_SERVO_PORTS; ++i)
        {
            logger_->info("Subscribing to servo position command channel: " + Channels::servoPositionCommand(i));
            auto posResult = broker_->subscribeScalarI32(
                Channels::servoPositionCommand(i),
                [this, i](const exlcm::scalar_i32_t& cmd) { onServoPositionCommand(i, cmd); }
            );
            if (posResult.isFailure())
            {
                return Result<void>::failure("Failed to subscribe to servo position commands: " + posResult.error());
            }

            logger_->info("Subscribing to servo mode command channel: " + Channels::servoMode(i));
            auto modeResult = broker_->subscribeScalarI8(
                Channels::servoMode(i),
                [this, i](const exlcm::scalar_i8_t& cmd) { onServoModeCommand(i, cmd); }
            );
            if (modeResult.isFailure())
            {
                return Result<void>::failure("Failed to subscribe to servo mode commands: " + modeResult.error());
            }
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

        // Subscribe to shutdown command
        logger_->info("Subscribing to shutdown command channel: " + std::string(Channels::SHUTDOWN_CMD));
        const auto shutdownResult = broker_->subscribeScalarI32(
            Channels::SHUTDOWN_CMD,
            [this](const exlcm::scalar_i32_t& cmd) { onShutdownCommand(cmd); }
        );
        if (shutdownResult.isFailure())
        {
            return Result<void>::failure("Failed to subscribe to shutdown command: " + shutdownResult.error());
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

        // When speed is 0, use active braking instead of coasting
        MotorDirection direction;
        if (powerValue == 0) {
            direction = MotorDirection::Brake;
        } else if (powerValue > 0) {
            direction = MotorDirection::Clockwise;
        } else {
            direction = MotorDirection::CounterClockwise;
        }
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

    void CommandSubscriber::onServoPositionCommand(const PortId port, const exlcm::scalar_i32_t& command)
    {
        if (!isInitialized_)
        {
            logger_->warn("Received servo command while not initialized");
            return;
        }

        const auto position = static_cast<ServoPosition>(command.value);

        auto result = deviceController_->setServoCommand(port, position);
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

        logger_->info("Received servo position_cmd on port " + std::to_string(port) + ": " + std::to_string(command.value));
    }

    void CommandSubscriber::onServoModeCommand(const PortId port, const exlcm::scalar_i8_t& command)
    {
        if (!isInitialized_)
        {
            logger_->warn("Received servo mode command while not initialized");
            return;
        }

        const auto mode = static_cast<ServoMode>(command.dir);

        auto result = deviceController_->setServoMode(port, mode);
        if (result.isFailure())
        {
            logger_->error("Failed to set servo mode: " + result.error());
            return;
        }

        // Force hardware update
        auto forceResult = deviceController_->processUpdate();
        if (forceResult.isFailure())
        {
            logger_->error("Failed to force device update after servo mode command: " + forceResult.error());
        }

        logger_->info("Received servo mode_cmd on port " + std::to_string(port) + ": " + std::to_string(command.dir));
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

    void CommandSubscriber::onShutdownCommand(const exlcm::scalar_i32_t& command) const
    {
        if (!isInitialized_)
        {
            logger_->warn("Received shutdown command while not initialized");
            return;
        }

        const bool enabled = command.value != 0;
        const auto result = deviceController_->setShutdown(enabled);
        if (result.isFailure())
        {
            logger_->error("Failed to set shutdown: " + result.error());
            return;
        }

        // Shutdown flag is handled entirely by STM32 firmware - no need for Pi-side device update

        // Publish shutdown status so subscribers (like the UI) can react
        // Bitmask: bit 0 = servo shutdown, bit 1 = motor shutdown (both enabled/disabled together)
        const uint8_t shutdownFlags = enabled ? 0x03 : 0x00;
        dataPublisher_->publishShutdownStatus(shutdownFlags);

        logger_->info("Shutdown " + std::string(enabled ? "enabled" : "disabled"));
    }
} // namespace wombat

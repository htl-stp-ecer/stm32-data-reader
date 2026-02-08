#include "wombat/services/CommandSubscriber.h"
#include <chrono>

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

        // Subscribe to motor velocity commands
        for (uint8_t i = 0; i < MAX_MOTOR_PORTS; ++i)
        {
            logger_->info("Subscribing to motor velocity command channel: " + Channels::motorVelocityCommand(i));
            auto velocityResult = broker_->subscribeScalarI32(
                Channels::motorVelocityCommand(i),
                [this, i](const exlcm::scalar_i32_t& cmd) { onMotorVelocityCommand(i, cmd); }
            );
            if (velocityResult.isFailure())
            {
                return Result<void>::failure("Failed to subscribe to motor velocity commands: " + velocityResult.error());
            }
        }

        // Subscribe to motor position commands (vector3f: x=velocity, y=goal_position)
        for (uint8_t i = 0; i < MAX_MOTOR_PORTS; ++i)
        {
            logger_->info("Subscribing to motor position command channel: " + Channels::motorPositionCommand(i));
            auto positionResult = broker_->subscribeVector3f(
                Channels::motorPositionCommand(i),
                [this, i](const exlcm::vector3f_t& cmd) { onMotorPositionCommand(i, cmd); }
            );
            if (positionResult.isFailure())
            {
                return Result<void>::failure("Failed to subscribe to motor position commands: " + positionResult.error());
            }
        }

        // Subscribe to motor relative commands (vector3f: x=velocity, y=delta_position)
        for (uint8_t i = 0; i < MAX_MOTOR_PORTS; ++i)
        {
            logger_->info("Subscribing to motor relative command channel: " + Channels::motorRelativeCommand(i));
            auto relativeResult = broker_->subscribeVector3f(
                Channels::motorRelativeCommand(i),
                [this, i](const exlcm::vector3f_t& cmd) { onMotorRelativeCommand(i, cmd); }
            );
            if (relativeResult.isFailure())
            {
                return Result<void>::failure("Failed to subscribe to motor relative commands: " + relativeResult.error());
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

        // Subscribe to motor PID commands (vector3f: x=kp, y=ki, z=kd)
        for (uint8_t i = 0; i < MAX_MOTOR_PORTS; ++i)
        {
            logger_->info("Subscribing to motor PID command channel: " + Channels::motorPidCommand(i));
            auto pidResult = broker_->subscribeVector3f(
                Channels::motorPidCommand(i),
                [this, i](const exlcm::vector3f_t& cmd) { onMotorPidCommand(i, cmd); }
            );
            if (pidResult.isFailure())
            {
                return Result<void>::failure("Failed to subscribe to motor PID commands: " + pidResult.error());
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

    bool CommandSubscriber::isTimestampNewer(const std::string& channel, int64_t timestamp)
    {
        auto it = latestTimestamps_.find(channel);
        if (it == latestTimestamps_.end())
        {
            latestTimestamps_[channel] = timestamp;
            return true;
        }

        if (timestamp > it->second)
        {
            it->second = timestamp;
            return true;
        }

        logger_->debug("Dropping stale message on " + channel +
            " (ts=" + std::to_string(timestamp) +
            " <= latest=" + std::to_string(it->second) + ")");
        return false;
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

    void CommandSubscriber::onMotorPowerCommand(const PortId port, const exlcm::scalar_i32_t& command)
    {
        const auto nowUs = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        const auto lcmLatencyUs = nowUs - command.timestamp;
        if (!isInitialized_)
        {
            logger_->warn("Received motor command while not initialized");
            return;
        }

        if (!isTimestampNewer(Channels::motorPowerCommand(port), command.timestamp))
            return;

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
    }

    void CommandSubscriber::onMotorVelocityCommand(const PortId port, const exlcm::scalar_i32_t& command)
    {
        if (!isInitialized_)
        {
            logger_->warn("Received motor velocity command while not initialized");
            return;
        }

        if (!isTimestampNewer(Channels::motorVelocityCommand(port), command.timestamp))
            return;

        const auto result = deviceController_->setMotorVelocity(port, command.value);
        if (result.isFailure())
        {
            logger_->error("Failed to set motor velocity: " + result.error());
            return;
        }

        logger_->info("Received motor velocity_cmd on port " + std::to_string(port) + ": " + std::to_string(command.value));
    }

    void CommandSubscriber::onMotorPositionCommand(const PortId port, const exlcm::vector3f_t& command)
    {
        if (!isInitialized_)
        {
            logger_->warn("Received motor position command while not initialized");
            return;
        }

        if (!isTimestampNewer(Channels::motorPositionCommand(port), command.timestamp))
            return;

        const int32_t velocity = static_cast<int32_t>(command.x);
        const int32_t goalPosition = static_cast<int32_t>(command.y);

        const auto result = deviceController_->setMotorPosition(port, velocity, goalPosition);
        if (result.isFailure())
        {
            logger_->error("Failed to set motor position: " + result.error());
            return;
        }

        logger_->info("Received motor position_cmd on port " + std::to_string(port) +
            ": velocity=" + std::to_string(velocity) + ", goal=" + std::to_string(goalPosition));
    }

    void CommandSubscriber::onMotorRelativeCommand(const PortId port, const exlcm::vector3f_t& command)
    {
        if (!isInitialized_)
        {
            logger_->warn("Received motor relative command while not initialized");
            return;
        }

        if (!isTimestampNewer(Channels::motorRelativeCommand(port), command.timestamp))
            return;

        const int32_t velocity = static_cast<int32_t>(command.x);
        const int32_t deltaPosition = static_cast<int32_t>(command.y);

        const auto result = deviceController_->setMotorRelative(port, velocity, deltaPosition);
        if (result.isFailure())
        {
            logger_->error("Failed to set motor relative: " + result.error());
            return;
        }

        logger_->info("Received motor relative_cmd on port " + std::to_string(port) +
            ": velocity=" + std::to_string(velocity) + ", delta=" + std::to_string(deltaPosition));
    }

    void CommandSubscriber::onServoPositionCommand(const PortId port, const exlcm::scalar_i32_t& command)
    {
        if (!isInitialized_)
        {
            logger_->warn("Received servo command while not initialized");
            return;
        }

        if (!isTimestampNewer(Channels::servoPositionCommand(port), command.timestamp))
            return;

        const auto position = static_cast<ServoPosition>(command.value);

        auto result = deviceController_->setServoCommand(port, position);
        if (result.isFailure())
        {
            logger_->error("Failed to set servo command: " + result.error());
            return;
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

        if (!isTimestampNewer(Channels::servoMode(port), command.timestamp))
            return;

        const auto mode = static_cast<ServoMode>(command.dir);

        auto result = deviceController_->setServoMode(port, mode);
        if (result.isFailure())
        {
            logger_->error("Failed to set servo mode: " + result.error());
            return;
        }
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

    void CommandSubscriber::onBemfResetCommand(const PortId port, const exlcm::scalar_i32_t& command)
    {
        if (!isInitialized_)
        {
            logger_->warn("Received BEMF reset command while not initialized");
            return;
        }

        if (!isTimestampNewer(Channels::bemfResetCommand(port), command.timestamp))
            return;

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

    void CommandSubscriber::onBemfScaleCommand(const PortId port, const exlcm::scalar_f_t& command)
    {
        if (!isInitialized_)
        {
            logger_->warn("Received BEMF scale command while not initialized");
            return;
        }

        if (!isTimestampNewer(Channels::bemfScaleCommand(port), command.timestamp))
            return;

        const auto result = deviceController_->setBemfScale(port, command.value);
        if (result.isFailure())
        {
            logger_->error("Failed to set BEMF scale for motor " + std::to_string(port) + ": " + result.error());
            return;
        }

        logger_->info("Set BEMF scale for motor " + std::to_string(port) + " to " + std::to_string(command.value));
    }

    void CommandSubscriber::onBemfOffsetCommand(const PortId port, const exlcm::scalar_f_t& command)
    {
        if (!isInitialized_)
        {
            logger_->warn("Received BEMF offset command while not initialized");
            return;
        }

        if (!isTimestampNewer(Channels::bemfOffsetCommand(port), command.timestamp))
            return;

        const auto result = deviceController_->setBemfOffset(port, command.value);
        if (result.isFailure())
        {
            logger_->error("Failed to set BEMF offset for motor " + std::to_string(port) + ": " + result.error());
            return;
        }

        logger_->info("Set BEMF offset for motor " + std::to_string(port) + " to " + std::to_string(command.value));
    }

    void CommandSubscriber::onBemfNominalVoltageCommand(const exlcm::scalar_i32_t& command)
    {
        if (!isInitialized_)
        {
            logger_->warn("Received BEMF nominal voltage command while not initialized");
            return;
        }

        if (!isTimestampNewer(Channels::BEMF_NOMINAL_VOLTAGE_CMD, command.timestamp))
            return;

        const auto result = deviceController_->setBemfNominalVoltage(static_cast<int16_t>(command.value));
        if (result.isFailure())
        {
            logger_->error("Failed to set BEMF nominal voltage: " + result.error());
            return;
        }

        logger_->info("Set BEMF nominal voltage ADC to " + std::to_string(command.value));
    }

    void CommandSubscriber::onMotorPidCommand(const PortId port, const exlcm::vector3f_t& command)
    {
        if (!isInitialized_)
        {
            logger_->warn("Received motor PID command while not initialized");
            return;
        }

        if (!isTimestampNewer(Channels::motorPidCommand(port), command.timestamp))
            return;

        const float kp = command.x;
        const float ki = command.y;
        const float kd = command.z;

        const auto result = deviceController_->setMotorPid(port, kp, ki, kd);
        if (result.isFailure())
        {
            logger_->error("Failed to set motor PID: " + result.error());
            return;
        }

        logger_->info("Received motor pid_cmd on port " + std::to_string(port) +
            ": kp=" + std::to_string(kp) + ", ki=" + std::to_string(ki) + ", kd=" + std::to_string(kd));
    }

    void CommandSubscriber::onShutdownCommand(const exlcm::scalar_i32_t& command)
    {
        if (!isInitialized_)
        {
            logger_->warn("Received shutdown command while not initialized");
            return;
        }

        if (!isTimestampNewer(Channels::SHUTDOWN_CMD, command.timestamp))
            return;

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

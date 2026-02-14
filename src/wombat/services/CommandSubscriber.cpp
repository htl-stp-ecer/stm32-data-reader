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

    template <LcmMessage MsgT>
    Result<void> CommandSubscriber::subscribeForPorts(
        PortId maxPorts,
        std::function<std::string(PortId)> channelFn,
        std::function<void(PortId, const MsgT &)> handler,
        const std::string& description)
    {
        for (PortId i = 0; i < maxPorts; ++i)
        {
            const auto channel = channelFn(i);
            logger_->info("Subscribing to " + description + " channel: " + channel);
            auto result = broker_->subscribe<MsgT>(
                channel,
                [handler, i](const MsgT& cmd) { handler(i, cmd); }
            );
            if (result.isFailure())
            {
                return Result<void>::failure("Failed to subscribe to " + description + ": " + result.error());
            }
        }
        return Result<void>::success();
    }

    Result<void> CommandSubscriber::initialize()
    {
        if (isInitialized_)
        {
            return Result<void>::success();
        }

        // Motor commands (per-port)
        auto r = subscribeForPorts<exlcm::scalar_i32_t>(
            MAX_MOTOR_PORTS, Channels::motorPowerCommand,
            [this](PortId p, const exlcm::scalar_i32_t& cmd) { onMotorPowerCommand(p, cmd); },
            "motor power command");
        if (r.isFailure()) return r;

        r = subscribeForPorts<exlcm::scalar_i32_t>(
            MAX_MOTOR_PORTS, Channels::motorVelocityCommand,
            [this](PortId p, const exlcm::scalar_i32_t& cmd) { onMotorVelocityCommand(p, cmd); },
            "motor velocity command");
        if (r.isFailure()) return r;

        r = subscribeForPorts<exlcm::vector3f_t>(
            MAX_MOTOR_PORTS, Channels::motorPositionCommand,
            [this](PortId p, const exlcm::vector3f_t& cmd) { onMotorPositionCommand(p, cmd); },
            "motor position command");
        if (r.isFailure()) return r;

        r = subscribeForPorts<exlcm::vector3f_t>(
            MAX_MOTOR_PORTS, Channels::motorRelativeCommand,
            [this](PortId p, const exlcm::vector3f_t& cmd) { onMotorRelativeCommand(p, cmd); },
            "motor relative command");
        if (r.isFailure()) return r;

        r = subscribeForPorts<exlcm::vector3f_t>(
            MAX_MOTOR_PORTS, Channels::motorPidCommand,
            [this](PortId p, const exlcm::vector3f_t& cmd) { onMotorPidCommand(p, cmd); },
            "motor PID command");
        if (r.isFailure()) return r;

        // BEMF commands (per-port)
        r = subscribeForPorts<exlcm::scalar_i32_t>(
            MAX_MOTOR_PORTS, Channels::bemfResetCommand,
            [this](PortId p, const exlcm::scalar_i32_t& cmd) { onBemfResetCommand(p, cmd); },
            "BEMF reset command");
        if (r.isFailure()) return r;

        r = subscribeForPorts<exlcm::scalar_f_t>(
            MAX_MOTOR_PORTS, Channels::bemfScaleCommand,
            [this](PortId p, const exlcm::scalar_f_t& cmd) { onBemfScaleCommand(p, cmd); },
            "BEMF scale command");
        if (r.isFailure()) return r;

        r = subscribeForPorts<exlcm::scalar_f_t>(
            MAX_MOTOR_PORTS, Channels::bemfOffsetCommand,
            [this](PortId p, const exlcm::scalar_f_t& cmd) { onBemfOffsetCommand(p, cmd); },
            "BEMF offset command");
        if (r.isFailure()) return r;

        // BEMF nominal voltage (single channel)
        logger_->info(
            "Subscribing to BEMF nominal voltage command channel: " + std::string(Channels::BEMF_NOMINAL_VOLTAGE_CMD));
        auto nominalResult = broker_->subscribe<exlcm::scalar_i32_t>(
            Channels::BEMF_NOMINAL_VOLTAGE_CMD,
            [this](const exlcm::scalar_i32_t& cmd) { onBemfNominalVoltageCommand(cmd); }
        );
        if (nominalResult.isFailure())
        {
            return Result<void>::failure(
                "Failed to subscribe to BEMF nominal voltage command: " + nominalResult.error());
        }

        // Servo commands (per-port)
        r = subscribeForPorts<exlcm::scalar_i32_t>(
            MAX_SERVO_PORTS, Channels::servoPositionCommand,
            [this](PortId p, const exlcm::scalar_i32_t& cmd) { onServoPositionCommand(p, cmd); },
            "servo position command");
        if (r.isFailure()) return r;

        r = subscribeForPorts<exlcm::scalar_i8_t>(
            MAX_SERVO_PORTS, Channels::servoMode,
            [this](PortId p, const exlcm::scalar_i8_t& cmd) { onServoModeCommand(p, cmd); },
            "servo mode command");
        if (r.isFailure()) return r;

        // System commands (single channels)
        logger_->info("Subscribing to data dump request channel: " + std::string(Channels::DATA_DUMP_REQUEST));
        auto dumpResult = broker_->subscribe<exlcm::scalar_i32_t>(
            Channels::DATA_DUMP_REQUEST,
            [this](const exlcm::scalar_i32_t& cmd) { onDataDumpRequest(cmd); }
        );
        if (dumpResult.isFailure())
        {
            return Result<void>::failure("Failed to subscribe to data dump request: " + dumpResult.error());
        }

        logger_->info("Subscribing to shutdown command channel: " + std::string(Channels::SHUTDOWN_CMD));
        auto shutdownResult = broker_->subscribe<exlcm::scalar_i32_t>(
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

        logger_->info("[TIMING] power_cmd port=" + std::to_string(port)
            + " value=" + std::to_string(command.value)
            + " msg_ts_us=" + std::to_string(command.timestamp)
            + " recv_epoch_us=" + std::to_string(nowUs)
            + " lcm_latency_us=" + std::to_string(lcmLatencyUs));

        const int32_t powerValue = command.value;

        const auto preSpiUs = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();

        Result<void> result = Result<void>::success();
        if (powerValue == 0)
        {
            result = deviceController_->setMotorVelocity(port, 0);
        }
        else
        {
            const MotorDirection direction = (powerValue > 0)
                                                 ? MotorDirection::Clockwise
                                                 : MotorDirection::CounterClockwise;
            const auto speed = static_cast<MotorSpeed>(std::abs(powerValue));
            result = deviceController_->setMotorCommand(port, direction, speed);
        }
        const auto postSpiUs = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();

        if (result.isFailure())
        {
            logger_->error("Failed to set motor command: " + result.error());
            return;
        }

        logger_->info("[TIMING] power_cmd port=" + std::to_string(port)
            + " spi_done_epoch_us=" + std::to_string(postSpiUs)
            + " spi_round_trip_us=" + std::to_string(postSpiUs - preSpiUs)
            + " total_from_send_us=" + std::to_string(postSpiUs - command.timestamp));
    }

    void CommandSubscriber::onMotorVelocityCommand(const PortId port, const exlcm::scalar_i32_t& command)
    {
        const auto nowUs = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        const auto lcmLatencyUs = nowUs - command.timestamp;

        if (!isInitialized_)
        {
            logger_->warn("Received motor velocity command while not initialized");
            return;
        }

        if (!isTimestampNewer(Channels::motorVelocityCommand(port), command.timestamp))
            return;

        logger_->info("[TIMING] velocity_cmd port=" + std::to_string(port)
            + " value=" + std::to_string(command.value)
            + " msg_ts_us=" + std::to_string(command.timestamp)
            + " recv_epoch_us=" + std::to_string(nowUs)
            + " lcm_latency_us=" + std::to_string(lcmLatencyUs));

        const auto preSpiUs = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        const auto result = deviceController_->setMotorVelocity(port, command.value);
        const auto postSpiUs = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();

        if (result.isFailure())
        {
            logger_->error("Failed to set motor velocity: " + result.error());
            return;
        }

        logger_->info("[TIMING] velocity_cmd port=" + std::to_string(port)
            + " spi_done_epoch_us=" + std::to_string(postSpiUs)
            + " spi_round_trip_us=" + std::to_string(postSpiUs - preSpiUs)
            + " total_from_send_us=" + std::to_string(postSpiUs - command.timestamp));
    }

    void CommandSubscriber::onMotorPositionCommand(const PortId port, const exlcm::vector3f_t& command)
    {
        const auto nowUs = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        const auto lcmLatencyUs = nowUs - command.timestamp;

        if (!isInitialized_)
        {
            logger_->warn("Received motor position command while not initialized");
            return;
        }

        if (!isTimestampNewer(Channels::motorPositionCommand(port), command.timestamp))
            return;

        const int32_t velocity = static_cast<int32_t>(command.x);
        const int32_t goalPosition = static_cast<int32_t>(command.y);

        logger_->info("[TIMING] position_cmd port=" + std::to_string(port)
            + " velocity=" + std::to_string(velocity) + " goal=" + std::to_string(goalPosition)
            + " msg_ts_us=" + std::to_string(command.timestamp)
            + " recv_epoch_us=" + std::to_string(nowUs)
            + " lcm_latency_us=" + std::to_string(lcmLatencyUs));

        const auto preSpiUs = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        const auto result = deviceController_->setMotorPosition(port, velocity, goalPosition);
        const auto postSpiUs = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();

        if (result.isFailure())
        {
            logger_->error("Failed to set motor position: " + result.error());
            return;
        }

        logger_->info("[TIMING] position_cmd port=" + std::to_string(port)
            + " spi_done_epoch_us=" + std::to_string(postSpiUs)
            + " spi_round_trip_us=" + std::to_string(postSpiUs - preSpiUs)
            + " total_from_send_us=" + std::to_string(postSpiUs - command.timestamp));
    }

    void CommandSubscriber::onMotorRelativeCommand(const PortId port, const exlcm::vector3f_t& command)
    {
        const auto nowUs = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        const auto lcmLatencyUs = nowUs - command.timestamp;

        if (!isInitialized_)
        {
            logger_->warn("Received motor relative command while not initialized");
            return;
        }

        if (!isTimestampNewer(Channels::motorRelativeCommand(port), command.timestamp))
            return;

        const int32_t velocity = static_cast<int32_t>(command.x);
        const int32_t deltaPosition = static_cast<int32_t>(command.y);

        logger_->info("[TIMING] relative_cmd port=" + std::to_string(port)
            + " velocity=" + std::to_string(velocity) + " delta=" + std::to_string(deltaPosition)
            + " msg_ts_us=" + std::to_string(command.timestamp)
            + " recv_epoch_us=" + std::to_string(nowUs)
            + " lcm_latency_us=" + std::to_string(lcmLatencyUs));

        const auto preSpiUs = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        const auto result = deviceController_->setMotorRelative(port, velocity, deltaPosition);
        const auto postSpiUs = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();

        if (result.isFailure())
        {
            logger_->error("Failed to set motor relative: " + result.error());
            return;
        }

        logger_->info("[TIMING] relative_cmd port=" + std::to_string(port)
            + " spi_done_epoch_us=" + std::to_string(postSpiUs)
            + " spi_round_trip_us=" + std::to_string(postSpiUs - preSpiUs)
            + " total_from_send_us=" + std::to_string(postSpiUs - command.timestamp));
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

        logger_->info(
            "Received servo position_cmd on port " + std::to_string(port) + ": " + std::to_string(command.value));
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
        const auto nowUs = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        const auto lcmLatencyUs = nowUs - command.timestamp;

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

        logger_->info("[TIMING] pid_cmd port=" + std::to_string(port)
            + " kp=" + std::to_string(kp) + " ki=" + std::to_string(ki) + " kd=" + std::to_string(kd)
            + " msg_ts_us=" + std::to_string(command.timestamp)
            + " recv_epoch_us=" + std::to_string(nowUs)
            + " lcm_latency_us=" + std::to_string(lcmLatencyUs));

        const auto preSpiUs = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        const auto result = deviceController_->setMotorPid(port, kp, ki, kd);
        const auto postSpiUs = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();

        if (result.isFailure())
        {
            logger_->error("Failed to set motor PID: " + result.error());
            return;
        }

        logger_->info("[TIMING] pid_cmd port=" + std::to_string(port)
            + " spi_done_epoch_us=" + std::to_string(postSpiUs)
            + " spi_round_trip_us=" + std::to_string(postSpiUs - preSpiUs)
            + " total_from_send_us=" + std::to_string(postSpiUs - command.timestamp));
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
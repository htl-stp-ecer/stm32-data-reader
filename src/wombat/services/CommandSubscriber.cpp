#include "wombat/services/CommandSubscriber.h"
#include <raccoon/orientation_matrix_t.hpp>
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
        const std::string& description,
        const raccoon::SubscribeOptions& options)
    {
        for (PortId i = 0; i < maxPorts; ++i)
        {
            const auto channel = channelFn(i);
            logger_->info("Subscribing to " + description + " channel: " + channel);
            auto result = broker_->subscribe<MsgT>(
                channel,
                [handler, i](const MsgT& cmd) { handler(i, cmd); },
                options
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

        static const raccoon::SubscribeOptions reliableOpts{.reliable = true};

        // Motor commands (per-port) — power and velocity are continuous control loops (plain)
        auto r = subscribeForPorts<raccoon::scalar_i32_t>(
            MAX_MOTOR_PORTS, Channels::motorPowerCommand,
            [this](PortId p, const raccoon::scalar_i32_t& cmd) { onMotorPowerCommand(p, cmd); },
            "motor power command");
        if (r.isFailure()) return r;

        r = subscribeForPorts<raccoon::scalar_i32_t>(
            MAX_MOTOR_PORTS, Channels::motorModeCommand,
            [this](PortId p, const raccoon::scalar_i32_t& cmd) { onMotorModeCommand(p, cmd); },
            "motor mode command", reliableOpts);
        if (r.isFailure()) return r;

        r = subscribeForPorts<raccoon::scalar_i32_t>(
            MAX_MOTOR_PORTS, Channels::motorStopCommand,
            [this](PortId p, const raccoon::scalar_i32_t& cmd) { onMotorStopCommand(p, cmd); },
            "motor stop command", reliableOpts);
        if (r.isFailure()) return r;

        r = subscribeForPorts<raccoon::scalar_i32_t>(
            MAX_MOTOR_PORTS, Channels::motorVelocityCommand,
            [this](PortId p, const raccoon::scalar_i32_t& cmd) { onMotorVelocityCommand(p, cmd); },
            "motor velocity command");
        if (r.isFailure()) return r;

        r = subscribeForPorts<raccoon::vector3f_t>(
            MAX_MOTOR_PORTS, Channels::motorPositionCommand,
            [this](PortId p, const raccoon::vector3f_t& cmd) { onMotorPositionCommand(p, cmd); },
            "motor position command", reliableOpts);
        if (r.isFailure()) return r;

        r = subscribeForPorts<raccoon::vector3f_t>(
            MAX_MOTOR_PORTS, Channels::motorRelativeCommand,
            [this](PortId p, const raccoon::vector3f_t& cmd) { onMotorRelativeCommand(p, cmd); },
            "motor relative command", reliableOpts);
        if (r.isFailure()) return r;

        r = subscribeForPorts<raccoon::vector3f_t>(
            MAX_MOTOR_PORTS, Channels::motorPidCommand,
            [this](PortId p, const raccoon::vector3f_t& cmd) { onMotorPidCommand(p, cmd); },
            "motor PID command", reliableOpts);
        if (r.isFailure()) return r;

        // Motor position reset (per-port) — one-shot, reliable
        r = subscribeForPorts<raccoon::scalar_i32_t>(
            MAX_MOTOR_PORTS, Channels::motorPositionResetCommand,
            [this](PortId p, const raccoon::scalar_i32_t& cmd) { onMotorPositionResetCommand(p, cmd); },
            "motor position reset command", reliableOpts);
        if (r.isFailure()) return r;

        // Servo commands (per-port) — set-and-forget, reliable
        r = subscribeForPorts<raccoon::scalar_f_t>(
            MAX_SERVO_PORTS, Channels::servoPositionCommand,
            [this](PortId p, const raccoon::scalar_f_t& cmd) { onServoPositionCommand(p, cmd); },
            "servo position command", reliableOpts);
        if (r.isFailure()) return r;

        r = subscribeForPorts<raccoon::scalar_i8_t>(
            MAX_SERVO_PORTS, Channels::servoMode,
            [this](PortId p, const raccoon::scalar_i8_t& cmd) { onServoModeCommand(p, cmd); },
            "servo mode command", reliableOpts);
        if (r.isFailure()) return r;

        // System commands (single channels)
        logger_->info("Subscribing to shutdown command channel: " + std::string(Channels::SHUTDOWN_CMD));
        auto shutdownResult = broker_->subscribe<raccoon::scalar_i32_t>(
            Channels::SHUTDOWN_CMD,
            [this](const raccoon::scalar_i32_t& cmd) { onShutdownCommand(cmd); },
            reliableOpts
        );
        if (shutdownResult.isFailure())
        {
            return Result<void>::failure("Failed to subscribe to shutdown command: " + shutdownResult.error());
        }

        // Kinematics config command (one-shot, reliable)
        logger_->info("Subscribing to kinematics config channel: " + std::string(Channels::KINEMATICS_CONFIG_CMD));
        auto kinResult = broker_->subscribe<raccoon::kinematics_config_t>(
            Channels::KINEMATICS_CONFIG_CMD,
            [this](const raccoon::kinematics_config_t& cmd) { onKinematicsConfigCommand(cmd); },
            reliableOpts
        );
        if (kinResult.isFailure())
        {
            return Result<void>::failure("Failed to subscribe to kinematics config: " + kinResult.error());
        }

        // Odometry reset command (one-shot, reliable)
        logger_->info("Subscribing to odometry reset channel: " + std::string(Channels::ODOM_RESET_CMD));
        auto odomResetResult = broker_->subscribe<raccoon::scalar_i32_t>(
            Channels::ODOM_RESET_CMD,
            [this](const raccoon::scalar_i32_t& cmd) { onOdometryResetCommand(cmd); },
            reliableOpts
        );
        if (odomResetResult.isFailure())
        {
            return Result<void>::failure("Failed to subscribe to odometry reset: " + odomResetResult.error());
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

    void CommandSubscriber::onMotorPowerCommand(const PortId port, const raccoon::scalar_i32_t& command)
    {
        if (!isInitialized_)
        {
            logger_->warn("Received motor command while not initialized");
            return;
        }

        if (!isTimestampNewer(Channels::motorPowerCommand(port), command.timestamp))
            return;

        const int32_t powerValue = command.value;

        // Map percentage (-100..100) to duty (-400..400), preserving sign for direction.
        // power=0 means zero duty (open-loop off), NOT velocity PID hold.
        const int32_t duty = (powerValue > 0)
                                 ? static_cast<int32_t>(std::min(powerValue, 100) * 4)
                                 : static_cast<int32_t>(std::max(powerValue, -100) * 4);
        const auto result = deviceController_->setMotorPwm(port, duty);

        if (result.isFailure())
        {
            logger_->error("Failed to set motor command: " + result.error());
            return;
        }
    }

    void CommandSubscriber::onMotorModeCommand(const PortId port, const raccoon::scalar_i32_t& command)
    {
        if (!isInitialized_)
        {
            logger_->warn("Received motor mode command while not initialized");
            return;
        }

        if (!isTimestampNewer(Channels::motorModeCommand(port), command.timestamp))
            return;

        // 0 = OFF, 1 = PASSIVE_BRAKE (matches MOTOR_CMD_MODE enum in pi_buffer.h)
        Result<void> result = Result<void>::success();
        if (command.value == 0)
        {
            result = deviceController_->setMotorOff(port);
        }
        else if (command.value == 1)
        {
            result = deviceController_->setMotorBrake(port);
        }
        else
        {
            logger_->warn("Unknown motor mode: " + std::to_string(command.value));
            return;
        }

        if (result.isFailure())
        {
            logger_->error("Failed to set motor mode: " + result.error());
            return;
        }

        logger_->info("Motor " + std::to_string(port) + " mode set to " +
            (command.value == 0 ? "OFF" : "PASSIVE_BRAKE"));
    }

    void CommandSubscriber::onMotorStopCommand(const PortId port, const raccoon::scalar_i32_t& command)
    {
        if (!isInitialized_)
        {
            logger_->warn("Received motor stop command while not initialized");
            return;
        }

        if (!isTimestampNewer(Channels::motorStopCommand(port), command.timestamp))
            return;

        // value == 0: coast (OFF), value != 0: passive brake
        Result<void> result = Result<void>::success();
        if (command.value == 0)
        {
            result = deviceController_->setMotorOff(port);
        }
        else
        {
            result = deviceController_->setMotorBrake(port);
        }

        if (result.isFailure())
        {
            logger_->error("Failed to stop motor: " + result.error());
            return;
        }

        logger_->info("Motor " + std::to_string(port) + " stopped (mode=" +
            (command.value == 0 ? "off" : "brake") + ")");
    }

    void CommandSubscriber::onMotorVelocityCommand(const PortId port, const raccoon::scalar_i32_t& command)
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
    }

    void CommandSubscriber::onMotorPositionCommand(const PortId port, const raccoon::vector3f_t& command)
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
    }

    void CommandSubscriber::onMotorRelativeCommand(const PortId port, const raccoon::vector3f_t& command)
    {
        if (!isInitialized_)
        {
            logger_->warn("Received motor relative command while not initialized");
            return;
        }

        if (!isTimestampNewer(Channels::motorRelativeCommand(port), command.timestamp))
            return;

        const int32_t velocity = static_cast<int32_t>(command.x);
        const int32_t delta = static_cast<int32_t>(command.y);

        auto posResult = deviceController_->getMotorPosition(port);
        if (posResult.isFailure())
        {
            logger_->error("Failed to read motor " + std::to_string(port) + " position for relative move: " + posResult.error());
            return;
        }

        const int32_t goalPosition = posResult.value() + delta;
        const auto result = deviceController_->setMotorPosition(port, velocity, goalPosition);

        if (result.isFailure())
        {
            logger_->error("Failed to set motor relative position: " + result.error());
            return;
        }

        logger_->debug("Motor " + std::to_string(port) + " relative move: delta=" +
            std::to_string(delta) + ", goal=" + std::to_string(goalPosition));
    }

    void CommandSubscriber::onServoPositionCommand(const PortId port, const raccoon::scalar_f_t& command)
    {
        if (!isInitialized_)
        {
            logger_->warn("Received servo command while not initialized");
            return;
        }

        if (!isTimestampNewer(Channels::servoPositionCommand(port), command.timestamp))
            return;

        const ServoPosition degrees = command.value;

        auto result = deviceController_->setServoCommand(port, degrees);
        if (result.isFailure())
        {
            logger_->error("Failed to set servo command: " + result.error());
            return;
        }

        logger_->info(
            "Received servo position_cmd on port " + std::to_string(port) + ": " + std::to_string(degrees) + " deg");
    }

    void CommandSubscriber::onServoModeCommand(const PortId port, const raccoon::scalar_i8_t& command)
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

    void CommandSubscriber::onMotorPositionResetCommand(const PortId port, const raccoon::scalar_i32_t& command)
    {
        if (!isInitialized_)
        {
            logger_->warn("Received motor position reset command while not initialized");
            return;
        }

        if (!isTimestampNewer(Channels::motorPositionResetCommand(port), command.timestamp))
            return;

        if (command.value == 0)
        {
            logger_->debug("Ignoring position reset command with zero value for motor " + std::to_string(port));
            return;
        }

        const auto result = deviceController_->resetMotorPosition(port);
        if (result.isFailure())
        {
            logger_->error("Failed to reset position for motor " + std::to_string(port) + ": " + result.error());
            return;
        }

        logger_->info("Reset position for motor " + std::to_string(port));
    }

    void CommandSubscriber::onMotorPidCommand(const PortId port, const raccoon::vector3f_t& command)
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
    }

    void CommandSubscriber::onShutdownCommand(const raccoon::scalar_i32_t& command)
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

        // Publish shutdown status so subscribers (like the UI) can react
        // Bitmask: bit 0 = servo shutdown, bit 1 = motor shutdown (both enabled/disabled together)
        const uint8_t shutdownFlags = enabled ? 0x03 : 0x00;
        dataPublisher_->publishShutdownStatus(shutdownFlags);

        logger_->info("Shutdown " + std::string(enabled ? "enabled" : "disabled"));
    }

    void CommandSubscriber::onKinematicsConfigCommand(const raccoon::kinematics_config_t& command)
    {
        if (!isInitialized_)
        {
            logger_->warn("Received kinematics config while not initialized");
            return;
        }

        if (!isTimestampNewer(Channels::KINEMATICS_CONFIG_CMD, command.timestamp))
            return;

        // Unpack flat array into 3x4 matrix
        float inv_matrix[3][4];
        for (int r = 0; r < 3; r++)
            for (int c = 0; c < 4; c++)
                inv_matrix[r][c] = command.inv_matrix[r * 4 + c];

        float ticks_to_rad[4];
        for (int i = 0; i < 4; i++)
            ticks_to_rad[i] = command.ticks_to_rad[i];

        // Unpack flat array into 4x3 forward kinematics matrix
        float fwd_matrix[4][3];
        for (int r = 0; r < 4; r++)
            for (int c = 0; c < 3; c++)
                fwd_matrix[r][c] = command.fwd_matrix[r * 3 + c];

        auto result = deviceController_->sendKinematicsConfig(inv_matrix, ticks_to_rad, fwd_matrix);
        if (result.isFailure())
        {
            logger_->error("Failed to send kinematics config: " + result.error());
            return;
        }

        logger_->info("Kinematics config forwarded to STM32");
    }

    void CommandSubscriber::onOdometryResetCommand(const raccoon::scalar_i32_t& command)
    {
        if (!isInitialized_)
        {
            logger_->warn("Received odometry reset while not initialized");
            return;
        }

        if (!isTimestampNewer(Channels::ODOM_RESET_CMD, command.timestamp))
            return;

        auto result = deviceController_->resetOdometry();
        if (result.isFailure())
        {
            logger_->error("Failed to reset odometry: " + result.error());
            return;
        }

        logger_->info("STM32 odometry reset");
    }
} // namespace wombat
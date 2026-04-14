#include "wombat/services/DeviceController.h"

namespace wombat
{
    DeviceController::DeviceController(std::unique_ptr<ISpi> spi, std::shared_ptr<Logger> logger)
        : spi_{std::move(spi)}, logger_{std::move(logger)}
    {
    }

    DeviceController::~DeviceController()
    {
        shutdown();
    }

    Result<void> DeviceController::initialize()
    {
        if (isInitialized_)
        {
            return Result<void>::success();
        }

        auto result = spi_->initialize();
        if (result.isFailure())
        {
            logger_->error("Failed to initialize SPI: " + result.error());
            return result;
        }

        // Initialize all motors to off state
        for (PortId port = 0; port < MAX_MOTOR_PORTS; ++port)
        {
            motorCommands_[port] = 0;
            motorModes_[port] = MotorMode::Off;
            auto setResult = spi_->setMotorOff(port);
            if (setResult.isFailure())
            {
                logger_->warn("Failed to initialize motor " + std::to_string(port) + ": " + setResult.error());
            }
        }

        // Initialize all servos to disabled state
        for (PortId port = 0; port < MAX_SERVO_PORTS; ++port)
        {
            servoCommands_[port] = 0;
            ServoState servoState{ServoMode::Disabled, 0.0f};
            auto setResult = spi_->setServoState(port, servoState);
            if (setResult.isFailure())
            {
                logger_->warn("Failed to initialize servo " + std::to_string(port) + ": " + setResult.error());
            }
        }

        isInitialized_ = true;
        logger_->info("Device controller initialized successfully");
        return Result<void>::success();
    }

    Result<void> DeviceController::shutdown()
    {
        if (!isInitialized_)
        {
            return Result<void>::success();
        }

        // Enable STM32 shutdown flag to disable motors and servos at firmware level
        spi_->setShutdown(true);

        auto result = spi_->shutdown();
        if (result.isFailure())
        {
            logger_->error("Failed to shutdown SPI: " + result.error());
            return result;
        }

        isInitialized_ = false;
        logger_->info("Device controller shut down successfully");
        return Result<void>::success();
    }

    Result<void> DeviceController::processUpdate()
    {
        if (!isInitialized_)
        {
            return Result<void>::failure("Device controller not initialized");
        }

        auto sensorResult = spi_->readSensorData();
        if (sensorResult.isFailure())
        {
            logger_->error("Failed to read sensor data: " + sensorResult.error());
            return Result<void>::failure(sensorResult.error());
        }

        lastSensorData_ = sensorResult.value();
        return Result<void>::success();
    }

    Result<void> DeviceController::setMotorOff(PortId port)
    {
        auto validationResult = validatePortId(port, MAX_MOTOR_PORTS);
        if (validationResult.isFailure())
        {
            return validationResult;
        }

        if (motorModes_[port] == MotorMode::Off)
        {
            logger_->debug("Motor " + std::to_string(port) + " already OFF, skipping SPI");
            return Result<void>::success();
        }

        motorCommands_[port] = 0;
        motorModes_[port] = MotorMode::Off;

        auto result = spi_->setMotorOff(port);
        if (result.isFailure())
        {
            logger_->error("Failed to set motor " + std::to_string(port) + " off: " + result.error());
            motorModes_[port] = MotorMode::Unknown;
            return result;
        }

        logger_->debug("Motor " + std::to_string(port) + " set to OFF");
        return Result<void>::success();
    }

    Result<void> DeviceController::setMotorBrake(PortId port)
    {
        auto validationResult = validatePortId(port, MAX_MOTOR_PORTS);
        if (validationResult.isFailure())
        {
            return validationResult;
        }

        if (motorModes_[port] == MotorMode::Brake)
        {
            logger_->debug("Motor " + std::to_string(port) + " already BRAKE, skipping SPI");
            return Result<void>::success();
        }

        motorCommands_[port] = 0;
        motorModes_[port] = MotorMode::Brake;

        auto result = spi_->setMotorBrake(port);
        if (result.isFailure())
        {
            logger_->error("Failed to set motor " + std::to_string(port) + " brake: " + result.error());
            motorModes_[port] = MotorMode::Unknown;
            return result;
        }

        logger_->debug("Motor " + std::to_string(port) + " set to PASSIVE BRAKE");
        return Result<void>::success();
    }

    Result<void> DeviceController::setMotorPwm(PortId port, int32_t duty)
    {
        auto validationResult = validatePortId(port, MAX_MOTOR_PORTS);
        if (validationResult.isFailure())
        {
            return validationResult;
        }

        motorCommands_[port] = duty;
        motorModes_[port] = MotorMode::Active;

        auto result = spi_->setMotorPwm(port, duty);
        if (result.isFailure())
        {
            logger_->error("Failed to set motor " + std::to_string(port) + " PWM: " + result.error());
            return result;
        }

        logger_->debug("Motor " + std::to_string(port) + " PWM set: duty=" + std::to_string(duty));
        return Result<void>::success();
    }

    Result<void> DeviceController::setMotorVelocity(PortId port, int32_t velocity)
    {
        auto validationResult = validatePortId(port, MAX_MOTOR_PORTS);
        if (validationResult.isFailure())
        {
            return validationResult;
        }

        motorModes_[port] = MotorMode::Active;

        auto result = spi_->setMotorVelocity(port, velocity);
        if (result.isFailure())
        {
            logger_->error("Failed to set motor " + std::to_string(port) + " velocity: " + result.error());
            return result;
        }

        logger_->debug("Motor " + std::to_string(port) + " velocity set: " + std::to_string(velocity));
        return Result<void>::success();
    }

    Result<void> DeviceController::setMotorPosition(PortId port, int32_t velocity, int32_t goalPosition)
    {
        auto validationResult = validatePortId(port, MAX_MOTOR_PORTS);
        if (validationResult.isFailure())
        {
            return validationResult;
        }

        motorModes_[port] = MotorMode::Active;

        auto result = spi_->setMotorPosition(port, velocity, goalPosition);
        if (result.isFailure())
        {
            logger_->error("Failed to set motor " + std::to_string(port) + " position: " + result.error());
            return result;
        }

        logger_->debug("Motor " + std::to_string(port) + " position set: velocity=" +
            std::to_string(velocity) + ", goal=" + std::to_string(goalPosition));
        return Result<void>::success();
    }

    Result<int32_t> DeviceController::getMotorPosition(PortId port) const
    {
        if (!isInitialized_)
        {
            return Result<int32_t>::failure("Device controller not initialized");
        }
        return spi_->getMotorPosition(port);
    }

    Result<uint8_t> DeviceController::getMotorDone() const
    {
        if (!isInitialized_)
        {
            return Result<uint8_t>::failure("Device controller not initialized");
        }
        return spi_->getMotorDone();
    }

    Result<void> DeviceController::setServoCommand(PortId port, ServoPosition position)
    {
        auto validationResult = validatePortId(port, MAX_SERVO_PORTS);
        if (validationResult.isFailure())
        {
            return validationResult;
        }

        servoCommands_[port] = position;

        ServoState state{ServoMode::Enabled, position};
        auto result = spi_->setServoState(port, state);
        if (result.isFailure())
        {
            logger_->error("Failed to set servo " + std::to_string(port) + " command: " + result.error());
            return result;
        }

        logger_->debug("Servo " + std::to_string(port) + " command set: position=" + std::to_string(position));
        return Result<void>::success();
    }

    Result<void> DeviceController::setServoMode(PortId port, ServoMode mode)
    {
        auto validationResult = validatePortId(port, MAX_SERVO_PORTS);
        if (validationResult.isFailure())
        {
            return validationResult;
        }

        // Get current position to preserve it when changing mode
        auto currentStateResult = spi_->getServoState(port);
        ServoPosition currentPosition = 0;
        if (currentStateResult.isSuccess())
        {
            currentPosition = currentStateResult.value().position;
        }

        ServoState state{mode, currentPosition};
        auto result = spi_->setServoState(port, state);
        if (result.isFailure())
        {
            logger_->error("Failed to set servo " + std::to_string(port) + " mode: " + result.error());
            return result;
        }

        logger_->debug("Servo " + std::to_string(port) + " mode set: " + std::to_string(static_cast<int>(mode)));
        return Result<void>::success();
    }

    Result<void> DeviceController::resetMotorPosition(PortId port)
    {
        if (!isInitialized_)
        {
            return Result<void>::failure("Device controller not initialized");
        }

        auto validationResult = validatePortId(port, MAX_MOTOR_PORTS);
        if (validationResult.isFailure())
        {
            return validationResult;
        }

        auto result = spi_->resetMotorPosition(port);
        if (result.isFailure())
        {
            logger_->error("Failed to reset position for motor " + std::to_string(port) + ": " + result.error());
            return result;
        }

        logger_->info("Position reset for motor " + std::to_string(port));
        return Result<void>::success();
    }

    Result<void> DeviceController::setMotorPid(PortId port, float kp, float ki, float kd)
    {
        if (!isInitialized_)
        {
            return Result<void>::failure("Device controller not initialized");
        }

        auto validationResult = validatePortId(port, MAX_MOTOR_PORTS);
        if (validationResult.isFailure())
        {
            return validationResult;
        }

        auto result = spi_->setMotorPid(port, kp, ki, kd);
        if (result.isFailure())
        {
            logger_->error("Failed to set motor PID for port " + std::to_string(port) + ": " + result.error());
            return result;
        }

        logger_->info("Motor PID set for port " + std::to_string(port) +
            ": kp=" + std::to_string(kp) + ", ki=" + std::to_string(ki) + ", kd=" + std::to_string(kd));
        return Result<void>::success();
    }

    Result<SensorData> DeviceController::getCurrentSensorData() const
    {
        if (!isInitialized_)
        {
            return Result<SensorData>::failure("Device controller not initialized");
        }

        return Result<SensorData>::success(lastSensorData_);
    }

    Result<MotorState> DeviceController::getMotorState(PortId port) const
    {
        if (!isInitialized_)
        {
            return Result<MotorState>::failure("Device controller not initialized");
        }
        return spi_->getMotorState(port);
    }

    Result<ServoState> DeviceController::getServoState(PortId port) const
    {
        if (!isInitialized_)
        {
            return Result<ServoState>::failure("Device controller not initialized");
        }
        return spi_->getServoState(port);
    }

    Result<void> DeviceController::setShutdown(bool enabled)
    {
        if (!isInitialized_)
        {
            return Result<void>::failure("Device controller not initialized");
        }

        if (enabled)
        {
            // Clear all motor and servo commands so stale state doesn't
            // re-activate actuators when shutdown is later disabled
            for (PortId port = 0; port < MAX_MOTOR_PORTS; ++port)
            {
                motorCommands_[port] = 0;
                motorModes_[port] = MotorMode::Off;
                spi_->setMotorOff(port);
            }
            for (PortId port = 0; port < MAX_SERVO_PORTS; ++port)
            {
                servoCommands_[port] = 0;
                spi_->setServoState(port, {ServoMode::Disabled, 0.0f});
            }
        }

        auto result = spi_->setShutdown(enabled);
        if (result.isFailure())
        {
            logger_->error("Failed to set shutdown: " + result.error());
            return result;
        }

        logger_->info("Shutdown " + std::string(enabled ? "enabled" : "disabled"));
        return Result<void>::success();
    }

    Result<void> DeviceController::sendKinematicsConfig(const float inv_matrix[3][4], const float ticks_to_rad[4],
                                                        const float fwd_matrix[4][3])
    {
        if (!isInitialized_)
        {
            return Result<void>::failure("Device controller not initialized");
        }
        return spi_->sendKinematicsConfig(inv_matrix, ticks_to_rad, fwd_matrix);
    }

    Result<void> DeviceController::resetOdometry()
    {
        if (!isInitialized_)
        {
            return Result<void>::failure("Device controller not initialized");
        }
        return spi_->resetOdometry();
    }

    Result<void> DeviceController::validatePortId(PortId port, PortId maxPort) const
    {
        if (port >= maxPort)
        {
            return Result<void>::failure("Invalid port ID: " + std::to_string(port) +
                " (max: " + std::to_string(maxPort - 1) + ")");
        }
        return Result<void>::success();
    }
} // namespace wombat
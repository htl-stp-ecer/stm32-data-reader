#include "wombat/services/DeviceController.h"
#include <cmath>

extern "C" {
#include "spi/pi_buffer.h"
}

namespace wombat
{
    namespace
    {
        float applyEasing(const float t, const int type)
        {
            switch (type)
            {
            case 1: return t * t;
            case 2: return t * (2.0f - t);
            case 3: { const float tt = t * t; return tt * (3.0f - 2.0f * t); }
            case 4: return (1.0f - std::cos(t * 3.14159265f)) * 0.5f;
            default: return t;
            }
        }
    }
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
            motorStates_[port] = MotorState{};
            auto setResult = spi_->setMotorState(port, motorStates_[port]);
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

        const auto now = std::chrono::steady_clock::now();
        for (PortId port = 0; port < MAX_SERVO_PORTS; ++port)
        {
            auto& s = smoothServoStates_[port];
            if (!s.active)
                continue;

            const float elapsed = std::chrono::duration<float>(now - s.startTime).count();
            const float t = std::min(elapsed / s.durationSec, 1.0f);
            const float eased = applyEasing(t, s.easingType);
            const float pos = s.startAngle + (s.targetAngle - s.startAngle) * eased;

            spi_->setServoState(port, {ServoMode::Enabled, pos});

            if (t >= 1.0f)
                s.active = false;
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

    Result<void> DeviceController::setMotorState(PortId port, const MotorState& state)
    {
        auto validationResult = validatePortId(port, MAX_MOTOR_PORTS);
        if (validationResult.isFailure())
        {
            return validationResult;
        }

        if (motorStates_[port].hasSameCommand(state))
        {
            logger_->debug("Motor " + std::to_string(port) + " state unchanged, skipping SPI");
            return Result<void>::success();
        }

        const auto previousState = motorStates_[port];
        motorStates_[port].controlMode = state.controlMode;
        motorStates_[port].target = state.target;
        motorStates_[port].goalPosition = state.goalPosition;

        auto result = spi_->setMotorState(port, state);
        if (result.isFailure())
        {
            logger_->error("Failed to set motor " + std::to_string(port) + " state: " + result.error());
            motorStates_[port] = previousState;
            return result;
        }

        logger_->debug("Motor " + std::to_string(port) + " state set: mode=" +
            std::to_string(static_cast<int>(state.controlMode)) +
            " target=" + std::to_string(state.target) +
            " goal=" + std::to_string(state.goalPosition));
        return Result<void>::success();
    }

    Result<void> DeviceController::setMotorOff(PortId port)
    {
        return setMotorState(port, MotorState{.controlMode = MotorControlMode::Off});
    }

    Result<void> DeviceController::setMotorBrake(PortId port)
    {
        return setMotorState(port, MotorState{.controlMode = MotorControlMode::PassiveBrake});
    }

    Result<void> DeviceController::setMotorPwm(PortId port, int32_t duty)
    {
        return setMotorState(port, MotorState{
                                 .controlMode = MotorControlMode::Pwm,
                                 .target = duty
                             });
    }

    Result<void> DeviceController::setMotorVelocity(PortId port, int32_t velocity)
    {
        return setMotorState(port, MotorState{
                                 .controlMode = MotorControlMode::MoveAtVelocity,
                                 .target = velocity
                             });
    }

    Result<void> DeviceController::setMotorPosition(PortId port, int32_t velocity, int32_t goalPosition)
    {
        return setMotorState(port, MotorState{
                                 .controlMode = MotorControlMode::MoveToPosition,
                                 .target = velocity,
                                 .goalPosition = goalPosition
                             });
    }

    Result<void> DeviceController::setChassisVelocity(float vx, float vy, float wz)
    {
        if (!isInitialized_)
        {
            return Result<void>::failure("Device controller not initialized");
        }

        // A zero body velocity is a full stop: passive-brake every motor instead
        // of holding it in chassis mode at a zero setpoint (the per-wheel velocity
        // PID would otherwise creep). This makes the chassis-velocity channel the
        // single, deterministic stop path — no separate motor mode_cmd is needed,
        // so there is no cross-channel race that could re-arm CHASSIS after a stop.
        if (vx == 0.0f && vy == 0.0f && wz == 0.0f)
        {
            for (PortId port = 0; port < MAX_MOTOR_PORTS; ++port)
            {
                // Leave non-drive ports (arm/other actuators) untouched — braking
                // them here would clobber whatever they were commanded to do.
                if (!driveMotors_[port])
                    continue;
                auto brakeResult = setMotorBrake(port);
                if (brakeResult.isFailure())
                {
                    logger_->error("Failed to brake motor " + std::to_string(port) +
                        " on zero chassis velocity: " + brakeResult.error());
                    return brakeResult;
                }
            }
            logger_->debug("Chassis velocity zero -> drive motors passive-braked");
            return Result<void>::success();
        }

        // Ensure the drive motors are in chassis-velocity mode. setMotorState is
        // idempotent (skips SPI when the command is unchanged), so repeated
        // chassis commands only push the mode bits once. Non-drive ports are left
        // alone so a 2-motor base can drive its arm motors independently.
        for (PortId port = 0; port < MAX_MOTOR_PORTS; ++port)
        {
            if (!driveMotors_[port])
                continue;
            auto modeResult = setMotorState(port, MotorState{.controlMode = MotorControlMode::Chassis});
            if (modeResult.isFailure())
            {
                logger_->error("Failed to set motor " + std::to_string(port) +
                    " to chassis mode: " + modeResult.error());
                return modeResult;
            }
        }

        // Stage the body-frame setpoint; this is pushed on every call so the
        // STM32 always tracks the latest commanded velocity.
        auto result = spi_->setChassisVelocity(vx, vy, wz);
        if (result.isFailure())
        {
            logger_->error("Failed to set chassis velocity: " + result.error());
            return result;
        }

        logger_->debug("Chassis velocity set: vx=" + std::to_string(vx) +
            " vy=" + std::to_string(vy) + " wz=" + std::to_string(wz));
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

        // A direct position command takes ownership immediately and cancels any
        // in-flight smooth trajectory for this servo.
        smoothServoStates_[port].active = false;
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

    Result<void> DeviceController::startSmoothServo(PortId port, float targetAngle, float speedDegPerSec,
                                                    int easingType)
    {
        auto validationResult = validatePortId(port, MAX_SERVO_PORTS);
        if (validationResult.isFailure())
            return validationResult;

        if (speedDegPerSec <= 0.0f)
            return Result<void>::failure("Speed must be > 0");

        auto currentStateResult = spi_->getServoState(port);
        const float startAngle = currentStateResult.isSuccess() ? currentStateResult.value().position : 0.0f;

        const float delta = std::abs(targetAngle - startAngle);
        if (delta < 0.5f)
        {
            smoothServoStates_[port].active = false;
            spi_->setServoState(port, {ServoMode::Enabled, targetAngle});
            return Result<void>::success();
        }

        auto& s = smoothServoStates_[port];
        s.startAngle = startAngle;
        s.targetAngle = targetAngle;
        s.durationSec = delta / speedDegPerSec;
        s.easingType = easingType;
        s.startTime = std::chrono::steady_clock::now();
        s.active = true;

        logger_->debug("Smooth servo " + std::to_string(port) + ": " +
            std::to_string(startAngle) + " -> " + std::to_string(targetAngle) +
            " deg, " + std::to_string(s.durationSec) + "s");
        return Result<void>::success();
    }

    Result<void> DeviceController::setServoMode(PortId port, ServoMode mode)
    {
        auto validationResult = validatePortId(port, MAX_SERVO_PORTS);
        if (validationResult.isFailure())
        {
            return validationResult;
        }

        if (mode != ServoMode::Enabled)
        {
            smoothServoStates_[port].active = false;
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
            // Clear all motor commands so stale state doesn't re-activate the
            // motors when shutdown is later cleared.
            for (PortId port = 0; port < MAX_MOTOR_PORTS; ++port)
            {
                motorStates_[port] = MotorState{};
                spi_->setMotorState(port, motorStates_[port]);
            }

            // Servos must HOLD their last position on shutdown — they are not
            // disabled here (only fully_disable_servos() releases them). Any
            // in-flight smooth motion is frozen at its current interpolated
            // position so the servo stops lerping and just holds. Idle servos
            // are left untouched: the firmware already keeps their last mode
            // and position.
            const auto now = std::chrono::steady_clock::now();
            for (PortId port = 0; port < MAX_SERVO_PORTS; ++port)
            {
                auto& s = smoothServoStates_[port];
                if (!s.active)
                    continue;

                const float elapsed = std::chrono::duration<float>(now - s.startTime).count();
                const float t = std::min(elapsed / s.durationSec, 1.0f);
                const float eased = applyEasing(t, s.easingType);
                const float pos = s.startAngle + (s.targetAngle - s.startAngle) * eased;

                s.active = false;
                spi_->setServoState(port, {ServoMode::Enabled, pos});
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
                                                        const float fwd_matrix[4][3], const float bemf_offset[4])
    {
        if (!isInitialized_)
        {
            return Result<void>::failure("Device controller not initialized");
        }

        // Derive the drive-motor mask from ticks_to_rad. The firmware treats a
        // wheel with |ticks_to_rad| < 1e-9 as dead (odometry_chassis_wheel_target
        // returns 0), so those ports are not chassis wheels and setChassisVelocity
        // must not brake or mode-switch them. Matches the firmware epsilon.
        std::array<bool, MAX_MOTOR_PORTS> mask{};
        int driveCount = 0;
        for (PortId port = 0; port < MAX_MOTOR_PORTS; ++port)
        {
            mask[port] = std::fabs(ticks_to_rad[port]) >= 1e-9f;
            if (mask[port]) ++driveCount;
        }
        // Guard against an empty/invalid config silently disabling the drivetrain:
        // if no motor qualifies, keep the legacy "drive all four" behaviour.
        if (driveCount == 0)
        {
            logger_->warn("Kinematics config has no drive motors (all ticks_to_rad ~0); "
                "keeping all motors as chassis drive wheels");
            mask.fill(true);
        }
        driveMotors_ = mask;

        return spi_->sendKinematicsConfig(inv_matrix, ticks_to_rad, fwd_matrix, bemf_offset);
    }

    Result<void> DeviceController::resetOdometry()
    {
        if (!isInitialized_)
        {
            return Result<void>::failure("Device controller not initialized");
        }
        return spi_->resetOdometry();
    }

    Result<void> DeviceController::setBemfEnabled(bool enabled)
    {
        // FEATURE_BEMF_DISABLE is inverted: enabled=true clears the bit, enabled=false sets it.
        if (enabled)
            featureFlags_ &= static_cast<uint8_t>(~FEATURE_BEMF_DISABLE);
        else
            featureFlags_ |= static_cast<uint8_t>(FEATURE_BEMF_DISABLE);

        auto result = spi_->setFeatureFlags(featureFlags_);
        if (result.isFailure())
        {
            logger_->error("Failed to push feature flags to STM32: " + result.error());
            return result;
        }

        logger_->info(std::string("BEMF ") + (enabled ? "enabled" : "disabled (speed mode)") +
            " — featureFlags=0x" + std::to_string(static_cast<unsigned>(featureFlags_)));
        return Result<void>::success();
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

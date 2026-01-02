#include "wombat/services/DeviceController.h"

namespace wombat {

DeviceController::DeviceController(std::unique_ptr<Spi> spi, std::shared_ptr<Logger> logger)
    : spi_{std::move(spi)}, logger_{std::move(logger)} {}

DeviceController::~DeviceController() {
    shutdown();
}

Result<void> DeviceController::initialize() {
    if (isInitialized_) {
        return Result<void>::success();
    }

    auto result = spi_->initialize();
    if (result.isFailure()) {
        logger_->error("Failed to initialize SPI: " + result.error());
        return result;
    }

    // Initialize all motors to off state
    for (PortId port = 0; port < MAX_MOTOR_PORTS; ++port) {
        motorCommands_[port] = 0;
        motorStopActive_[port] = false;
        MotorState motorState{MotorDirection::Off, 0, 0};
        auto setResult = spi_->setMotorState(port, motorState);
        if (setResult.isFailure()) {
            logger_->warn("Failed to initialize motor " + std::to_string(port) + ": " + setResult.error());
        }
    }

    // Initialize all servos to disabled state
    for (PortId port = 0; port < MAX_SERVO_PORTS; ++port) {
        servoCommands_[port] = 0;
        ServoState servoState{ServoMode::Disabled, 0};
        auto setResult = spi_->setServoState(port, servoState);
        if (setResult.isFailure()) {
            logger_->warn("Failed to initialize servo " + std::to_string(port) + ": " + setResult.error());
        }
    }

    isInitialized_ = true;
    emergencyStopActive_ = false;
    logger_->info("Device controller initialized successfully");
    return Result<void>::success();
}

Result<void> DeviceController::shutdown() {
    if (!isInitialized_) {
        return Result<void>::success();
    }

    auto emergencyResult = emergencyStop();
    if (emergencyResult.isFailure()) {
        logger_->error("Failed to perform emergency stop during shutdown: " + emergencyResult.error());
    }

    auto result = spi_->shutdown();
    if (result.isFailure()) {
        logger_->error("Failed to shutdown SPI: " + result.error());
        return result;
    }

    isInitialized_ = false;
    logger_->info("Device controller shut down successfully");
    return Result<void>::success();
}

Result<void> DeviceController::processUpdate() {
    if (!isInitialized_) {
        return Result<void>::failure("Device controller not initialized");
    }

    if (emergencyStopActive_) {
        return Result<void>::success();
    }

    // Enforce stop latch by continually driving stopped state
    for (PortId port = 0; port < MAX_MOTOR_PORTS; ++port) {
        if (motorStopActive_[port].load()) {
            motorCommands_[port] = 0;
            MotorState motorState{MotorDirection::Off, 0, 0};
            auto stopResult = spi_->setMotorState(port, motorState);
            if (stopResult.isFailure()) {
                logger_->warn("Failed to enforce stop latch on motor " + std::to_string(port) + ": " + stopResult.error());
            }
        }
    }

    auto sensorResult = spi_->readSensorData();
    if (sensorResult.isFailure()) {
        logger_->error("Failed to read sensor data: " + sensorResult.error());
        return Result<void>::failure(sensorResult.error());
    }

    lastSensorData_ = sensorResult.value();
    return Result<void>::success();
}

Result<void> DeviceController::setMotorCommand(PortId port, MotorDirection direction, MotorSpeed speed) {
    auto validationResult = validatePortId(port, MAX_MOTOR_PORTS);
    if (validationResult.isFailure()) {
        return validationResult;
    }

    if (emergencyStopActive_) {
        return Result<void>::failure("Emergency stop is active, motor commands disabled");
    }

    if (motorStopActive_[port].load()) {
        return Result<void>::failure("Motor stop switch active for port " + std::to_string(port));
    }

    motorCommands_[port] = speed;

    const MotorState state{direction, speed, 0};
    auto result = spi_->setMotorState(port, state);
    if (result.isFailure()) {
        logger_->error("Failed to set motor " + std::to_string(port) + " command: " + result.error());
        return result;
    }

    logger_->debug("Motor " + std::to_string(port) + " command set: direction=" +
                  std::to_string(static_cast<int>(direction)) + ", speed=" + std::to_string(speed));
    return Result<void>::success();
}

Result<void> DeviceController::setServoCommand(PortId port, ServoPosition position) {
    auto validationResult = validatePortId(port, MAX_SERVO_PORTS);
    if (validationResult.isFailure()) {
        return validationResult;
    }

    if (emergencyStopActive_) {
        return Result<void>::failure("Emergency stop is active, servo commands disabled");
    }

    servoCommands_[port] = position;

    ServoState state{ServoMode::Enabled, position};
    auto result = spi_->setServoState(port, state);
    if (result.isFailure()) {
        logger_->error("Failed to set servo " + std::to_string(port) + " command: " + result.error());
        return result;
    }

    logger_->debug("Servo " + std::to_string(port) + " command set: position=" + std::to_string(position));
    return Result<void>::success();
}

Result<void> DeviceController::setMotorStop(PortId port, bool engaged) {
    auto validationResult = validatePortId(port, MAX_MOTOR_PORTS);
    if (validationResult.isFailure()) {
        return validationResult;
    }

    motorStopActive_[port] = engaged;

    if (engaged) {
        motorCommands_[port] = 0;
        MotorState motorState{MotorDirection::Off, 0, 0};
        auto result = spi_->setMotorState(port, motorState);
        if (result.isFailure()) {
            logger_->error("Failed to apply stop command to motor " + std::to_string(port) + ": " + result.error());
            return result;
        }

        logger_->warn("Motor " + std::to_string(port) + " stop engaged - ignoring commands until wake-up");
    } else {
        logger_->info("Motor " + std::to_string(port) + " stop cleared - accepting commands again");
    }

    return Result<void>::success();
}

Result<void> DeviceController::resetBemfSum(PortId port) {
    if (!isInitialized_) {
        return Result<void>::failure("Device controller not initialized");
    }

    auto validationResult = validatePortId(port, MAX_MOTOR_PORTS);
    if (validationResult.isFailure()) {
        return validationResult;
    }

    auto result = spi_->resetBemfSum(port);
    if (result.isFailure()) {
        logger_->error("Failed to reset BEMF sum for motor " + std::to_string(port) + ": " + result.error());
        return result;
    }

    logger_->info("BEMF sum reset for motor " + std::to_string(port));
    return Result<void>::success();
}

Result<SensorData> DeviceController::getCurrentSensorData() const {
    if (!isInitialized_) {
        return Result<SensorData>::failure("Device controller not initialized");
    }

    return Result<SensorData>::success(lastSensorData_);
}

Result<MotorState> DeviceController::getMotorState(PortId port) const {
    if (!isInitialized_) {
        return Result<MotorState>::failure("Device controller not initialized");
    }
    return spi_->getMotorState(port);
}

Result<ServoState> DeviceController::getServoState(PortId port) const {
    if (!isInitialized_) {
        return Result<ServoState>::failure("Device controller not initialized");
    }
    return spi_->getServoState(port);
}

Result<void> DeviceController::emergencyStop() {
    if (!isInitialized_) {
        return Result<void>::failure("Device controller not initialized");
    }

    emergencyStopActive_ = true;
    logger_->warn("Emergency stop activated - stopping all motors and disabling servos");

    // Stop all motors
    for (PortId port = 0; port < MAX_MOTOR_PORTS; ++port) {
        motorCommands_[port] = 0;
        MotorState motorState{MotorDirection::Off, 0, 0};
        auto result = spi_->setMotorState(port, motorState);
        if (result.isFailure()) {
            logger_->error("Failed to stop motor " + std::to_string(port) + " during emergency stop");
        }
    }

    // Disable all servos
    for (PortId port = 0; port < MAX_SERVO_PORTS; ++port) {
        ServoState servoState{ServoMode::Disabled, 0};
        auto result = spi_->setServoState(port, servoState);
        if (result.isFailure()) {
            logger_->error("Failed to disable servo " + std::to_string(port) + " during emergency stop");
        }
    }

    auto forceUpdateResult = spi_->forceUpdate();
    if (forceUpdateResult.isFailure()) {
        logger_->error("Failed to force hardware update during emergency stop");
        return forceUpdateResult;
    }

    return Result<void>::success();
}

Result<void> DeviceController::validatePortId(PortId port, PortId maxPort) const {
    if (port >= maxPort) {
        return Result<void>::failure("Invalid port ID: " + std::to_string(port) +
                                    " (max: " + std::to_string(maxPort - 1) + ")");
    }
    return Result<void>::success();
}

} // namespace wombat

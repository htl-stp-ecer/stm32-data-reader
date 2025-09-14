#include "wombat/services/CommandSubscriber.h"

namespace wombat {

CommandSubscriber::CommandSubscriber(std::shared_ptr<LcmBroker> broker,
                                   std::shared_ptr<DeviceController> deviceController,
                                   std::shared_ptr<Logger> logger)
    : broker_{std::move(broker)},
      deviceController_{std::move(deviceController)},
      logger_{std::move(logger)} {}

Result<void> CommandSubscriber::initialize() {
    if (isInitialized_) {
        return Result<void>::success();
    }

    // Subscribe to motor power commands
    auto motorResult = broker_->subscribeScalarI32(
        Channels::motorPowerCommand(0),
        [this](const exlcm::scalar_i32_t& cmd) { onMotorPowerCommand(cmd); }
    );
    if (motorResult.isFailure()) {
        return Result<void>::failure("Failed to subscribe to motor commands: " + motorResult.error());
    }

    // Subscribe to servo position commands
    auto servoResult = broker_->subscribeScalarI32(
        Channels::servoPositionCommand(0),
        [this](const exlcm::scalar_i32_t& cmd) { onServoPositionCommand(cmd); }
    );
    if (servoResult.isFailure()) {
        return Result<void>::failure("Failed to subscribe to servo commands: " + servoResult.error());
    }

    isInitialized_ = true;
    logger_->info("Command subscriber initialized successfully");
    return Result<void>::success();
}

Result<void> CommandSubscriber::shutdown() {
    if (!isInitialized_) {
        return Result<void>::success();
    }

    isInitialized_ = false;
    logger_->info("Command subscriber shut down");
    return Result<void>::success();
}

void CommandSubscriber::onMotorPowerCommand(const exlcm::scalar_i32_t& command) {
    if (!isInitialized_) {
        logger_->warn("Received motor command while not initialized");
        return;
    }

    const int32_t powerValue = command.value;
    const auto direction = powerValue >= 0 ? MotorDirection::Clockwise : MotorDirection::CounterClockwise;
    const auto speed = static_cast<MotorSpeed>(std::abs(powerValue));

    auto result = deviceController_->setMotorCommand(0, direction, speed);
    if (result.isFailure()) {
        logger_->error("Failed to set motor command: " + result.error());
        return;
    }

    // Force hardware update
    auto forceResult = deviceController_->processUpdate();
    if (forceResult.isFailure()) {
        logger_->error("Failed to force device update after motor command: " + forceResult.error());
    }

    logger_->info("Received motor power_cmd: " + std::to_string(powerValue));
}

void CommandSubscriber::onServoPositionCommand(const exlcm::scalar_i32_t& command) {
    if (!isInitialized_) {
        logger_->warn("Received servo command while not initialized");
        return;
    }

    const auto position = static_cast<ServoPosition>(command.value);

    auto result = deviceController_->setServoCommand(0, position);
    if (result.isFailure()) {
        logger_->error("Failed to set servo command: " + result.error());
        return;
    }

    // Force hardware update
    auto forceResult = deviceController_->processUpdate();
    if (forceResult.isFailure()) {
        logger_->error("Failed to force device update after servo command: " + forceResult.error());
    }

    logger_->info("Received servo position_cmd: " + std::to_string(command.value));
}

} // namespace wombat
//
// Created by tobias on 9/14/25.
//
#pragma once

#include "wombat/core/Types.h"
#include "wombat/core/Result.h"
#include "wombat/core/Logger.h"
#include "wombat/hardware/Spi.hpp"
#include <memory>
#include <array>
#include <atomic>

namespace wombat {

class DeviceController {
public:
    DeviceController(std::unique_ptr<Spi> spi, std::shared_ptr<Logger> logger);
    ~DeviceController();

    Result<void> initialize();
    Result<void> shutdown();
    Result<void> processUpdate();

    Result<void> setMotorCommand(PortId port, MotorDirection direction, MotorSpeed speed);
    Result<void> setMotorStop(PortId port, bool engaged);
    Result<void> setServoCommand(PortId port, ServoPosition position);
    Result<void> setServoMode(PortId port, ServoMode mode);
    Result<void> resetBemfSum(PortId port);
    Result<void> setBemfScale(PortId port, float scale);
    Result<void> setBemfOffset(PortId port, float offset);
    Result<void> setBemfNominalVoltage(int16_t adcValue);

    Result<SensorData> getCurrentSensorData() const;
    Result<MotorState> getMotorState(PortId port) const;
    Result<ServoState> getServoState(PortId port) const;

    // STM32 shutdown flag - disables motors and servos at firmware level
    Result<void> setShutdown(bool enabled);

private:
    std::unique_ptr<Spi> spi_;
    std::shared_ptr<Logger> logger_;

    std::array<std::atomic<MotorSpeed>, MAX_MOTOR_PORTS> motorCommands_{};
    std::array<std::atomic<ServoPosition>, MAX_SERVO_PORTS> servoCommands_{};
    std::array<std::atomic_bool, MAX_MOTOR_PORTS> motorStopActive_{};

    SensorData lastSensorData_{};
    bool isInitialized_{false};

    [[nodiscard]] Result<void> validatePortId(PortId port, PortId maxPort) const;
};

} // namespace wombat

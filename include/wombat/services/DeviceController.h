//
// Created by tobias on 9/14/25.
//
#pragma once

#include "wombat/core/DeviceTypes.h"
#include "wombat/core/Result.h"
#include "wombat/core/Logger.h"
#include "wombat/hardware/ISpi.h"
#include <memory>
#include <array>
#include <atomic>

namespace wombat
{
    class DeviceController
    {
    public:
        DeviceController(std::unique_ptr<ISpi> spi, std::shared_ptr<Logger> logger);
        ~DeviceController();

        Result<void> initialize();
        Result<void> shutdown();
        Result<void> processUpdate();

        Result<void> setMotorOff(PortId port);
        Result<void> setMotorBrake(PortId port);
        Result<void> setMotorPwm(PortId port, int32_t duty);
        Result<void> setMotorVelocity(PortId port, int32_t velocity);
        Result<void> setMotorPosition(PortId port, int32_t velocity, int32_t goalPosition);
        Result<int32_t> getMotorPosition(PortId port) const;
        Result<uint8_t> getMotorDone() const;
        Result<void> setServoCommand(PortId port, ServoPosition position);
        Result<void> setServoMode(PortId port, ServoMode mode);
        Result<void> resetMotorPosition(PortId port);
        Result<void> setBemfScale(PortId port, float scale);
        Result<void> setBemfOffset(PortId port, float offset);
        Result<void> setBemfNominalVoltage(int16_t adcValue);
        Result<void> setMotorPid(PortId port, float kp, float ki, float kd);

        Result<SensorData> getCurrentSensorData() const;
        Result<MotorState> getMotorState(PortId port) const;
        Result<ServoState> getServoState(PortId port) const;

        // STM32 shutdown flag - disables motors and servos at firmware level
        Result<void> setShutdown(bool enabled);

    private:
        std::unique_ptr<ISpi> spi_;
        std::shared_ptr<Logger> logger_;

        std::array<std::atomic<int32_t>, MAX_MOTOR_PORTS> motorCommands_{};
        std::array<std::atomic<ServoPosition>, MAX_SERVO_PORTS> servoCommands_{};

        SensorData lastSensorData_{};
        bool isInitialized_{false};

        [[nodiscard]] Result<void> validatePortId(PortId port, PortId maxPort) const;
    };
} // namespace wombat
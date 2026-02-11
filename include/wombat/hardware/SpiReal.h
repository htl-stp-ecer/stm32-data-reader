#pragma once

#include "wombat/hardware/ISpi.h"
#include "wombat/core/Configuration.h"
#include "wombat/core/Logger.h"
#include <memory>
#include <array>

namespace wombat
{
    class SpiReal final : public ISpi
    {
    public:
        explicit SpiReal(const Configuration::Spi& cfg, std::shared_ptr<Logger> logger);

        Result<void> initialize() override;
        Result<void> shutdown() override;
        Result<void> forceUpdate() override;

        Result<SensorData> readSensorData() override;

        Result<void> setMotorState(PortId port, const MotorState& state) override;
        Result<void> setMotorVelocity(PortId port, int32_t velocity) override;
        Result<void> setMotorPosition(PortId port, int32_t velocity, int32_t goalPosition) override;
        Result<void> setMotorRelative(PortId port, int32_t velocity, int32_t deltaPosition) override;
        Result<int32_t> getMotorPosition(PortId port) override;
        Result<uint8_t> getMotorDone() override;
        Result<MotorState> getMotorState(PortId port) const override;

        Result<void> setServoState(PortId port, const ServoState& state) override;
        Result<ServoState> getServoState(PortId port) const override;

        Result<void> resetBemfSum(PortId port) override;
        Result<void> setMotorPid(PortId port, float kp, float ki, float kd) override;
        Result<void> setBemfScale(PortId port, float scale) override;
        Result<void> setBemfOffset(PortId port, float offset) override;
        Result<void> setBemfNominalVoltage(int16_t adcValue) override;

        Result<void> setShutdown(bool enabled) override;

    private:
        Configuration::Spi cfg_;
        std::shared_ptr<Logger> logger_;
        std::array<MotorState, MAX_MOTOR_PORTS> motors_{};
        std::array<ServoState, MAX_SERVO_PORTS> servos_{};
        std::array<int32_t, MAX_MOTOR_PORTS> bemfOffsets_{};
    };
}
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

        Result<void> setMotorOff(PortId port) override;
        Result<void> setMotorBrake(PortId port) override;
        Result<void> setMotorPwm(PortId port, int32_t duty) override;
        Result<void> setMotorVelocity(PortId port, int32_t velocity) override;
        Result<void> setMotorPosition(PortId port, int32_t velocity, int32_t goalPosition) override;
        Result<int32_t> getMotorPosition(PortId port) override;
        Result<uint8_t> getMotorDone() override;
        Result<MotorState> getMotorState(PortId port) const override;

        Result<void> setServoState(PortId port, const ServoState& state) override;
        Result<ServoState> getServoState(PortId port) const override;

        Result<void> resetMotorPosition(PortId port) override;
        Result<void> setMotorPid(PortId port, float kp, float ki, float kd) override;

        Result<void> setShutdown(bool enabled) override;

        Result<void> sendKinematicsConfig(const float inv_matrix[3][4], const float ticks_to_rad[4],
                                          const float fwd_matrix[4][3]) override;
        Result<void> resetOdometry() override;

    private:
        Configuration::Spi cfg_;
        std::shared_ptr<Logger> logger_;
        std::array<MotorState, MAX_MOTOR_PORTS> motors_{};
        std::array<ServoState, MAX_SERVO_PORTS> servos_{};
        float filteredBatteryVoltage_{0.0f};
    };
}
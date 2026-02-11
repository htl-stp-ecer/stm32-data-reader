#pragma once

#include "wombat/core/DeviceTypes.h"
#include "wombat/core/Result.h"
#include <cstdint>

namespace wombat
{
    class ISpi
    {
    public:
        virtual ~ISpi() = default;

        virtual Result<void> initialize() = 0;
        virtual Result<void> shutdown() = 0;
        virtual Result<void> forceUpdate() = 0;

        virtual Result<SensorData> readSensorData() = 0;

        virtual Result<void> setMotorState(PortId port, const MotorState& state) = 0;
        virtual Result<void> setMotorVelocity(PortId port, int32_t velocity) = 0;
        virtual Result<void> setMotorPosition(PortId port, int32_t velocity, int32_t goalPosition) = 0;
        virtual Result<void> setMotorRelative(PortId port, int32_t velocity, int32_t deltaPosition) = 0;
        virtual Result<int32_t> getMotorPosition(PortId port) = 0;
        virtual Result<uint8_t> getMotorDone() = 0;
        virtual Result<MotorState> getMotorState(PortId port) const = 0;

        virtual Result<void> setServoState(PortId port, const ServoState& state) = 0;
        virtual Result<ServoState> getServoState(PortId port) const = 0;

        virtual Result<void> resetBemfSum(PortId port) = 0;
        virtual Result<void> setMotorPid(PortId port, float kp, float ki, float kd) = 0;
        virtual Result<void> setBemfScale(PortId port, float scale) = 0;
        virtual Result<void> setBemfOffset(PortId port, float offset) = 0;
        virtual Result<void> setBemfNominalVoltage(int16_t adcValue) = 0;

        virtual Result<void> setShutdown(bool enabled) = 0;
    };
}
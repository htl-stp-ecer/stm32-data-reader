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
        virtual Result<void> setMotorOff(PortId port) = 0;
        virtual Result<void> setMotorBrake(PortId port) = 0;
        virtual Result<void> setMotorPwm(PortId port, int32_t duty) = 0;
        virtual Result<void> setMotorVelocity(PortId port, int32_t velocity) = 0;
        virtual Result<void> setMotorPosition(PortId port, int32_t velocity, int32_t goalPosition) = 0;

        // Stage the body-frame chassis velocity setpoint [vx (m/s), vy (m/s), wz (rad/s)]
        // for MOT_MODE_CHASSIS. Callers set the motors to chassis mode separately.
        virtual Result<void> setChassisVelocity(float vx, float vy, float wz) = 0;
        virtual Result<int32_t> getMotorPosition(PortId port) = 0;
        virtual Result<uint8_t> getMotorDone() = 0;
        virtual Result<MotorState> getMotorState(PortId port) const = 0;

        virtual Result<void> setServoState(PortId port, const ServoState& state) = 0;
        virtual Result<ServoState> getServoState(PortId port) const = 0;

        virtual Result<void> resetMotorPosition(PortId port) = 0;
        virtual Result<void> setMotorPid(PortId port, float kp, float ki, float kd) = 0;
        virtual Result<void> setShutdown(bool enabled) = 0;

        virtual Result<void> sendKinematicsConfig(const float inv_matrix[3][4], const float ticks_to_rad[4],
                                                  const float fwd_matrix[4][3], const float bemf_offset[4]) = 0;
        virtual Result<void> resetOdometry() = 0;

        // Feature flags: replace the full opt-in toggle byte (see FEATURE_*).
        virtual Result<void> setFeatureFlags(uint8_t flags) = 0;
    };
}

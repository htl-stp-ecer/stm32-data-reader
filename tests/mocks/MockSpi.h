#pragma once

#include "wombat/hardware/ISpi.h"
#include <gmock/gmock.h>

namespace wombat::test
{
    class MockSpi : public ISpi
    {
    public:
        MOCK_METHOD(Result<void>, initialize, (), (override)
        );
        MOCK_METHOD(Result<void>, shutdown, (), (override)
        );
        MOCK_METHOD(Result<void>, forceUpdate, (), (override)
        );

        MOCK_METHOD(Result<SensorData>, readSensorData, (), (override)
        );

        MOCK_METHOD(Result<void>, setMotorOff, (PortId port), (override)
        );
        MOCK_METHOD(Result<void>, setMotorBrake, (PortId port), (override)
        );
        MOCK_METHOD(Result<void>, setMotorPwm, (PortId port, int32_t duty), (override)
        );
        MOCK_METHOD(Result<void>, setMotorVelocity, (PortId port, int32_t velocity), (override)
        );
        MOCK_METHOD(Result<void>, setMotorPosition, (PortId port, int32_t velocity, int32_t goalPosition), (override)
        );
        MOCK_METHOD(Result<int32_t>, getMotorPosition, (PortId port), (override)
        );
        MOCK_METHOD(Result<uint8_t>, getMotorDone, (), (override)
        );
        MOCK_METHOD(Result<MotorState>, getMotorState, (PortId port), (
        
        const
        ,
        override
        )
        );

        MOCK_METHOD(Result<void>, setServoState, (PortId port, const ServoState& state), (override)
        );
        MOCK_METHOD(Result<ServoState>, getServoState, (PortId port), (
        
        const
        ,
        override
        )
        );

        MOCK_METHOD(Result<void>, resetMotorPosition, (PortId port), (override)
        );
        MOCK_METHOD(Result<void>, setMotorPid, (PortId port, float kp, float ki, float kd), (override)
        );
        MOCK_METHOD(Result<void>, setShutdown, (bool enabled), (override)
        );
    };
}
//
// Created by tobias on 9/14/25.
//
#pragma once

#include "wombat/core/Types.h"
#include "wombat/core/Result.h"
#include "wombat/core/Logger.h"
#include "wombat/messaging/LcmBroker.h"
#include <memory>

namespace wombat
{
    class DataPublisher
    {
    public:
        DataPublisher(std::shared_ptr<LcmBroker> broker, std::shared_ptr<Logger> logger);

        Result<void> publishSensorData(const SensorData& data);
        Result<void> publishMotorState(PortId port, const MotorState& state);
        Result<void> publishServoState(PortId port, const ServoState& state);

    private:
        std::shared_ptr<LcmBroker> broker_;
        std::shared_ptr<Logger> logger_;

        exlcm::vector3f_t convertVector3f(const Vector3f& vector) const;
        exlcm::scalar_f_t convertScalarF(float value) const;
        exlcm::scalar_i32_t convertScalarI32(int32_t value) const;
        exlcm::scalar_i8_t convertScalarI8(uint8_t value) const;

        Result<void> publishAnalogValues(const std::array<AnalogValue, MAX_ANALOG_PORTS>& values);
        Result<void> publishDigitalBits(DigitalValue digitalBits);
    };
} // namespace wombat

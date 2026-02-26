//
// Created by tobias on 9/14/25.
// Modified by jakob on 11/20/25
//
#pragma once

#include "wombat/core/DeviceTypes.h"
#include "wombat/core/Channels.h"
#include "wombat/core/Result.h"
#include "wombat/core/Logger.h"
#include "wombat/core/AxisRemap.h"
#include "wombat/messaging/LcmBroker.h"
#include <memory>
#include <chrono>
#include <optional>

namespace wombat
{
    class DataPublisher
    {
    public:
        DataPublisher(std::shared_ptr<LcmBroker> broker, std::shared_ptr<Logger> logger);

        void setAxisRemap(const int8_t matrix[9]);
        Result<void> publishSensorData(const SensorData& data);
        Result<void> publishMotorState(PortId port, const MotorState& state);
        Result<void> publishMotorPosition(PortId port, int32_t position);
        Result<void> publishMotorDone(PortId port, bool done);
        Result<void> publishServoState(PortId port, const ServoState& state);
        Result<void> publishShutdownStatus(uint8_t shutdownFlags);

    private:
        std::shared_ptr<LcmBroker> broker_;
        std::shared_ptr<Logger> logger_;
        AxisRemap remap_;

        // Accuracy throttling and change detection
        std::optional<ImuAccuracy> lastAccuracy_;
        std::chrono::steady_clock::time_point lastAccuracyPublishTime_{};
        static constexpr std::chrono::seconds accuracyPublishInterval_{15};

        Result<void> publishAnalogValues(const std::array<AnalogValue, MAX_ANALOG_PORTS>& values);
        Result<void> publishDigitalBits(DigitalValue digitalBits);
        Result<void> publishAccuracy(const ImuAccuracy& accuracy);
    };
} // namespace wombat
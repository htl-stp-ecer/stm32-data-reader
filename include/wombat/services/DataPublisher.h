//
// Created by tobias on 9/14/25.
// Modified by jakob on 11/20/25
//
#pragma once

#include "wombat/core/DeviceTypes.h"
#include <raccoon/Channels.h>
#include <raccoon/vector3f_t.hpp>
#include <raccoon/quaternion_t.hpp>
#include <raccoon/scalar_f_t.hpp>
#include "wombat/core/Result.h"
#include "wombat/core/Logger.h"
#include "wombat/messaging/LcmBroker.h"
#include <chrono>
#include <memory>
#include <optional>

namespace wombat
{
    namespace Channels = raccoon::Channels;

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
        // Publishes the BEMF-enabled state as retained scalar_i32_t (1 = enabled, 0 = disabled).
        Result<void> publishBemfEnabled(bool enabled);

    private:
        std::shared_ptr<LcmBroker> broker_;
        std::shared_ptr<Logger> logger_;

        // Accuracy change detection
        std::optional<ImuAccuracy> lastAccuracy_;

        // ---- Per-channel publish gates ----------------------------------
        //
        // The main loop drives publishSensorData at ~200 Hz, which used
        // to flood every channel even though every consumer in
        // raccoon-lib polls at ≤ 50 Hz and most sensors are noisy enough
        // that bit-identical samples are rare (so the broker's exact-
        // match dedup misses noise jitter). A per-channel gate cuts the
        // rate to a configurable max Hz and also drops samples whose
        // change vs the last published value is below an L-infinity
        // noise threshold — i.e. "no axis moved by more than `epsilon`".
        //
        // Each gate keeps its own last-published value + timestamp.
        // Templated on the message type so vector3f_t, quaternion_t and
        // scalar_f_t share the same machinery via the L∞ helper in
        // DataPublisher.cpp. Accelerometer intentionally has no gate
        // (full 200 Hz, no noise-suppression) since IMU calibration
        // routines downstream need every raw frame.
        template <typename MessageType>
        struct PublishGate
        {
            std::chrono::milliseconds minInterval{0};
            float noiseEpsilon{0.0f};
            std::optional<MessageType> lastValue;
            std::chrono::steady_clock::time_point lastTime{};
        };

        PublishGate<raccoon::vector3f_t>   gyroGate_;
        PublishGate<raccoon::vector3f_t>   magGate_;
        PublishGate<raccoon::vector3f_t>   linAccelGate_;
        PublishGate<raccoon::vector3f_t>   accelVelGate_;
        PublishGate<raccoon::quaternion_t> dmpOrientGate_;
        PublishGate<raccoon::scalar_f_t>   headingGate_;
        PublishGate<raccoon::scalar_f_t>   tempGate_;
        // Odometry: rate-gate only, noise epsilon = 0 (any change makes
        // it through). Caller cares about position drift below sensor
        // noise so we don't want to swallow micro-updates.
        PublishGate<raccoon::scalar_f_t>   odomPosXGate_;
        PublishGate<raccoon::scalar_f_t>   odomPosYGate_;
        PublishGate<raccoon::scalar_f_t>   odomHeadingGate_;
        PublishGate<raccoon::scalar_f_t>   odomVxGate_;
        PublishGate<raccoon::scalar_f_t>   odomVyGate_;
        PublishGate<raccoon::scalar_f_t>   odomWzGate_;

        // Gate check: returns true (and updates last-{value,time}) if
        // the sample should be published, false if it should be
        // suppressed. Defined inline so callers in DataPublisher.cpp
        // can drop it next to each publish call without indirection;
        // member function so it can touch the private PublishGate
        // template. linfDelta overloads live in DataPublisher.cpp's
        // anonymous namespace.
        template <typename MessageType>
        bool gateAllows(PublishGate<MessageType>& gate,
                        const MessageType& msg,
                        std::chrono::steady_clock::time_point now);

        Result<void> publishAnalogValues(const std::array<AnalogValue, MAX_ANALOG_PORTS>& values);
        Result<void> publishDigitalBits(DigitalValue digitalBits);
        Result<void> publishAccuracy(const ImuAccuracy& accuracy);
    };
} // namespace wombat
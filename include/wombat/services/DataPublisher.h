//
// Created by tobias on 9/14/25.
// Modified by jakob on 11/20/25
//
#pragma once

#include "wombat/core/Types.h"
#include "wombat/core/Result.h"
#include "wombat/core/Logger.h"
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

        Result<void> publishSensorData(const SensorData& data);
        Result<void> publishMotorState(PortId port, const MotorState& state);
        Result<void> publishMotorPosition(PortId port, int32_t position);
        Result<void> publishMotorDone(PortId port, bool done);
        Result<void> publishServoState(PortId port, const ServoState& state);

        /**
         * @brief Publish the current shutdown status
         * @param shutdownFlags Bitmask: bit 0 = servo shutdown, bit 1 = motor shutdown
         * @return Result indicating success or failure
         */
        Result<void> publishShutdownStatus(uint8_t shutdownFlags);

        /**
         * @brief Update and publish CPU temperature if enough time has elapsed
         * @param publishInterval Minimum time between CPU temperature publishes
         * @return Result indicating success or failure
         */
        Result<void> updateCpuTemperature(std::chrono::milliseconds publishInterval = std::chrono::milliseconds(1000));

    private:
        std::shared_ptr<LcmBroker> broker_;
        std::shared_ptr<Logger> logger_;

        // CPU temperature tracking
        std::chrono::steady_clock::time_point lastCpuTempPublishTime_{std::chrono::steady_clock::now()};
        float lastPublishedCpuTemperature_{0.0f};

        // Accuracy throttling and change detection
        std::optional<ImuAccuracy> lastAccuracy_;
        std::chrono::steady_clock::time_point lastAccuracyPublishTime_{};
        static constexpr std::chrono::seconds accuracyPublishInterval_{15};

        exlcm::vector3f_t convertVector3f(const Vector3f& vector) const;
        exlcm::quaternion_t convertQuaternion(const Quaternionf& quaternion) const;
        exlcm::scalar_f_t convertScalarF(float value) const;
        exlcm::scalar_i32_t convertScalarI32(int32_t value) const;
        exlcm::scalar_i8_t convertScalarI8(uint8_t value) const;

        Result<void> publishAnalogValues(const std::array<AnalogValue, MAX_ANALOG_PORTS>& values);
        Result<void> publishDigitalBits(DigitalValue digitalBits);

        /**
         * @brief Read CPU temperature from system thermal zone
         * @return Result containing temperature in Celsius or error
         */
        Result<float> readCpuTemperature();

        /**
         * @brief Publish CPU temperature value
         * @param temperature Temperature in degrees Celsius
         * @return Result indicating success or failure
         */
        Result<void> publishCpuTemperature(float temperature);
        Result<void> publishAccuracy(const ImuAccuracy& accuracy);
    };
} // namespace wombat

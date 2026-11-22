#pragma once

#include "wombat/core/DeviceTypes.h"

#include <chrono>

namespace wombat
{
    // Detects an IMU that does not deliver data. A working IMU never reports exact zeros: at rest
    // the accelerometer shows gravity and the temperature is above zero. When the chip does not
    // answer (broken, not connected, failed self-test of the firmware), the STM32 still sends its
    // packets, with all IMU values at zero, so nothing else in the data shows the fault.
    //
    // The check watches the samples for `window` after the first one. If any IMU value is not zero
    // in that time, the IMU counts as working and the check ends. If every sample was zero, observe()
    // returns true once, when the window is over.
    class ImuHealthCheck
    {
    public:
        using Clock = std::chrono::steady_clock;

        explicit ImuHealthCheck(std::chrono::milliseconds window = std::chrono::seconds(10))
            : window_{window}
        {
        }

        [[nodiscard]] static bool allZero(const SensorData& d)
        {
            const auto zero = [](const Vector3f& v) { return v.x == 0.0f && v.y == 0.0f && v.z == 0.0f; };
            return zero(d.gyro) && zero(d.accelerometer) && zero(d.linearAcceleration)
                && d.dmpOrientation.w == 0.0f && d.dmpOrientation.x == 0.0f
                && d.dmpOrientation.y == 0.0f && d.dmpOrientation.z == 0.0f
                && d.heading == 0.0f && d.temperature == 0.0f;
        }

        // True exactly once: the window is over and all samples in it were zero.
        bool observe(const SensorData& data, Clock::time_point now = Clock::now())
        {
            if (finished_)
                return false;
            if (!allZero(data))
            {
                finished_ = true;
                return false;
            }
            if (!started_)
            {
                started_ = true;
                start_ = now;
                return false;
            }
            if (now - start_ < window_)
                return false;
            finished_ = true;
            return true;
        }

    private:
        std::chrono::milliseconds window_;
        bool started_{false};
        bool finished_{false};
        Clock::time_point start_{};
    };
}

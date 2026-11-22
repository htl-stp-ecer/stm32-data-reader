#pragma once

#include "wombat/core/DeviceTypes.h"

#include <chrono>

namespace wombat
{
    // Detects an IMU that does not deliver data. A working IMU never reports a temperature of exactly
    // 0.0 and its heading changes with the orientation. When the chip does not answer (broken, not
    // connected, failed self-test of the firmware), the STM32 still sends its packets, with the IMU
    // values at zero, so nothing else in the data shows the fault. (Values that the firmware derives,
    // such as the linear acceleration, are not zero then, so they say nothing.)
    //
    // The check watches the samples for `window` after the first one. If the temperature or the heading
    // is not zero in that time, the IMU counts as working and the check ends. If every sample had both at
    // zero, observe() returns true once, when the window is over.
    class ImuHealthCheck
    {
    public:
        using Clock = std::chrono::steady_clock;

        explicit ImuHealthCheck(std::chrono::milliseconds window = std::chrono::seconds(10))
            : window_{window}
        {
        }

        [[nodiscard]] static bool silent(const SensorData& d)
        {
            return d.temperature == 0.0f && d.heading == 0.0f;
        }

        // True exactly once: the window is over and all samples in it were zero.
        bool observe(const SensorData& data, Clock::time_point now = Clock::now())
        {
            if (finished_)
                return false;
            if (!silent(data))
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

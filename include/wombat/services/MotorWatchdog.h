#pragma once

#include <chrono>

namespace wombat
{
    class DeviceController;

    /**
     * Hardware safety watchdog driven by heartbeats from raccoon-lib.
     *
     * Python sends a heartbeat every ~100 ms while missions are running.
     * If no heartbeat arrives within `timeout_` (default 500 ms), the watchdog
     * fires and calls DeviceController::setShutdown(true), which sets the
     * firmware-level shutdown flag via SPI — motors stop even if the Python
     * process hangs or crashes.
     *
     * The watchdog is dormant until the first heartbeat is received, so the
     * robot can boot and wait for light without being killed.
     */
    class MotorWatchdog
    {
    public:
        using Clock = std::chrono::steady_clock;
        using TimePoint = Clock::time_point;
        using Duration = std::chrono::milliseconds;

        explicit MotorWatchdog(Duration timeout = Duration{500});

        /// Called by CommandSubscriber on each heartbeat message.
        void feed();

        /// Called every main-loop tick. Fires shutdown if deadline exceeded.
        void update(DeviceController& controller);

        bool hasFired() const { return fired_; }

    private:
        const Duration timeout_;
        TimePoint lastFeed_{};
        bool armed_{false};
        bool fired_{false};
    };
} // namespace wombat

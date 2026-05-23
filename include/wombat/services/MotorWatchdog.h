#pragma once

#include <chrono>

namespace wombat
{
    class DeviceController;
    class DataPublisher;

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
     *
     * After firing, the watchdog can recover: if `recoverFeeds` heartbeats
     * arrive in succession (each within `timeout_`), shutdown is cleared. A
     * gap longer than `timeout_` during recovery resets the counter, so a
     * single late heartbeat cannot unlatch a still-broken sender.
     */
    class MotorWatchdog
    {
    public:
        using Clock = std::chrono::steady_clock;
        using TimePoint = Clock::time_point;
        using Duration = std::chrono::milliseconds;

        explicit MotorWatchdog(Duration timeout = Duration{500}, int recoverFeeds = 3);

        /// Called by CommandSubscriber on each heartbeat message.
        void feed();

        /// Called every main-loop tick. Fires shutdown if deadline exceeded,
        /// or clears it once recovery hysteresis is satisfied. On both
        /// transitions the shutdown_status channel is updated via the
        /// publisher so the UI can react independently of any user program.
        void update(DeviceController& controller, DataPublisher& publisher);

        bool hasFired() const { return fired_; }

        /// Drop fired state without touching hardware. Use when the UI manually
        /// clears a watchdog-triggered shutdown, so the watchdog can re-arm
        /// and fire again if heartbeats stay missing.
        void resetAfterManualClear();

    private:
        const Duration timeout_;
        const int recoverFeeds_;
        TimePoint lastFeed_{};
        bool armed_{false};
        bool fired_{false};
        int recoveryCount_{0};
    };
} // namespace wombat

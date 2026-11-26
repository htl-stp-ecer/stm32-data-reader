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
     *
     * While the latch is engaged by command, the motors are already off: the
     * watchdog does not fire and heartbeats do not release the latch. The
     * reported latch state therefore keeps the command as its source, e.g.
     * when a program latches the motors and exits.
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

        /// Called by CommandSubscriber after a latch command was applied.
        /// Engaging takes the latch over from the watchdog. Releasing a latch
        /// (from either source) makes the watchdog dormant; the next heartbeat
        /// arms it. Does not touch hardware.
        void onLatchCommand(bool engaged);

    private:
        const Duration timeout_;
        const int recoverFeeds_;
        TimePoint lastFeed_{};
        bool armed_{false};
        bool fired_{false};
        bool latchedByCommand_{false};
        int recoveryCount_{0};
    };
} // namespace wombat

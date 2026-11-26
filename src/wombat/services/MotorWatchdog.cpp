#include "wombat/services/MotorWatchdog.h"
#include "wombat/services/DataPublisher.h"
#include "wombat/services/DeviceController.h"
#include "wombat/core/ShutdownFlags.h"

#include <cstdio>

namespace wombat
{
    namespace
    {
        // Latch state published on Channels::SHUTDOWN_STATUS (see shutdown_flags):
        // a watchdog fire stops the motors and leaves the servos holding their
        // last position (see DeviceController::setShutdown).
        constexpr uint8_t kShutdownClear = shutdown_flags::kClear;
        constexpr uint8_t kShutdownByWatchdog = shutdown_flags::kWatchdogLatch;
    } // namespace

    MotorWatchdog::MotorWatchdog(Duration timeout, int recoverFeeds)
        : timeout_{timeout}, recoverFeeds_{recoverFeeds}
    {
    }

    void MotorWatchdog::feed()
    {
        const auto now = Clock::now();

        if (fired_)
        {
            const auto gap = std::chrono::duration_cast<Duration>(now - lastFeed_);
            if (gap > timeout_)
                recoveryCount_ = 1;
            else
                ++recoveryCount_;
        }

        lastFeed_ = now;
        armed_ = true;
    }

    void MotorWatchdog::onLatchCommand(bool engaged)
    {
        // Releasing drops back to the boot-like dormant state: no program may be
        // running, and the last heartbeat may be long past. feed() re-arms.
        // A release without a latch leaves a running watchdog armed.
        if (!engaged && (fired_ || latchedByCommand_))
            armed_ = false;

        latchedByCommand_ = engaged;
        fired_ = false;
        recoveryCount_ = 0;
    }

    void MotorWatchdog::update(DeviceController& controller, DataPublisher& publisher)
    {
        if (!armed_ || latchedByCommand_)
            return;

        const auto elapsed = std::chrono::duration_cast<Duration>(Clock::now() - lastFeed_);

        if (!fired_)
        {
            if (elapsed > timeout_)
            {
                fired_ = true;
                recoveryCount_ = 0;
                std::fprintf(stderr,
                    "[watchdog] Heartbeat timeout after %lld ms — setting hardware shutdown\n",
                    static_cast<long long>(elapsed.count()));
                controller.setShutdown(true);
                publisher.publishShutdownStatus(kShutdownByWatchdog);
            }
            return;
        }

        // fired_ == true: handle recovery / reset of stale recovery progress.
        if (elapsed > timeout_ && recoveryCount_ > 0)
        {
            recoveryCount_ = 0;
        }
        else if (recoveryCount_ >= recoverFeeds_)
        {
            std::fprintf(stderr,
                "[watchdog] Heartbeat recovered (%d consecutive feeds) — clearing hardware shutdown\n",
                recoveryCount_);
            fired_ = false;
            recoveryCount_ = 0;
            controller.setShutdown(false);
            publisher.publishShutdownStatus(kShutdownClear);
        }
    }
} // namespace wombat

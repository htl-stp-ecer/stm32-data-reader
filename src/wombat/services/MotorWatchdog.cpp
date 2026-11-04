#include "wombat/services/MotorWatchdog.h"
#include "wombat/services/DataPublisher.h"
#include "wombat/services/DeviceController.h"

#include <cstdio>

namespace wombat
{
    namespace
    {
        // Bitmask published on Channels::SHUTDOWN_STATUS.
        //   bit 0 (0x01): servo shutdown active
        //   bit 1 (0x02): motor shutdown active
        //   bit 2 (0x04): source = watchdog (else: user-initiated)
        constexpr uint8_t kShutdownClear = 0x00;
        // Motor + watchdog source. The servo bit is intentionally NOT set: a
        // watchdog fire stops the motors but leaves the servos holding their
        // last position (see DeviceController::setShutdown).
        constexpr uint8_t kShutdownByWatchdog = 0x06;
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

    void MotorWatchdog::resetAfterManualClear()
    {
        fired_ = false;
        recoveryCount_ = 0;
        // Drop back to boot-like dormant state: the user cleared the shutdown
        // explicitly, presumably because no program is running. The watchdog
        // re-arms automatically on the next heartbeat via feed().
        armed_ = false;
    }

    void MotorWatchdog::update(DeviceController& controller, DataPublisher& publisher)
    {
        if (!armed_)
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

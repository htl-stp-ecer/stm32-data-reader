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
        constexpr uint8_t kShutdownByWatchdog = 0x07;
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
        // Keep armed_ as-is: if heartbeats had been arriving, the watchdog
        // stays armed and will re-fire on the next gap. If they never arrived
        // (armed_ still false), the watchdog stays dormant — same as boot.
        lastFeed_ = Clock::now();
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

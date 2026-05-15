#include "wombat/services/MotorWatchdog.h"
#include "wombat/services/DeviceController.h"

#include <cstdio>

namespace wombat
{
    MotorWatchdog::MotorWatchdog(Duration timeout)
        : timeout_{timeout}
    {
    }

    void MotorWatchdog::feed()
    {
        lastFeed_ = Clock::now();
        armed_ = true;
    }

    void MotorWatchdog::update(DeviceController& controller)
    {
        if (!armed_ || fired_)
            return;

        const auto elapsed = std::chrono::duration_cast<Duration>(Clock::now() - lastFeed_);
        if (elapsed > timeout_)
        {
            fired_ = true;
            std::fprintf(stderr,
                "[watchdog] Heartbeat timeout after %lld ms — setting hardware shutdown\n",
                static_cast<long long>(elapsed.count()));
            controller.setShutdown(true);
        }
    }
} // namespace wombat

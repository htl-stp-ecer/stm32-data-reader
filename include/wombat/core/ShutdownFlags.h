#pragma once

#include <cstdint>

namespace wombat::shutdown_flags
{
    // Bitmask published on Channels::SHUTDOWN_STATUS: the state of the hardware
    // latch. The latch stops the MOTORS only. Servos keep holding their last
    // position through a shutdown; releasing them is an explicit action
    // (fully_disable_servos()). Bit 0 is reserved and never set.
    constexpr uint8_t kClear = 0x00;
    constexpr uint8_t kMotor = 0x02;           // motors are latched off
    constexpr uint8_t kSourceWatchdog = 0x04;  // set when the heartbeat watchdog latched, clear for a user shutdown

    constexpr uint8_t kUserLatch = kMotor;
    constexpr uint8_t kWatchdogLatch = kMotor | kSourceWatchdog;
}

//
// Receive-side command tracer for ordering diagnostics.
//
#pragma once

#include <atomic>
#include <cstdint>
#include <fstream>
#include <mutex>
#include <string>

namespace wombat
{
    /**
     * Opt-in receive-side command tracer (companion to raccoon-lib's
     * CommandTrace send-side tracer).
     *
     * When the environment variable ``WOMBAT_CMD_TRACE`` names a writable path,
     * every motor/servo/chassis command is logged twice: once when its transport
     * handler fires (stage ``recv``) and once when a servo position is actually
     * staged into the SPI buffer for the STM32 (stage ``spi``). Otherwise the
     * tracer is inert.
     *
     * This exists to answer "do servo commands execute in the order they were
     * sent?". Each servo port is its own transport channel with no cross-channel
     * ordering guarantee, and the reader drains channels in port order within a
     * spin window, so a command sent later can be applied earlier. The Python
     * analyzer correlates these records with the send side by ``(ch, ts_us)`` and
     * flags commands applied out of their intended send order.
     *
     * Per-line JSON fields:
     *   - ``t_ns``  steady-clock nanoseconds at the event (intra-process order)
     *   - ``w_us``  system wall-clock µs at the event — same clock as the send
     *               side's ``ts_us``, so send->recv latency = w_us - ts_us
     *   - ``rseq``  process-global monotonic counter (arrival/apply order)
     *   - ``stage`` ``recv`` (handler fired) or ``spi`` (staged to SPI buffer)
     *   - ``kind``  short command kind (``servo_pos``, ``motor_vel``, ...)
     *   - ``ch``    channel name ("" for the ``spi`` stage, which is post-decode)
     *   - ``port``  device port, or -1 when not port-scoped
     *   - ``v``     command value (single scalar; the primary setpoint)
     *   - ``ts_us`` the message ``timestamp`` echoed from the wire — the
     *               correlation key with the send side (0 when unavailable,
     *               e.g. the ``spi`` stage)
     */
    class CmdTrace
    {
    public:
        static CmdTrace& instance();

        bool enabled() const { return enabled_; }

        /// Append one record. Cheap no-op when tracing is disabled.
        void record(const char* stage, const char* kind, const std::string& channel, int port,
                    double value, int64_t msgTimestampUsec);

    private:
        CmdTrace();
        ~CmdTrace();

        bool enabled_ = false;
        std::ofstream out_;
        std::mutex mutex_;
        std::atomic<uint64_t> seq_{0};
    };
}

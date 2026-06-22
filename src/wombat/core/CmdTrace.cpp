#include "wombat/core/CmdTrace.h"

#include <chrono>
#include <cstdlib>

namespace wombat
{
    namespace
    {
        int64_t steadyNowNsec()
        {
            return std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::steady_clock::now().time_since_epoch())
                .count();
        }

        // System wall clock in microseconds — the SAME clock the send side uses
        // for ``ts_us``, so send->recv latency is (recv w_us - send ts_us).
        int64_t systemNowUsec()
        {
            return std::chrono::duration_cast<std::chrono::microseconds>(
                    std::chrono::system_clock::now().time_since_epoch())
                .count();
        }
    }

    CmdTrace& CmdTrace::instance()
    {
        static CmdTrace tracer;
        return tracer;
    }

    CmdTrace::CmdTrace()
    {
        const char* path = std::getenv("WOMBAT_CMD_TRACE");
        if (path == nullptr || path[0] == '\0')
            return;
        out_.open(path, std::ios::out | std::ios::trunc);
        enabled_ = out_.is_open();
    }

    CmdTrace::~CmdTrace()
    {
        if (out_.is_open())
            out_.flush();
    }

    void CmdTrace::record(const char* stage, const char* kind, const std::string& channel, int port,
                          double value, int64_t msgTimestampUsec)
    {
        if (!enabled_)
            return;

        const uint64_t seq = seq_.fetch_add(1, std::memory_order_relaxed);
        const int64_t tNs = steadyNowNsec();
        const int64_t wUs = systemNowUsec();

        std::lock_guard<std::mutex> lock(mutex_);
        out_ << "{\"t_ns\":" << tNs << ",\"w_us\":" << wUs << ",\"rseq\":" << seq << ",\"stage\":\""
            << stage << "\",\"kind\":\"" << kind << "\",\"ch\":\"" << channel
            << "\",\"port\":" << port << ",\"v\":" << value << ",\"ts_us\":" << msgTimestampUsec
            << "}\n";
        // Flush periodically, NOT per line: this runs inside the reader's command
        // handlers (the control loop), and a per-line flush on the SD card can
        // block hundreds of ms under write-back pressure — the very stall we are
        // trying to measure. Flushing every 64 records bounds data loss on a
        // crash to <64 lines while keeping the hot path off the disk.
        if ((seq & 0x3F) == 0)
            out_.flush();
    }
}

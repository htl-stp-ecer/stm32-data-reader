#!/usr/bin/env python3
"""
pid_tune.py — Motor velocity PID tuning utility for STM32

COMMANDS
  set-pid    Send Kp/Ki/Kd gains to one or all motors
  tune       Run a velocity step response, record BEMF, save CSV + PNG plot

EXAMPLES
  # Set PID for motor 0
  python3 pid_tune.py set-pid --port 0 --kp 1.0 --ki 0.5 --kd 0.01

  # Set PID for all motors
  python3 pid_tune.py set-pid --all --kp 1.0 --ki 0.5 --kd 0.01

  # Step response: spin motor 0 at velocity 200 for 3 s, record BEMF
  python3 pid_tune.py tune --port 0 --velocity 200

  # Same, but also update PID first
  python3 pid_tune.py tune --port 0 --kp 1.0 --ki 0.5 --kd 0.01 --velocity 200

  # Longer run, custom output file
  python3 pid_tune.py tune --port 0 --velocity 150 --duration 5 --output run1.csv
"""

import argparse
import csv
import sys
import time
from datetime import datetime

try:
    from raccoon_transport import Transport
    from raccoon_transport.channels import Channels
    from raccoon_transport.types.raccoon import vector3f_t, scalar_i32_t
except ImportError:
    print("ERROR: raccoon_transport not installed.")
    print("  Run: pip install -e /path/to/raccoon-transport/python")
    sys.exit(1)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _now_us() -> int:
    return int(time.time() * 1e6)


def _send_pid(transport: Transport, port: int, kp: float, ki: float, kd: float):
    msg = vector3f_t()
    msg.timestamp = _now_us()
    msg.x = float(kp)
    msg.y = float(ki)
    msg.z = float(kd)
    transport.publish(Channels.motor_pid_command(port), msg, reliable=True)
    print(f"  Motor {port}: Kp={kp}  Ki={ki}  Kd={kd}")


def _stop_motor(transport: Transport, port: int):
    msg = scalar_i32_t()
    msg.timestamp = _now_us()
    msg.value = 0  # MOT_MODE_OFF
    transport.publish(Channels.motor_stop_command(port), msg, reliable=True)


def _reset_position(transport: Transport, port: int):
    msg = scalar_i32_t()
    msg.timestamp = _now_us()
    msg.value = 1
    transport.publish(Channels.motor_position_reset_command(port), msg, reliable=True)


def _spin_reliable_ms(transport: Transport, ms: int):
    """Pump the transport event loop for ~ms milliseconds (handles ACKs etc.)."""
    deadline = time.monotonic() + ms / 1000.0
    while time.monotonic() < deadline:
        transport.spin_once(timeout_ms=20)


# ---------------------------------------------------------------------------
# Commands
# ---------------------------------------------------------------------------

def cmd_set_pid(args):
    ports = list(range(4)) if args.all else [args.port]
    transport = Transport()
    try:
        _spin_reliable_ms(transport, 200)
        print("Setting motor PID values:")
        for port in ports:
            _send_pid(transport, port, args.kp, args.ki, args.kd)
        # Pump to allow reliable delivery ACKs
        _spin_reliable_ms(transport, 400)
        print("Done.")
    finally:
        transport.close()


def cmd_tune(args):
    port = args.port
    duration = args.duration
    velocity = args.velocity

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_csv = args.output or f"bemf_port{port}_{ts}.csv"

    # Collected samples as (elapsed_s, bemf)
    samples: list[tuple[float, int]] = []
    t_start: list[float] = [0.0]  # mutable ref for closure

    def on_bemf(_channel, data):
        try:
            msg = scalar_i32_t.decode(data)
        except Exception:
            return
        elapsed = time.monotonic() - t_start[0]
        samples.append((elapsed, msg.value))

    transport = Transport()
    try:
        _spin_reliable_ms(transport, 200)

        # Optionally update PID first
        if args.kp is not None or args.ki is not None or args.kd is not None:
            missing = [n for n, v in [("--kp", args.kp), ("--ki", args.ki), ("--kd", args.kd)] if v is None]
            if missing:
                print(f"ERROR: When providing PID gains, all three are required. Missing: {', '.join(missing)}")
                return
            print(f"Updating PID for motor {port}:")
            _send_pid(transport, port, args.kp, args.ki, args.kd)
            _spin_reliable_ms(transport, 400)

        # Reset position counter so we start clean
        _reset_position(transport, port)
        _spin_reliable_ms(transport, 400)

        # Subscribe to BEMF
        transport.subscribe(Channels.back_emf(port), on_bemf)
        _spin_reliable_ms(transport, 100)

        # --- Step response ---
        print(f"Step response: port={port}  velocity={velocity}  duration={duration}s")
        print("Recording... (Ctrl+C to abort early)")

        vel_msg = scalar_i32_t()
        vel_msg.value = velocity

        t_start[0] = time.monotonic()
        end_time = t_start[0] + duration

        try:
            while time.monotonic() < end_time:
                vel_msg.timestamp = _now_us()
                transport.publish(Channels.motor_velocity_command(port), vel_msg)
                transport.spin_once(timeout_ms=20)
        except KeyboardInterrupt:
            print("\nAborted early.")

        # Stop motor
        _stop_motor(transport, port)

        # Collect trailing data (motor coasting to stop)
        _spin_reliable_ms(transport, 600)

    finally:
        transport.close()

    # --- Save CSV ---
    if not samples:
        print("WARNING: No BEMF samples received. Is the reader running?")
        return

    with open(output_csv, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["time_s", "bemf"])
        writer.writerows(samples)

    sample_rate = len(samples) / (samples[-1][0] if samples[-1][0] > 0 else 1)
    print(f"Saved {len(samples)} samples ({sample_rate:.0f} Hz) → {output_csv}")

    # --- Plot ---
    if args.no_plot:
        return

    try:
        import matplotlib
        matplotlib.use("Agg")  # Headless-safe; works with or without display
        import matplotlib.pyplot as plt

        times = [s[0] for s in samples]
        values = [s[1] for s in samples]

        fig, ax = plt.subplots(figsize=(10, 4))
        ax.plot(times, values, linewidth=1.2, color="#1f77b4", label="BEMF")
        ax.axhline(y=velocity, color="#d62728", linestyle="--", linewidth=1, label=f"Target: {velocity}")
        ax.axvline(x=0, color="#2ca02c", linestyle=":", linewidth=1, alpha=0.7, label="Step start")
        ax.axvline(x=duration, color="#ff7f0e", linestyle=":", linewidth=1, alpha=0.7, label="Motor stop")

        title = f"Motor {port} velocity step response"
        if args.kp is not None:
            title += f"  |  Kp={args.kp}  Ki={args.ki}  Kd={args.kd}"
        ax.set_title(title)
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("BEMF (ticks)")
        ax.legend(loc="upper right")
        ax.grid(True, alpha=0.3)
        fig.tight_layout()

        plot_file = output_csv.replace(".csv", ".png")
        fig.savefig(plot_file, dpi=150)
        plt.close(fig)
        print(f"Plot saved → {plot_file}")

    except ImportError:
        print("matplotlib not installed — skipping plot (CSV is saved).")
        print("  Install with: pip install matplotlib")


# ---------------------------------------------------------------------------
# Argument parsing
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Motor velocity PID tuning utility",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    sub = parser.add_subparsers(dest="command", required=True)

    # --- set-pid ---
    p_set = sub.add_parser("set-pid", help="Send PID gains to the STM32")
    grp = p_set.add_mutually_exclusive_group(required=True)
    grp.add_argument("--port", type=int, choices=[0, 1, 2, 3], help="Motor port (0-3)")
    grp.add_argument("--all", action="store_true", help="Apply to all four motors")
    p_set.add_argument("--kp", type=float, required=True, help="Proportional gain")
    p_set.add_argument("--ki", type=float, required=True, help="Integral gain")
    p_set.add_argument("--kd", type=float, required=True, help="Derivative gain")

    # --- tune ---
    p_tune = sub.add_parser(
        "tune",
        help="Velocity step response: run motor, record BEMF, save CSV + plot",
    )
    p_tune.add_argument("--port", type=int, choices=[0, 1, 2, 3], required=True)
    p_tune.add_argument("--velocity", type=int, required=True, help="Velocity target (BEMF ticks/s)")
    p_tune.add_argument("--duration", type=float, default=3.0, help="Step duration in seconds (default: 3)")
    p_tune.add_argument("--kp", type=float, default=None, help="Set Kp before running (optional)")
    p_tune.add_argument("--ki", type=float, default=None, help="Set Ki before running (optional)")
    p_tune.add_argument("--kd", type=float, default=None, help="Set Kd before running (optional)")
    p_tune.add_argument("--output", type=str, default=None, help="CSV output path (auto-named if omitted)")
    p_tune.add_argument("--no-plot", action="store_true", help="Skip plot generation")

    args = parser.parse_args()

    if args.command == "set-pid":
        cmd_set_pid(args)
    elif args.command == "tune":
        cmd_tune(args)


if __name__ == "__main__":
    main()

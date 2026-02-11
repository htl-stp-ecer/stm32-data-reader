#!/usr/bin/env python3
"""
Motor PID Tuning Helper
- Sets all 4 motors via power (direct PWM) or velocity (PID-controlled)
- Logs BEMF, position, and timestamps to CSV
- Ctrl+C to brake and exit

Usage:
  python3 motor_pid_tuning.py                  # default: power mode, 70%
  python3 motor_pid_tuning.py --mode velocity  # velocity mode (PID-controlled)
  python3 motor_pid_tuning.py --speed 50       # 50% speed
"""

import argparse
import csv
import lcm
import os
import select
import time

from exlcm.scalar_i32_t import scalar_i32_t

# ── Configuration ──────────────────────────────────────────────
MAX_SPEED = 1500
NUM_MOTORS = 4
CSV_FILE = "motor_pid_log.csv"
# ───────────────────────────────────────────────────────────────

parser = argparse.ArgumentParser(description="Motor PID Tuning Helper")
parser.add_argument("--mode", choices=["power", "velocity"], default="power",
                    help="power = direct PWM, velocity = PID-controlled (default: power)")
parser.add_argument("--speed", type=int, default=70,
                    help="speed in percent 0-100 (default: 70)")
args = parser.parse_args()

MODE = args.mode
SPEED_VALUE = args.speed

lc = lcm.LCM()

# Latest data per motor
motor_data = {i: {} for i in range(NUM_MOTORS)}
start_time = time.time()


def bemf_handler(channel, data):
    msg = scalar_i32_t.decode(data)
    port = int(channel.split("/")[2])
    motor_data[port]["bemf"] = msg.value
    motor_data[port]["bemf_ts"] = msg.timestamp


def position_handler(channel, data):
    msg = scalar_i32_t.decode(data)
    port = int(channel.split("/")[2])
    motor_data[port]["position"] = msg.value
    motor_data[port]["pos_ts"] = msg.timestamp


for i in range(NUM_MOTORS):
    lc.subscribe(f"libstp/bemf/{i}/value", bemf_handler)
    lc.subscribe(f"libstp/motor/{i}/position", position_handler)


def send_command(value, mode=None):
    """Send motor command to all motors using the given mode."""
    m = mode or MODE
    channel_suffix = "power_cmd" if m == "power" else "velocity_cmd"
    if m != "power":
        value = int(MAX_SPEED * value / 100)

    msg = scalar_i32_t()
    msg.timestamp = int(time.time() * 1e6)
    msg.value = value
    for i in range(NUM_MOTORS):
        lc.publish(f"libstp/motor/{i}/{channel_suffix}", msg.encode())


def brake():
    """Active brake: send power_cmd=0 (short-circuit braking on STM32)."""
    send_command(0, mode="power")
    print("\nMotors braked.")


# CSV setup
csv_file = open(CSV_FILE, "w", newline="")
writer = csv.writer(csv_file)
BRAKE_TAIL_SECONDS = 3  # keep recording after brake

writer.writerow([
    "elapsed_s", "brake_elapsed_s",
    "bemf_0", "pos_0", "ts_0",
    "bemf_1", "pos_1", "ts_1",
    "bemf_2", "pos_2", "ts_2",
    "bemf_3", "pos_3", "ts_3",
])

print(f"Mode: {MODE} | Speed: {args.speed}% (value={SPEED_VALUE})")
if MODE == "velocity":
    print(f"Velocity command value sent to motors: {int(MAX_SPEED * SPEED_VALUE / 100)}")
print(f"Logging to {CSV_FILE}")
print("Press Ctrl+C to brake and stop.\n")

brake_time = None  # set when braking starts


def log_and_print():
    """Write one CSV row and print live status. Returns elapsed seconds."""
    elapsed = round(time.time() - start_time, 4)
    brake_elapsed = round(time.time() - brake_time, 4) if brake_time else ""
    row = [elapsed, brake_elapsed]
    for i in range(NUM_MOTORS):
        d = motor_data[i]
        row.extend([
            d.get("bemf", ""),
            d.get("position", ""),
            d.get("bemf_ts", d.get("pos_ts", "")),
        ])
    writer.writerow(row)

    label = "BRAKE" if brake_time else "RUN"
    parts = []
    for i in range(NUM_MOTORS):
        d = motor_data[i]
        b = d.get("bemf", "?")
        p = d.get("position", "?")
        parts.append(f"M{i}[bemf={b} pos={p}]")
    print(f"\r{elapsed:>8.2f}s [{label}]  " + "  ".join(parts), end="", flush=True)
    return elapsed


send_command(SPEED_VALUE)

try:
    while True:
        rfds, _, _ = select.select([lc.fileno()], [], [], 0.01)
        if rfds:
            lc.handle()
        log_and_print()

except KeyboardInterrupt:
    pass

# Brake and keep recording for BRAKE_TAIL_SECONDS
brake()
brake_time = time.time()
print(f"Recording brake tail for {BRAKE_TAIL_SECONDS}s...")

while time.time() - brake_time < BRAKE_TAIL_SECONDS:
    rfds, _, _ = select.select([lc.fileno()], [], [], 0.01)
    if rfds:
        lc.handle()
    log_and_print()

csv_file.close()
print(f"\nCSV saved to {CSV_FILE}")

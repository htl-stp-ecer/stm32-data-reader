#!/usr/bin/env python3
"""
Bus servo (Feetech STS, e.g. STS3215) end-to-end test over raccoon transport.

Runs on the Pi next to stm32_data_reader:

  python3 bus_servo_test.py            # servo on port 0
  python3 bus_servo_test.py --port 2

Puts the port into bus mode (servo mode 3), then exercises a full-speed
position move, a slow eased move, "wait until done", continuous rotation in
both directions and the multi-turn position feedback. Ends relaxed (torque off).
The servo moves --amplitude deg either side of --center and spins a few turns
(skip that with --no-spin if something is mounted that limits its travel).
"""
import argparse
import sys
import time

try:
    from raccoon_transport.types.raccoon import scalar_f_t, scalar_i8_t, vector3f_t, scalar_i32_t
except ImportError:
    print("ERROR: raccoon_transport not installed.")
    sys.exit(1)


def open_transport():
    # The pure-Python raccoon_transport.Transport is an in-process queue; on
    # the Pi the shared-memory transport the reader uses comes from raccoon.
    try:
        import raccoon
        return raccoon.get_transport()
    except ImportError:
        from raccoon_transport import Transport
        return Transport()

SERVO_MODE_BUS = 3


def ch(port: int, name: str) -> str:
    # Spelled out so the script also runs against an older installed
    # raccoon_transport that lacks the bus servo channel helpers.
    return f"raccoon/servo/{port}/{name}"


def now_us() -> int:
    return int(time.time() * 1e6)


class BusServo:
    def __init__(self, transport, port: int):
        self.t = transport
        self.port = port
        self.done = None
        self.done_seen_running = False
        self.feedback = None  # (deg, deg/s, load %)
        self.health = None  # (V, C, status / -1)
        self._subs = [
            transport.subscribe(ch(port, "done"), self._on_done),
            transport.subscribe(ch(port, "feedback"), self._on_feedback),
            transport.subscribe(ch(port, "health"), self._on_health),
        ]

    def _on_done(self, _c, data):
        self.done = scalar_i32_t.decode(data).value
        if self.done == 0:
            self.done_seen_running = True

    def _on_feedback(self, _c, data):
        m = vector3f_t.decode(data)
        self.feedback = (m.x, m.y, m.z)

    def _on_health(self, _c, data):
        m = vector3f_t.decode(data)
        self.health = (m.x, m.y, m.z)

    def _pump(self, timeout_ms: int):
        if hasattr(self.t, "spin_once"):
            self.t.spin_once(timeout_ms=timeout_ms)
        else:
            time.sleep(timeout_ms / 1000.0)  # shared transport delivers on its own thread

    def spin(self, seconds: float):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self._pump(10)

    def _send(self, name, msg):
        msg.timestamp = now_us()
        self.done_seen_running = False
        # The shared-memory transport does not drop messages; only the
        # fallback LCM-style transport needs reliable delivery.
        if hasattr(self.t, "spin_once"):
            self.t.publish(ch(self.port, name), msg, reliable=True)
        else:
            self.t.publish(ch(self.port, name), msg)

    def clear_shutdown(self):
        # A finished raccoon program leaves the reader in shutdown (motors and
        # servos off at firmware level); a new program clears it first.
        m = scalar_i32_t()
        m.value = 0
        m.timestamp = now_us()
        if hasattr(self.t, "spin_once"):
            self.t.publish("raccoon/system/shutdown_cmd", m, reliable=True)
        else:
            self.t.publish("raccoon/system/shutdown_cmd", m)

    def set_bus_mode(self):
        m = scalar_i8_t()
        m.dir = SERVO_MODE_BUS
        self._send("mode_cmd", m)

    def position(self, degrees: float):
        m = scalar_f_t()
        m.value = degrees
        self._send("position_cmd", m)

    def smooth(self, degrees: float, deg_per_s: float, easing: int = 3):
        m = vector3f_t()
        m.x, m.y, m.z = degrees, deg_per_s, float(easing)
        self._send("smooth_cmd", m)

    def velocity(self, deg_per_s: float):
        m = scalar_f_t()
        m.value = deg_per_s
        self._send("velocity_cmd", m)

    def wait_done(self, timeout: float = 5.0) -> tuple[int, float]:
        """Blocks until the latest command finished. Returns (done value, seconds)."""
        start = time.monotonic()
        while time.monotonic() - start < timeout:
            self._pump(5)
            # The reader flips done to 0 as soon as it sends a new command;
            # require that (or 300 ms) so a stale "done" never counts.
            fresh = self.done_seen_running or time.monotonic() - start > 0.3
            if fresh and self.done in (1, 2):
                return self.done, time.monotonic() - start
        return 0, time.monotonic() - start

    def wait_connected(self, timeout: float = 6.0) -> bool:
        start = time.monotonic()
        while time.monotonic() - start < timeout:
            self._pump(10)
            if self.health is not None and self.health[2] >= 0:
                return True
        return False


def report(label: str, servo: BusServo, result=None):
    fb = servo.feedback or (float("nan"),) * 3
    line = f"  {label:<34} pos={fb[0]:8.1f} deg  speed={fb[1]:7.1f} deg/s  load={fb[2]:6.1f} %"
    if result is not None:
        done, secs = result
        state = {0: "TIMEOUT", 1: "reached", 2: "STALLED"}[done]
        line += f"  -> {state} after {secs:.2f} s"
    print(line)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=0)
    parser.add_argument("--center", type=float, default=180.0, help="deg, middle of the free range")
    parser.add_argument("--amplitude", type=float, default=60.0, help="deg either side of --center")
    parser.add_argument("--no-spin", action="store_true", help="skip continuous rotation (mechanical stop)")
    args = parser.parse_args()

    t = open_transport()
    servo = BusServo(t, args.port)
    # Give the reader time to discover this new publisher before the first
    # command; messages sent right after opening the transport can be missed.
    servo.spin(1.0)
    c, a = args.center, args.amplitude

    print(f"Bus servo test on port {args.port}")
    servo.clear_shutdown()
    servo.spin(0.2)
    servo.set_bus_mode()
    if not servo.wait_connected():
        print("  servo did not connect (health channel never reported it)")
        return 1
    servo.spin(0.3)
    v, temp, status = servo.health
    print(f"  connected: {v:.1f} V, {temp:.0f} C, status 0x{int(status):02X}")
    report("relaxed (torque off)", servo)

    print("Position, full speed:")
    for target in (c, c + a, c - a, c):
        servo.position(target)
        report(f"position_cmd {target:.0f} deg", servo, servo.wait_done())

    print("Smooth, servo-native profile:")
    servo.smooth(c + a, 60.0, easing=3)
    report(f"smooth_cmd {c + a:.0f} deg @ 60 deg/s eased", servo, servo.wait_done())
    servo.smooth(c, 180.0, easing=0)
    report(f"smooth_cmd {c:.0f} deg @ 180 deg/s linear", servo, servo.wait_done())

    if not args.no_spin:
        print("Continuous rotation:")
        start = servo.feedback[0]
        servo.velocity(180.0)
        servo.spin(3.0)
        report("velocity_cmd +180 deg/s for 3 s", servo)
        print(f"  {'':<34} turned {servo.feedback[0] - start:+.0f} deg (multi-turn)")
        start = servo.feedback[0]
        servo.velocity(-360.0)
        servo.spin(2.0)
        report("velocity_cmd -360 deg/s for 2 s", servo)
        print(f"  {'':<34} turned {servo.feedback[0] - start:+.0f} deg (multi-turn)")
        servo.velocity(0.0)
        servo.spin(0.5)
        report("velocity_cmd 0", servo)

        print("Back to position mode:")
        servo.position(c)
        report(f"position_cmd {c:.0f} deg", servo, servo.wait_done())

    servo.set_bus_mode()
    servo.spin(0.5)
    report("relaxed again", servo)
    return 0


if __name__ == "__main__":
    code = main()
    try:
        import raccoon
        raccoon.shutdown_transport()
    except ImportError:
        pass
    sys.exit(code)

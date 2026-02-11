import lcm
import sys
import time

sys.path.insert(0, sys.path[0])  # ensure local exlcm package is found
from exlcm.vector3f_t import vector3f_t


def set_motor_pid(port: int, kp: float, ki: float, kd: float):
    lc = lcm.LCM()

    msg = vector3f_t()
    msg.timestamp = int(time.time() * 1e6)
    msg.x = kp
    msg.y = ki
    msg.z = kd

    channel = f"libstp/motor/{port}/pid_cmd"
    lc.publish(channel, msg.encode())
    print(f"Published PID on {channel}: kp={kp}, ki={ki}, kd={kd}")


if __name__ == "__main__":
    port = int(sys.argv[1]) if len(sys.argv) > 1 else 0
    kp = float(sys.argv[2]) if len(sys.argv) > 2 else 2.0
    ki = float(sys.argv[3]) if len(sys.argv) > 3 else 0.1
    kd = float(sys.argv[4]) if len(sys.argv) > 4 else 0.05

    set_motor_pid(port, kp, ki, kd)

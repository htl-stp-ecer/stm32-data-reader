#!/usr/bin/env python3
import lcm

from exlcm.scalar_i8_t import scalar_i8_t
from exlcm.scalar_f_t import scalar_f_t
from exlcm.scalar_i32_t import scalar_i32_t
from exlcm.vector3f_t import vector3f_t


def motor_handler(channel, msg):
    data = scalar_i32_t.decode(msg)
    print(f"[Motor] Channel: {channel}, value: {data.value}")

def servo_handler(channel, msg):
    data = scalar_i32_t.decode(msg)
    print(f"[Servo] Channel: {channel}, value: {data.value}")

def gyro_handler(channel, msg):
    data = vector3f_t.decode(msg)
    print(f"[Gyro] Channel: {channel}, x={data.x}, y={data.y}, z={data.z}")

def accel_handler(channel, msg):
    data = vector3f_t.decode(msg)
    print(f"[Accel] Channel: {channel}, x={data.x}, y={data.y}, z={data.z}")

def mag_handler(channel, msg):
    data = vector3f_t.decode(msg)
    print(f"[Mag] Channel: {channel}, x={data.x}, y={data.y}, z={data.z}")

def temp_handler(channel, msg):
    data = scalar_f_t.decode(msg)
    print(f"[Temp] Channel: {channel}, value={data.value}")

def battery_handler(channel, msg):
    data = scalar_f_t.decode(msg)
    print(f"[Battery] Channel: {channel}, value={data.value}")


def analog_handler(channel, msg):
    data = scalar_i32_t.decode(msg)
    print(f"[Analog] Channel: {channel}, value={data.value}")

def digital_handler(channel, msg):
    data = scalar_i32_t.decode(msg)
    print(f"[Digital] Channel: {channel}, value={data.value}")

def bemf_handler(channel, msg):
    data = scalar_i32_t.decode(msg)
    print(f"[BEMF] Channel: {channel}, value={data.value}")


lc = lcm.LCM()

# Subscribe to all relevant channels
# lc.subscribe("libstp/battery/voltage", battery_handler)
# lc.subscribe("motors_0_power_cmd", motor_handler)
# lc.subscribe("servos_0_position_cmd", servo_handler)
# lc.subscribe("libstp/gyro/value", gyro_handler)
# lc.subscribe("libstp/accel/value", accel_handler)
# lc.subscribe("libstp/mag/value", mag_handler)
# lc.subscribe("libstp/temp/value", temp_handler)
#
# for i in range(6):
#     lc.subscribe(f"libstp/analog/{i}/value", analog_handler)
#
for i in range(11):
    lc.subscribe(f"libstp/digital/{i}/value", digital_handler)
#
# lc.subscribe("libstp/bemf/0/value", bemf_handler)

print("Listening for LCM messages...")
try:
    while True:
        lc.handle()
except KeyboardInterrupt:
    print("Exiting...")

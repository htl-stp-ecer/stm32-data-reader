//
// Feetech STS/SCS serial bus servos (e.g. Seeed STS3215) on the servo ports.
//
// The STS bus is half-duplex TTL UART (1 Mbit/s, 8N1) on the signal wire. The
// servo port pins (PC8, PC7, PE6, PE5) have no USART TX alternate function, so
// the UART is bit-banged against the DWT cycle counter. The TXS0104 level
// shifter between the pins and the servo headers is bidirectional, so
// releasing a pin (input) lets the servo drive the line back.
//
// A port is a bus port while the Pi puts it in SERVO_PORT_MODE_BUS. The Pi then
// sends a BusServoCommand per port and reads back a BusServoState (see
// pi_buffer.h). bus_servo_update() runs at most one bus transaction per call.
// Each transaction runs with interrupts disabled (~100-300 us); the 1 MHz
// microSeconds tick is advanced by the time it was blocked.
//

#ifndef BUS_SERVO_H
#define BUS_SERVO_H

#include <stdint.h>

#define BUS_SERVO_BROADCAST_ID 0xFE

/* STS3215 control table (subset) */
#define STS_REG_ID               5
#define STS_REG_BAUD             6
#define STS_REG_OPERATING_MODE   33 // 0 = position, 1 = wheel (constant speed)
#define STS_REG_TORQUE_ENABLE    40
#define STS_REG_ACCELERATION     41 // x100 steps/s^2, 0 = no ramp
#define STS_REG_GOAL_POSITION    42 // 2 bytes, little endian, 0..4095
#define STS_REG_GOAL_SPEED       46 // 2 bytes, bit 15 = sign
#define STS_REG_LOCK             55
#define STS_REG_PRESENT_POSITION 56 // 2 bytes, little endian
#define STS_REG_PRESENT_VOLTAGE  62 // 0.1 V
#define STS_REG_PRESENT_TEMP     63 // deg C

void bus_servo_init(void);

/* Called by the servo code when a port enters or leaves SERVO_PORT_MODE_BUS.
 * Disabling releases torque (best effort) and hands the pin back to its PWM timer. */
void bus_servo_set_port_enabled(int port, uint8_t enabled);

/* Main loop: applies new commands, polls servo state, reconnects lost servos. */
void bus_servo_update(uint32_t nowMs);

/* Low-level access. Each returns the servo's error byte (>= 0), or < 0 if no
 * valid reply arrived. For a ping to the broadcast ID, *foundId receives the
 * replying servo's ID. */
int bus_servo_ping(int port, uint8_t id, uint8_t* foundId);
int bus_servo_read(int port, uint8_t id, uint8_t addr, uint8_t len, uint8_t* out);
int bus_servo_write(int port, uint8_t id, uint8_t addr, const uint8_t* data, uint8_t len);

#endif //BUS_SERVO_H

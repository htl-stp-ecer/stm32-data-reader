// SpiMock.c — ultra-cheap userspace mock for spi.h
// Build: compile and link this file instead of your real Spi.c
// License: CC0/Unlicense — do whatever you want.

#include "wombat/hardware/Spi.h"

#include <cstring>
#include <ctime>

// -------------------- tiny "HW" model --------------------

#ifndef SPI_MOCK_MAX_MOTORS
#define SPI_MOCK_MAX_MOTORS 8
#endif

#ifndef SPI_MOCK_MAX_SERVOS
#define SPI_MOCK_MAX_SERVOS 8
#endif

#ifndef SPI_MOCK_MAX_ANALOG
#define SPI_MOCK_MAX_ANALOG 8
#endif

static bool g_inited = false;
static bool g_spi_mode = false; // ignored, but kept for completeness

static struct timespec g_t0;
static struct timespec g_t_last;
static uint32_t g_last_update_us = 0;
static double g_t_secs = 0.0;

// motors
static MotorDir g_motor_dir[SPI_MOCK_MAX_MOTORS];
static uint32_t g_motor_cmd[SPI_MOCK_MAX_MOTORS]; // arbitrary units

// servos
static ServoMode g_servo_mode[SPI_MOCK_MAX_SERVOS];
static uint16_t g_servo_pos[SPI_MOCK_MAX_SERVOS]; // 0..2047

// shutdown flags
static bool g_shutdown_servo = false;
static bool g_shutdown_motor = false;

// imu, battery
static float g_gyro[3] = {0};
static float g_accel[3] = {0};
static float g_mag[3] = {0};
static float g_temp_c = 28.0f;
static float g_batt_v = 12.3f;

// digital/analog
static uint16_t g_digital = 0x0001; // simple walking 1
static uint16_t g_analog[SPI_MOCK_MAX_ANALOG];

// -------------------- helpers --------------------

static inline double ts_diff_s(const struct timespec* a, const struct timespec* b)
{
    return (double)(a->tv_sec - b->tv_sec) + (double)(a->tv_nsec - b->tv_nsec) / 1e9;
}

static inline uint32_t clamp_u32(uint32_t v, uint32_t lo, uint32_t hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

static inline uint16_t clamp_u16(uint16_t v, uint16_t lo, uint16_t hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

static inline int idx_ok(uint8_t idx, int maxn)
{
    return idx < (uint8_t)maxn;
}

// -------------------- lifecycle --------------------

bool spi_init(uint32_t speed_hz)
{
    (void)speed_hz; // not used in the mock

    memset(g_motor_dir, 0, sizeof(g_motor_dir));
    memset(g_motor_cmd, 0, sizeof(g_motor_cmd));
    memset(g_servo_mode, 0, sizeof(g_servo_mode));
    memset(g_servo_pos, 0, sizeof(g_servo_pos));
    memset(g_analog, 0, sizeof(g_analog));

    g_shutdown_servo = false;
    g_shutdown_motor = false;

    g_gyro[0] = g_gyro[1] = g_gyro[2] = 0.0f;
    g_accel[0] = 0.0f;
    g_accel[1] = 0.0f;
    g_accel[2] = 1.0f; // gravity-ish
    g_mag[0] = 0.3f;
    g_mag[1] = 0.0f;
    g_mag[2] = 0.5f;
    g_temp_c = 28.0f;
    g_batt_v = 12.3f;

    g_digital = 0x0001;

    clock_gettime(CLOCK_MONOTONIC, &g_t0);
    g_t_last = g_t0;
    g_last_update_us = 0;
    g_t_secs = 0.0;

    g_inited = true;
    return true;
}

bool spi_update(void)
{
    if (!g_inited) return false;

    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);

    double dt = ts_diff_s(&now, &g_t_last);
    if (dt < 0) dt = 0;
    if (dt > 0.2) dt = 0.2; // clamp if app stalls

    g_t_secs += dt;
    g_last_update_us = (uint32_t)(dt * 1e6);
    g_t_last = now;

    // IMU: tiny smooth motion
    const float w = 2.0f * (float)M_PI * 0.2f; // 0.2 Hz
    float t = (float)g_t_secs;

    g_gyro[0] = 5.0f * sinf(w * t);
    g_gyro[1] = 3.0f * cosf(w * t * 0.7f);
    g_gyro[2] = 2.0f * sinf(w * t * 1.3f);

    g_accel[0] = 0.02f * sinf(w * t * 0.5f);
    g_accel[1] = 0.02f * cosf(w * t * 0.5f);
    g_accel[2] = 1.00f + 0.01f * sinf(w * t);

    g_mag[0] = 0.30f + 0.05f * sinf(w * t * 0.3f);
    g_mag[1] = 0.10f + 0.04f * cosf(w * t * 0.4f);
    g_mag[2] = 0.50f + 0.03f * sinf(w * t * 0.5f);

    g_temp_c = 28.0f + 0.3f * sinf(w * t * 0.2f);

    // Battery: very slow drift down + tiny ripple
    g_batt_v -= (float)(dt * 0.0005); // ~0.001 V every 2 s
    if (g_batt_v < 10.5f) g_batt_v = 12.3f; // wrap
    g_batt_v += 0.02f * sinf(2.0f * (float)M_PI * 2.0f * t); // 2 Hz ripple

    // Digital: rotate a single '1' bit
    g_digital = (uint16_t)((g_digital << 1) | (g_digital >> 15));

    // Analog: deterministic pattern per index + a small wobble
    for (int i = 0; i < SPI_MOCK_MAX_ANALOG; ++i)
    {
        float base = 800.0f + 100.0f * (float)i; // 800, 900, ...
        float wob = 50.0f * sinf(w * t * (0.6f + 0.1f * (float)i));
        int val = (int)(base + wob);
        if (val < 0) val = 0;
        if (val > 4095) val = 4095;
        g_analog[i] = (uint16_t)val;
    }

    return true;
}

void set_spi_mode(bool on)
{
    g_spi_mode = on; // no-op in mock
}

bool spi_force_update(void)
{
    return spi_update();
}

void spi_close(void)
{
    g_inited = false;
}

// -------------------- setters (TX) --------------------

void set_shutdown_flag(uint8_t bit, bool value)
{
    switch (bit)
    {
    case SHUTDOWN_SERVO_FLAG: g_shutdown_servo = value;
        break;
    case SHUTDOWN_MOTOR_FLAG: g_shutdown_motor = value;
        break;
    default: /* ignore */ break;
    }
}

void set_motor(uint8_t port, MotorDir dir, uint32_t value)
{
    if (!idx_ok(port, SPI_MOCK_MAX_MOTORS)) return;
    g_motor_dir[port] = dir;
    g_motor_cmd[port] = value;
}

void set_servo_mode(uint8_t port, ServoMode mode)
{
    if (!idx_ok(port, SPI_MOCK_MAX_SERVOS)) return;
    g_servo_mode[port] = mode;
}

void set_servo_pos(uint8_t port, uint16_t raw /* 0-2047 */)
{
    if (!idx_ok(port, SPI_MOCK_MAX_SERVOS)) return;
    g_servo_pos[port] = clamp_u16(raw, 0, 2047);
}

// -------------------- getters (RX) --------------------

uint16_t get_servo_pos(uint8_t port)
{
    if (!idx_ok(port, SPI_MOCK_MAX_SERVOS)) return 0;
    // If servo shutdown is set or mode is disabled, just report 0
    if (g_shutdown_servo || g_servo_mode[port] != SERVO_MODE_ENABLED)
        return 0;
    // Tiny +/-1 jitter to look alive, driven by g_t_secs
    int jitter = ((int)(g_t_secs * 1000.0) + port) % 3 - 1; // -1..+1
    int v = (int)g_servo_pos[port] + jitter;
    return (uint16_t)clamp_u16((uint16_t)(v < 0 ? 0 : v), 0, 2047);
}

uint32_t last_update_us(void)
{
    return g_last_update_us;
}

// IMU
float gyroX(void) { return g_gyro[0]; }
float gyroY(void) { return g_gyro[1]; }
float gyroZ(void) { return g_gyro[2]; }

float accelX(void) { return g_accel[0]; }
float accelY(void) { return g_accel[1]; }
float accelZ(void) { return g_accel[2]; }

float magX(void) { return g_mag[0]; }
float magY(void) { return g_mag[1]; }
float magZ(void) { return g_mag[2]; }

float imuTemperature(void) { return g_temp_c; }

// Motors → back-EMF (toy model)
int32_t bemf(uint8_t mot)
{
    if (!idx_ok(mot, SPI_MOCK_MAX_MOTORS)) return 0;
    if (g_shutdown_motor || g_motor_dir[mot] == MOTOR_DIR_OFF) return 0;

    int sign = (g_motor_dir[mot] == MOTOR_DIR_CCW) ? -1 : +1;
    // crude scaling to keep values small-ish
    int32_t v = (int32_t)(g_motor_cmd[mot] / 4u);
    return sign * v;
}

// IO
uint16_t analog_in(uint8_t idx)
{
    if (!idx_ok(idx, SPI_MOCK_MAX_ANALOG)) return 0;
    return g_analog[idx];
}

uint16_t digital_raw(void)
{
    return g_digital;
}

bool digital(uint8_t bit)
{
    if (bit >= 16) return false;
    return (g_digital >> bit) & 0x1;
}

float battery_voltage(void)
{
    return g_batt_v;
}

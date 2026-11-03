//
// Feetech STS bus servos on the servo ports: bit-banged half-duplex UART plus
// the per-port command/feedback state machine. See bus_servo.h for why this is
// software and not a USART.
//

#include "../../include/Actors/bus_servo.h"

#include <stdio.h>
#include <string.h>
#include "main.h"
#include "Actors/servo.h"
#include "Communication/communication_with_pi.h"
#include "Hardware/timer.h"
#include "Hardware/timerInit.h"

#define BUS_BAUD 1000000u
#define BUS_FIRST_BYTE_TIMEOUT_US 600u // STS return delay defaults to 0, reply follows within a few us
#define BUS_INTER_BYTE_TIMEOUT_US 50u

#define STS_INST_PING  0x01
#define STS_INST_READ  0x02
#define STS_INST_WRITE 0x03

#define STS_OPERATING_MODE_POSITION 0
#define STS_OPERATING_MODE_WHEEL    1

#define BUS_MAX_PACKET 32

#define BUS_MIN_GAP_MS         1 // between two transactions on any port, so interrupts get to run
#define BUS_SEARCH_INTERVAL_MS 500
#define BUS_POLL_ACTIVE_MS     20 // while a move is in progress or the servo spins
#define BUS_POLL_IDLE_MS       100
#define BUS_MAX_FAILURES       5 // consecutive failed transactions before the servo counts as lost
#define BUS_DONE_TOLERANCE     10 // steps (~0.9 deg)
#define BUS_STALL_TIME_MS      150 // standing still this long short of the goal = stalled

typedef struct
{
    GPIO_TypeDef* gpio;
    uint16_t pin;
    uint8_t alternate; // PWM timer alternate function, restored when the port leaves bus mode
} BusPin;

static const BusPin busPins[NUM_SERVOS] = {
    {S0_PWM_GPIO_Port, S0_PWM_Pin, GPIO_AF2_TIM3},
    {S1_PWM_GPIO_Port, S1_PWM_Pin, GPIO_AF2_TIM3},
    {S2_PWM_GPIO_Port, S2_PWM_Pin, GPIO_AF3_TIM9},
    {S3_PWM_GPIO_Port, S3_PWM_Pin, GPIO_AF3_TIM9},
};

typedef enum
{
    LINK_OFF = 0,
    LINK_SEARCHING,
    LINK_CONNECTED,
} BusLink;

#define BUS_UNKNOWN 0xFF

typedef struct
{
    BusLink link;
    uint8_t id;
    uint8_t failures;
    uint32_t nextPollMs;

    BusServoCommand target; // latest command from the Pi
    BusServoCommand applied; // last command fully written to the servo
    uint8_t applyPending;
    uint8_t goalWritten; // goal/speed/acc of target already written
    uint8_t stsMode; // operating mode on the servo, BUS_UNKNOWN until read
    uint8_t torque; // torque enable on the servo, BUS_UNKNOWN until read

    uint32_t commandMs;
    uint32_t stillSinceMs;
    uint8_t done;
    uint8_t stalled;
    uint8_t moving;

    int32_t position; // unwrapped
    uint16_t raw;
    uint8_t havePosition;
} BusPort;

static BusPort ports[NUM_SERVOS];
static uint32_t cyclesPerBit;
static uint32_t cyclesPerUs;
static uint32_t lastTransactionMs;
static uint8_t nextPort;

/* ------------------------------------------------------------------ */
/* Bit-banged UART                                                      */
/* ------------------------------------------------------------------ */

static inline void bus_drive(const BusPin* p)
{
    const uint32_t shift = 2u * (uint32_t)__builtin_ctz(p->pin);
    p->gpio->MODER = (p->gpio->MODER & ~(3u << shift)) | (1u << shift);
}

static inline void bus_release(const BusPin* p)
{
    p->gpio->MODER &= ~(3u << (2u * (uint32_t)__builtin_ctz(p->pin)));
}

static inline void bus_wait_until(const uint32_t t)
{
    while ((int32_t)(DWT->CYCCNT - t) < 0)
    {
    }
}

static void bus_tx_byte(const BusPin* p, const uint8_t b, uint32_t* t)
{
    uint16_t frame = (uint16_t)((b << 1) | 0x200); // start bit 0, 8 data bits LSB first, stop bit 1
    for (int i = 0; i < 10; i++)
    {
        bus_wait_until(*t);
        p->gpio->BSRR = (frame & 1) ? p->pin : ((uint32_t)p->pin << 16);
        frame >>= 1;
        *t += cyclesPerBit;
    }
}

// Returns 1 on a byte, 0 on timeout, -1 on framing error.
static int bus_rx_byte(const BusPin* p, uint8_t* out, const uint32_t timeoutCycles)
{
    const uint32_t start = DWT->CYCCNT;
    while (p->gpio->IDR & p->pin)
    {
        if (DWT->CYCCNT - start > timeoutCycles)
            return 0;
    }

    // Sample in the middle of each data bit.
    uint32_t t = DWT->CYCCNT + cyclesPerBit + cyclesPerBit / 2;
    uint8_t b = 0;
    for (int i = 0; i < 8; i++)
    {
        bus_wait_until(t);
        if (p->gpio->IDR & p->pin)
            b |= (uint8_t)(1u << i);
        t += cyclesPerBit;
    }

    bus_wait_until(t);
    if (!(p->gpio->IDR & p->pin))
        return -1;

    *out = b;
    return 1;
}

static int bus_transfer(const BusPin* p, const uint8_t* tx, const int txLen, uint8_t* rx, const int rxLen)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    const uint32_t startCycles = DWT->CYCCNT;

    uint32_t t = startCycles;
    p->gpio->BSRR = p->pin;
    bus_drive(p);
    t += cyclesPerBit; // one idle bit before the first start bit
    for (int i = 0; i < txLen; i++)
        bus_tx_byte(p, tx[i], &t);
    bus_wait_until(t); // finish the last stop bit
    bus_release(p);

    int n = 0;
    uint32_t timeout = BUS_FIRST_BYTE_TIMEOUT_US * cyclesPerUs;
    while (n < rxLen)
    {
        if (bus_rx_byte(p, &rx[n], timeout) <= 0)
            break;
        n++;
        timeout = BUS_INTER_BYTE_TIMEOUT_US * cyclesPerUs;
    }

    // The 1 MHz TIM6 tick could not run meanwhile (at most one update stayed
    // pending). Advance microSeconds by the blocked time instead, so BEMF,
    // PID and odometry dt stay right.
    const uint32_t blockedUs = (DWT->CYCCNT - startCycles) / cyclesPerUs;
    if (blockedUs > 0)
    {
        microSeconds += blockedUs;
        __HAL_TIM_CLEAR_FLAG(&htim6, TIM_FLAG_UPDATE);
        NVIC_ClearPendingIRQ(TIM6_DAC_IRQn);
    }

    __set_PRIMASK(primask);
    return n;
}

static int bus_build(uint8_t* buf, const uint8_t id, const uint8_t inst, const uint8_t* params, const uint8_t n)
{
    buf[0] = 0xFF;
    buf[1] = 0xFF;
    buf[2] = id;
    buf[3] = (uint8_t)(n + 2);
    buf[4] = inst;
    uint8_t sum = (uint8_t)(id + n + 2 + inst);
    for (int i = 0; i < n; i++)
    {
        buf[5 + i] = params[i];
        sum += params[i];
    }
    buf[5 + n] = (uint8_t)~sum;
    return n + 6;
}

// Sends one instruction and validates a status packet with nReply payload bytes.
// Returns the status error byte, or -1 (no/short reply), -2 (bad header), -3 (bad checksum).
static int bus_request(const int port, const uint8_t id, const uint8_t inst, const uint8_t* params, const uint8_t n,
                       uint8_t* replyParams, const uint8_t nReply, uint8_t* replyId)
{
    uint8_t tx[BUS_MAX_PACKET];
    uint8_t rx[BUS_MAX_PACKET];
    const int txLen = bus_build(tx, id, inst, params, n);
    const int expected = 6 + nReply;
    const int got = bus_transfer(&busPins[port], tx, txLen, rx, expected);

    if (got < expected)
        return -1;
    if (rx[0] != 0xFF || rx[1] != 0xFF || rx[3] != nReply + 2)
        return -2;

    uint8_t sum = 0;
    for (int i = 2; i < expected - 1; i++)
        sum += rx[i];
    if ((uint8_t)~sum != rx[expected - 1])
        return -3;

    if (replyId)
        *replyId = rx[2];
    if (nReply)
        memcpy(replyParams, &rx[5], nReply);
    return rx[4];
}

int bus_servo_ping(const int port, const uint8_t id, uint8_t* foundId)
{
    return bus_request(port, id, STS_INST_PING, NULL, 0, NULL, 0, foundId);
}

int bus_servo_read(const int port, const uint8_t id, const uint8_t addr, const uint8_t len, uint8_t* out)
{
    const uint8_t params[2] = {addr, len};
    return bus_request(port, id, STS_INST_READ, params, 2, out, len, NULL);
}

int bus_servo_write(const int port, const uint8_t id, const uint8_t addr, const uint8_t* data, const uint8_t len)
{
    uint8_t params[BUS_MAX_PACKET - 7];
    if (len + 1 > (int)sizeof(params))
        return -1;
    params[0] = addr;
    memcpy(&params[1], data, len);
    return bus_request(port, id, STS_INST_WRITE, params, (uint8_t)(len + 1), NULL, 0, NULL);
}

/* ------------------------------------------------------------------ */
/* Port state machine                                                   */
/* ------------------------------------------------------------------ */

static void bus_pin_to_bus(const int port)
{
    // Idle high; the TXS0104 pull-ups hold the line high while released.
    const BusPin* p = &busPins[port];
    GPIO_InitTypeDef init = {0};
    init.Pin = p->pin;
    init.Mode = GPIO_MODE_INPUT;
    init.Pull = GPIO_PULLUP;
    init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(p->gpio, &init);
    p->gpio->BSRR = p->pin;
    p->gpio->OTYPER &= ~p->pin;
}

static void bus_pin_to_pwm(const int port)
{
    const BusPin* p = &busPins[port];
    GPIO_InitTypeDef init = {0};
    init.Pin = p->pin;
    init.Mode = GPIO_MODE_AF_PP;
    init.Pull = GPIO_NOPULL;
    init.Speed = GPIO_SPEED_FREQ_LOW;
    init.Alternate = p->alternate;
    HAL_GPIO_Init(p->gpio, &init);
}

static int circular_distance(const uint16_t a, const uint16_t b)
{
    int d = ((int)a - (int)b) % BUS_SERVO_STEPS_PER_REV;
    if (d < 0)
        d += BUS_SERVO_STEPS_PER_REV;
    if (d > BUS_SERVO_STEPS_PER_REV / 2)
        d = BUS_SERVO_STEPS_PER_REV - d;
    return d;
}

static int16_t sign_magnitude_to_int(const uint16_t v, const uint16_t signBit)
{
    const int16_t magnitude = (int16_t)(v & (signBit - 1));
    return (v & signBit) ? (int16_t)-magnitude : magnitude;
}

static void bus_publish(const int port)
{
    const BusPort* s = &ports[port];
    volatile BusServoState* out = &txBuffer.busServo[port];

    uint8_t flags = 0;
    if (s->link == LINK_CONNECTED)
        flags |= BUS_SERVO_FLAG_CONNECTED;
    if (s->moving)
        flags |= BUS_SERVO_FLAG_MOVING;
    if (s->done)
        flags |= BUS_SERVO_FLAG_DONE;
    if (s->stalled)
        flags |= BUS_SERVO_FLAG_STALLED;
    if (s->torque == 1)
        flags |= BUS_SERVO_FLAG_TORQUE;

    out->position = s->position;
    out->rawPosition = s->raw;
    out->flags = flags;
    out->commandSeq = s->applied.seq;
}

static void bus_lost(const int port, const uint32_t now)
{
    BusPort* s = &ports[port];
    printf("[bus] port %d: servo id %u lost\r\n", port, s->id);
    s->link = LINK_SEARCHING;
    s->failures = 0;
    s->nextPollMs = now + BUS_SEARCH_INTERVAL_MS;
    s->stsMode = BUS_UNKNOWN;
    s->torque = BUS_UNKNOWN;
    s->moving = 0;
    s->applyPending = 1;
    s->goalWritten = 0;
    txBuffer.busServo[port].status = 0;
    bus_publish(port);
}

// Book-keeping after every transaction on a connected servo.
static uint8_t bus_result(const int port, const int err, const uint32_t now)
{
    BusPort* s = &ports[port];
    if (err < 0)
    {
        if (++s->failures >= BUS_MAX_FAILURES)
            bus_lost(port, now);
        return 0;
    }
    s->failures = 0;
    txBuffer.busServo[port].status = (uint8_t)err;
    return 1;
}

static void bus_search(const int port, const uint32_t now)
{
    BusPort* s = &ports[port];
    uint8_t found = 0;

    // One servo per port, so the broadcast ping finds it whatever its ID is.
    s->nextPollMs = now + BUS_SEARCH_INTERVAL_MS;
    if (bus_servo_ping(port, BUS_SERVO_BROADCAST_ID, &found) < 0)
        return;

    s->link = LINK_CONNECTED;
    s->id = found;
    s->failures = 0;
    s->stsMode = BUS_UNKNOWN;
    s->torque = BUS_UNKNOWN;
    s->applyPending = 1;
    s->goalWritten = 0;
    s->nextPollMs = now;
    printf("[bus] port %d: servo id %u connected\r\n", port, found);
    bus_publish(port);
}

static void bus_poll(const int port, const uint32_t now)
{
    BusPort* s = &ports[port];

    // Present position .. moving flag in one read: 56 position, 58 speed,
    // 60 load, 62 voltage, 63 temperature, 64 async flag, 65 status, 66 moving.
    uint8_t st[11];
    const int err = bus_servo_read(port, s->id, STS_REG_PRESENT_POSITION, sizeof(st), st);
    if (!bus_result(port, err, now))
        return;

    const uint16_t raw = (uint16_t)((st[0] | (st[1] << 8)) % BUS_SERVO_STEPS_PER_REV);
    if (!s->havePosition)
    {
        s->position = raw;
        s->havePosition = 1;
    }
    else
    {
        // Polled at >= 10 Hz; even full speed covers far less than half a turn per poll.
        int delta = (int)raw - (int)s->raw;
        if (delta > BUS_SERVO_STEPS_PER_REV / 2)
            delta -= BUS_SERVO_STEPS_PER_REV;
        else if (delta < -BUS_SERVO_STEPS_PER_REV / 2)
            delta += BUS_SERVO_STEPS_PER_REV;
        s->position += delta;
    }
    s->raw = raw;
    s->moving = st[10] ? 1 : 0;

    volatile BusServoState* out = &txBuffer.busServo[port];
    out->speed = sign_magnitude_to_int((uint16_t)(st[2] | (st[3] << 8)), 0x8000);
    out->load = sign_magnitude_to_int((uint16_t)(st[4] | (st[5] << 8)), 0x0400);
    out->voltage = st[6];
    out->temperature = st[7];
    out->status = st[9];

    if (s->moving)
        s->stillSinceMs = now;

    const uint8_t positionMove = !s->applyPending && s->applied.mode == BUS_SERVO_MODE_POSITION;
    if (positionMove && !s->done)
    {
        const int error = circular_distance(raw, s->applied.goalPosition);
        if (!s->moving && error <= BUS_DONE_TOLERANCE)
        {
            s->done = 1;
            s->stalled = 0;
        }
        else if (!s->moving && now - s->stillSinceMs >= BUS_STALL_TIME_MS && now - s->commandMs >= BUS_STALL_TIME_MS)
        {
            // Blocked or overloaded: report done so a "wait until done" never hangs.
            s->done = 1;
            s->stalled = 1;
        }
    }

    const uint8_t active = s->applied.mode == BUS_SERVO_MODE_WHEEL || (positionMove && !s->done) || s->moving;
    s->nextPollMs = now + (active ? BUS_POLL_ACTIVE_MS : BUS_POLL_IDLE_MS);
    bus_publish(port);
}

static void bus_write_byte(const int port, const uint8_t reg, const uint8_t value, uint8_t* shadow,
                           const uint32_t now)
{
    if (bus_result(port, bus_servo_write(port, ports[port].id, reg, &value, 1), now))
        *shadow = value;
}

// One transaction towards applying ports[port].target.
static void bus_apply_step(const int port, const uint32_t now)
{
    BusPort* s = &ports[port];
    const BusServoCommand* c = &s->target;

    if (s->stsMode == BUS_UNKNOWN || s->torque == BUS_UNKNOWN)
    {
        // Learn the servo's state first, so a reconnect does not drop a held load.
        uint8_t regs[8]; // 33 operating mode .. 40 torque enable
        if (bus_result(port, bus_servo_read(port, s->id, STS_REG_OPERATING_MODE, sizeof(regs), regs), now))
        {
            s->stsMode = regs[0];
            s->torque = regs[7] ? 1 : 0;
        }
        return;
    }

    if (c->mode == BUS_SERVO_MODE_RELAXED)
    {
        if (s->torque != 0)
        {
            bus_write_byte(port, STS_REG_TORQUE_ENABLE, 0, &s->torque, now);
            return;
        }
    }
    else
    {
        const uint8_t wantMode = c->mode == BUS_SERVO_MODE_WHEEL ? STS_OPERATING_MODE_WHEEL : STS_OPERATING_MODE_POSITION;
        if (s->stsMode != wantMode)
        {
            // Switch operating mode with torque off.
            if (s->torque != 0)
                bus_write_byte(port, STS_REG_TORQUE_ENABLE, 0, &s->torque, now);
            else
                bus_write_byte(port, STS_REG_OPERATING_MODE, wantMode, &s->stsMode, now);
            return;
        }

        if (!s->goalWritten)
        {
            // 41 acceleration, 42 goal position, 44 goal time (0 = use speed), 46 goal speed.
            // Written before torque on, so enabling torque never jumps to a stale goal.
            const uint16_t goal = c->mode == BUS_SERVO_MODE_WHEEL
                                      ? s->raw
                                      : (uint16_t)(c->goalPosition % BUS_SERVO_STEPS_PER_REV);
            const int16_t speed = c->speed;
            uint16_t speedReg = (uint16_t)(speed < 0 ? -speed : speed) & 0x7FFF;
            if (c->mode == BUS_SERVO_MODE_WHEEL && speed < 0)
                speedReg |= 0x8000;

            const uint8_t data[7] = {
                c->acc,
                (uint8_t)goal, (uint8_t)(goal >> 8),
                0, 0,
                (uint8_t)speedReg, (uint8_t)(speedReg >> 8),
            };
            if (bus_result(port, bus_servo_write(port, s->id, STS_REG_ACCELERATION, data, sizeof(data)), now))
                s->goalWritten = 1;
            return;
        }

        if (s->torque != 1)
        {
            bus_write_byte(port, STS_REG_TORQUE_ENABLE, 1, &s->torque, now);
            return;
        }
    }

    // Fully applied.
    s->applied = *c;
    s->applyPending = 0;
    s->commandMs = now;
    s->stillSinceMs = now;
    s->done = c->mode != BUS_SERVO_MODE_POSITION;
    s->stalled = 0;
    s->nextPollMs = now + BUS_POLL_ACTIVE_MS;
    bus_publish(port);
}

// The Pi's DMA can rewrite rxBuffer while we copy it; read until two copies match.
static void bus_read_command(const int port, BusServoCommand* out)
{
    BusServoCommand a;
    BusServoCommand b;
    do
    {
        memcpy(&a, (const void*)&rxBuffer.busServo[port], sizeof(a));
        memcpy(&b, (const void*)&rxBuffer.busServo[port], sizeof(b));
    }
    while (memcmp(&a, &b, sizeof(a)) != 0);
    *out = a;
}

void bus_servo_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    cyclesPerBit = SystemCoreClock / BUS_BAUD;
    cyclesPerUs = SystemCoreClock / 1000000u;
    memset(ports, 0, sizeof(ports));
}

void bus_servo_set_port_enabled(const int port, const uint8_t enabled)
{
    if (port < 0 || port >= NUM_SERVOS)
        return;
    BusPort* s = &ports[port];

    if (enabled && s->link == LINK_OFF)
    {
        bus_pin_to_bus(port);
        s->link = LINK_SEARCHING;
        s->failures = 0;
        s->nextPollMs = 0;
        s->applyPending = 1;
        s->goalWritten = 0;
        s->done = 0;
        s->stalled = 0;
        s->moving = 0;
        s->havePosition = 0;
        printf("[bus] port %d: bus mode on, searching\r\n", port);
        bus_publish(port);
    }
    else if (!enabled && s->link != LINK_OFF)
    {
        if (s->link == LINK_CONNECTED && s->torque != 0)
        {
            const uint8_t off = 0;
            bus_servo_write(port, s->id, STS_REG_TORQUE_ENABLE, &off, 1);
        }
        bus_pin_to_pwm(port);
        s->link = LINK_OFF;
        memset((void*)&txBuffer.busServo[port], 0, sizeof(BusServoState));
        printf("[bus] port %d: bus mode off\r\n", port);
    }
}

void bus_servo_update(const uint32_t nowMs)
{
    for (int port = 0; port < NUM_SERVOS; port++)
    {
        BusPort* s = &ports[port];
        if (s->link == LINK_OFF)
            continue;

        BusServoCommand c;
        bus_read_command(port, &c);
        if (memcmp(&c, &s->target, sizeof(c)) != 0)
        {
            s->target = c;
            s->applyPending = 1;
            s->goalWritten = 0;
        }
    }

    if (nowMs - lastTransactionMs < BUS_MIN_GAP_MS)
        return;

    for (int i = 0; i < NUM_SERVOS; i++)
    {
        const int port = (nextPort + i) % NUM_SERVOS;
        BusPort* s = &ports[port];
        const uint8_t due = (int32_t)(nowMs - s->nextPollMs) >= 0;

        if (s->link == LINK_CONNECTED && s->applyPending)
            bus_apply_step(port, nowMs);
        else if (s->link == LINK_CONNECTED && due)
            bus_poll(port, nowMs);
        else if (s->link == LINK_SEARCHING && due)
            bus_search(port, nowMs);
        else
            continue;

        lastTransactionMs = nowMs;
        nextPort = (uint8_t)((port + 1) % NUM_SERVOS);
        return;
    }
}

//
// Created by tobias on 6/9/25.
//

#pragma once
#include <chrono>
#include <cmath>
#include <cstring>
#include <linux/spi/spidev.h>

#include "pi_buffer.h"
#include <stdexcept>

#define SPI_DEVICE "/dev/spidev0.0"
#define SERVO_MINIMUM_DUTYCYCLE 300u

#define SHUTDOWN_SERVO_FLAG 0
#define SHUTDOWN_MOTOR_FLAG 1

namespace platform::wombat::core
{
    /** Length of the full-duplex transfer (pi_buffer.h decides). */
    constexpr std::size_t kBufSize = BUFFER_LENGTH_DUPELX_COMMUNICATION;

    /** RAII wrapper for the Raspberry Pi’s spidev interface.  */
    class Spi
    {
    public:
        /** Grab the singleton – cheap & thread-safe (C++11 static-init). */
        static Spi& instance()
        {
            static Spi impl;
            return impl;
        }

        /** Initialise (idempotent).  */
        bool init(uint32_t speed_hz = 20'000'000);

        /** Perform one full-duplex transfer.  
         *  Returns `false` if the STM32 reports a version mismatch. */
        bool update();

        /** Perform one full-duplex transfer.
         *  Returns `false` if the STM32 reports a version mismatch.
         *  should be used at the end of a task/program to make sure the last chagnes to the spi buffer were sent
         */
        bool forceUpdate();

        /** Raw buffer access for the helpers below. */
        [[nodiscard]] uint8_t* tx() { return m_tx; }
        [[nodiscard]] const uint8_t* rx() const { return m_rx; }

        /* non-copyable / non-movable */
        Spi(const Spi&) = delete;
        Spi& operator=(const Spi&) = delete;


    private:
        Spi() = default;
        ~Spi();

        int m_fd{-1};
        spi_ioc_transfer m_tr{};
        alignas(4) uint8_t m_tx[kBufSize]{};
        alignas(4) uint8_t m_rx[kBufSize]{};
    };

    /* --------------------------------- typed helpers -------------------------------- */

    /*  Convenience inline helpers that map straight to the offsets in pi_buffer.h
     *  – kept **header-only** to enable the optimiser to inline everything away.       */

    enum class MotorDir : uint8_t { Off = 0b00, CCW = 0b01, CW = 0b10, ServoLike = 0b11 };

    enum class ServoMode : uint8_t { FullyDisabled = 0, Disabled = 1, Enabled = 2 };

    /* -------- setters (TX) -------- */
    inline void setShutdownFlag(const uint8_t bit, const bool value)
    {
        auto* tx = Spi::instance().tx();
        value
            ? tx[TX_SYSTEM_SHUTDOWN] |= (1u << bit)
            : tx[TX_SYSTEM_SHUTDOWN] &= ~(1u << bit);

        if (!Spi::instance().update())
        {
           throw std::invalid_argument("stm sucks");
        }
    }

    inline void setMotor(const uint8_t port, MotorDir dir, const uint32_t value)
    {
        if (port > 3) return;
        auto* tx = Spi::instance().tx();
        tx[TX_MOT_MODE] = (tx[TX_MOT_MODE] & ~(0b11u << (port * 2))) | (static_cast<uint8_t>(dir) << (port * 2));
        std::memcpy(&tx[TX_SPEED_POS_MOT_0 + port * 4], &value, sizeof value);
    }

    inline void setServoMode(const uint8_t port, ServoMode mode)
    {
        if (port > 3) return;
        auto* tx = Spi::instance().tx();

        const uint8_t bitPos = port * 2;
        // Clear the 2 bits corresponding to the servo
        tx[TX_SERVO_MODE] &= ~(0b11 << bitPos);
        // Set the new motor mode (only the lower 2 bits are used)
        tx[TX_SERVO_MODE] |= ((static_cast<uint8_t>(mode) & 0b11) << bitPos);
    }

    inline uint16_t getServoPos(const uint8_t port)
    {
        if (port > 3) return 0;

        uint16_t pos;
        std::memcpy(&pos, &(Spi::instance().tx()[TX_POS_SERVO_0 + port * 2]), sizeof pos);

        // not my bullshit
        const double degrees = (static_cast<double>(pos) - 1500.0) / 10.0; // [-90, 90]
        double dval = (degrees + 90.0)  * 2047.0 / 180.0; // [0, 2047]

        if (dval < 0.0) dval = 0.0;
        if (dval > 2047.0) dval = 2047.01;
        const auto val = static_cast<unsigned short>(dval + 0.5);

        return val;
    }

    inline void setServoPos(uint8_t port, uint16_t pos /*raw PWM-ticks*/)
    {
        if (port > 3) return;
        auto* tx = Spi::instance().tx();

        // not my bullshit
        unsigned short val =  1500 + std::round(1800.0 * ((double)pos / 2047.0)) - (1800 / 2);
        const uint16_t corrected = val;
        std::memcpy(&tx[TX_POS_SERVO_0 + port * 2], &corrected, sizeof corrected);
    }

    /* -------- getters (RX) -------- */
    inline uint32_t lastUpdateUs()
    {
        const auto* rx = Spi::instance().rx();
        uint32_t v;
        std::memcpy(&v, &rx[RX_UPDATE_TIME], sizeof v);
        return v;
    }

    inline float gyroX()
    {
        float f;
        std::memcpy(&f, &Spi::instance().rx()[RX_GYRO_X], sizeof f);
        return f;
    }

    inline float gyroY()
    {
        float f;
        std::memcpy(&f, &Spi::instance().rx()[RX_GYRO_Y], sizeof f);
        return f;
    }

    inline float gyroZ()
    {
        float f;
        std::memcpy(&f, &Spi::instance().rx()[RX_GYRO_Z], sizeof f);
        return f;
    }

    /* -------- accel (RX) -------- */
    inline float accelX()
    {
        float f;
        std::memcpy(&f, &Spi::instance().rx()[RX_ACCEL_X], sizeof f);
        return f;
    }

    inline float accelY()
    {
        float f;
        std::memcpy(&f, &Spi::instance().rx()[RX_ACCEL_Y], sizeof f);
        return f;
    }

    inline float accelZ()
    {
        float f;
        std::memcpy(&f, &Spi::instance().rx()[RX_ACCEL_Z], sizeof f);
        return f;
    }

    /* -------- mag (RX) -------- */
    inline float magX()
    {
        float f;
        std::memcpy(&f, &Spi::instance().rx()[RX_MAG_X], sizeof f);
        return f;
    }

    inline float magY()
    {
        float f;
        std::memcpy(&f, &Spi::instance().rx()[RX_MAG_Y], sizeof f);
        return f;
    }

    inline float magZ()
    {
        float f;
        std::memcpy(&f, &Spi::instance().rx()[RX_MAG_Z], sizeof f);
        return f;
    }

    /* -------- BEMF (RX) -------- */
    inline int32_t bemf(uint8_t mot /*0-3*/)
    {
        auto* rx = Spi::instance().rx();

        if (mot > 3) return 0;
        int32_t value;
        std::memcpy(&value, &rx[RX_BEMF_READING_MOT0 + mot * 4], sizeof value);
        return value / 250;
    }

    /* -------- analog sensors (RX) -------- */
    inline uint16_t analog(uint8_t idx /*0-5*/)
    {
        if (idx > 5) return 0;
        uint16_t v;
        std::memcpy(&v, &Spi::instance().rx()[RX_ANALOG_SENSOR_0 + idx * 2], sizeof v);
        return v;
    }

    /* -------- digital inputs (RX) -------- */
    inline uint16_t digitalRaw()
    {
        uint16_t v;
        std::memcpy(&v, &Spi::instance().rx()[RX_DIGITAL_VALUES], sizeof v);
        return v;
    }

    inline bool digital(uint8_t bit /*0-15*/)
    {
        return (bit < 16) && (digitalRaw() & (1u << bit));
    }
}

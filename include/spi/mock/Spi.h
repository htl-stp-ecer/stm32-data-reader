#pragma once
#include <cstdint>
#include <array>
#include <random>
#include <cmath>

namespace platform::wombat::core
{
    constexpr std::size_t kBufSize = 256;

    class Spi
    {
    public:
        static Spi& instance()
        {
            static Spi impl;
            return impl;
        }

        bool init(uint32_t speed_hz = 20'000'000);

        int random0to2()
        {
            static std::mt19937 rng{std::random_device{}()};
            static std::uniform_int_distribution<int> dist(0, 2);
            return dist(rng);
        }

        bool update();

        bool forceUpdate();

        ~Spi() = default;

        uint8_t* tx() { return m_tx.data(); }
        const uint8_t* rx() const { return m_rx.data(); }

    private:
        Spi() = default;
        int m_fd{-1};
        std::array<uint8_t, kBufSize> m_tx{};
        std::array<uint8_t, kBufSize> m_rx{};
    };

    // === Random helpers ===
    inline int random0to2()
    {
        static std::mt19937 rng{std::random_device{}()};
        static std::uniform_int_distribution<int> dist(0, 2);
        return dist(rng);
    }

    inline uint32_t lastUpdateUs()
    {
        static uint32_t counter = 0;
        return counter++;
    }


    enum class MotorDir : uint8_t { Off = 0b00, CCW = 0b01, CW = 0b10, ServoLike = 0b11 };

    enum class ServoMode : uint8_t { FullyDisabled = 0, Disabled = 1, Enabled = 2 };

    inline float gyroX() { return static_cast<float>(random0to2()); }
    inline float gyroY() { return static_cast<float>(random0to2()); }
    inline float gyroZ() { return static_cast<float>(random0to2()); }

    inline float accelX() { return static_cast<float>(random0to2()); }
    inline float accelY() { return static_cast<float>(random0to2()); }
    inline float accelZ() { return static_cast<float>(random0to2()); }

    inline float magX() { return static_cast<float>(random0to2()); }
    inline float magY() { return static_cast<float>(random0to2()); }
    inline float magZ() { return static_cast<float>(random0to2()); }

    inline int32_t bemf(uint8_t) { return random0to2(); }
    inline uint16_t analog(uint8_t) { return static_cast<uint16_t>(random0to2()); }
    inline uint16_t digitalRaw() { return random0to2(); }
    inline bool digital(uint8_t) { return random0to2() > 0; }

    inline void setShutdownFlag(const uint8_t, bool) {}
    inline void setMotor(uint8_t, MotorDir, uint32_t) {}
    inline void setServoMode(uint8_t, ServoMode) {}
    inline void setServoPos(uint8_t, uint16_t) {}
    inline uint16_t getServoPos(uint8_t) { return static_cast<uint16_t>(random0to2()); }
}

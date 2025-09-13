#ifndef STM32_DATA_READER_LCMDATAWRITER_H
#define STM32_DATA_READER_LCMDATAWRITER_H

#include "exlcm/scalar_f_t.hpp"
#include "exlcm/scalar_i32_t.hpp"
#include "exlcm/vector3f_t.hpp"
#include "lcm/lcm-cpp.hpp"

#include <unordered_map>
#include <vector>
#include <string>
#include <cstring>

namespace platform::wombat::core
{
    enum class ServoMode : uint8_t;
    enum class MotorDir : uint8_t;

    class LcmDataWriter
    {
    public:
        static LcmDataWriter& instance()
        {
            static LcmDataWriter writer;
            return writer;
        }

        void setMotor(uint8_t port, MotorDir dir, exlcm::scalar_i32_t valueData);
        void setServoMode(uint8_t port, ServoMode mode);
        void setServoPos(uint8_t port, exlcm::scalar_i32_t posData);
        void setGyro(exlcm::vector3f_t gyroData);
        void setAccel(exlcm::vector3f_t accelData);
        void setMag(exlcm::vector3f_t magData);
        void setBemf(uint8_t mot, exlcm::scalar_i32_t valueData);
        void setAnalog(uint16_t idx, exlcm::scalar_i32_t valueData);
        void setDigital(uint16_t idx, exlcm::scalar_i32_t valueData);
        void setTemp(exlcm::scalar_f_t value);

    private:
        LcmDataWriter() = default;
        ~LcmDataWriter();

        lcm::LCM lcm;

        const int MIN_MOTOR_PORT = 0;
        const int MAX_MOTOR_PORT = 3;

        const int MIN_SERVO_PORT = 0;
        const int MAX_SERVO_PORT = 3;

        const int MIN_ANALOG = 0;
        const int MAX_ANALOG = 3;

        const int MIN_DIGITAL = 0;
        const int MAX_DIGITAL = 3;

        std::unordered_map<std::string, std::vector<uint8_t>> lastMsgs;

        template <typename Msg>
        bool publishIfChanged(const std::string& channel, const Msg& msg)
        {
            const auto* bytes = reinterpret_cast<const uint8_t*>(&msg);
            const size_t size = sizeof(Msg);

            auto& cached = lastMsgs[channel];
            if (cached.size() == size && std::memcmp(cached.data(), bytes, size) == 0)
            {
                return false;
            }

            cached.assign(bytes, bytes + size);
            lcm.publish(channel, &msg);
            return true;
        }
    };
}

#endif // STM32_DATA_READER_LCMDATAWRITER_H

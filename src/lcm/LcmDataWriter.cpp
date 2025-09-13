#include "lcm/LcmDataWriter.h"

#include "exlcm/scalar_i32_t.hpp"
#include "exlcm/vector3f_t.hpp"
#include "exlcm/scalar_i8_t.hpp"
#include "spdlog/spdlog.h"

using namespace platform::wombat::core;

void LcmDataWriter::setMotor(uint8_t port, MotorDir dir, exlcm::scalar_i32_t valueData)
{
    if (!lcm.good()) { spdlog::warn("[LCM-WRITER] Set Motor in LCM didn't work"); return; }
    if (MIN_MOTOR_PORT > port || port > MAX_MOTOR_PORT) return;

    constexpr std::string_view motorChannelBase = "libstp/motor/{}/";

    exlcm::scalar_i8_t dirData{};
    dirData.dir = static_cast<uint8_t>(dir);

    publishIfChanged(std::format("{}direction", std::format(motorChannelBase, static_cast<int>(port))), dirData);
    publishIfChanged(std::format("{}value", std::format(motorChannelBase, static_cast<int>(port))), valueData);
}

void LcmDataWriter::setServoMode(uint8_t port, ServoMode mode)
{
    if (!lcm.good()) { spdlog::warn("[LCM-WRITER] Set Servo Mode in LCM didn't work"); return; }
    if (MIN_SERVO_PORT > port || MAX_SERVO_PORT < port) return;

    constexpr static std::string_view servoChannelBase = "libstp/servo/{}/";

    exlcm::scalar_i8_t modeData{};
    modeData.dir = static_cast<uint8_t>(mode);
    publishIfChanged(std::format("{}mode", std::format(servoChannelBase, static_cast<int>(port))), modeData);
}

void LcmDataWriter::setServoPos(uint8_t port, exlcm::scalar_i32_t posData)
{
    if (!lcm.good()) { spdlog::warn("[LCM-WRITER] Set Servo Position in LCM didn't work"); return; }
    if (MIN_SERVO_PORT > port || MAX_SERVO_PORT < port) return;

    constexpr static std::string_view servoChannelBase = "libstp/servo/{}/";
    publishIfChanged(std::format("{}position", std::format(servoChannelBase, static_cast<int>(port))), posData);
}

void LcmDataWriter::setGyro(exlcm::vector3f_t gyroData)
{
    if (!lcm.good()) { spdlog::warn("[LCM-WRITER] Set Gyro in LCM didn't work"); return; }
    publishIfChanged("libstp/gyro/value", gyroData);
}

void LcmDataWriter::setAccel(exlcm::vector3f_t accelData)
{
    if (!lcm.good()) { spdlog::warn("[LCM-WRITER] Set Accel in LCM didn't work"); return; }
    publishIfChanged("libstp/accel/value", accelData);
}

void LcmDataWriter::setMag(exlcm::vector3f_t magData)
{
    if (!lcm.good()) { spdlog::warn("[LCM-WRITER] Set Mag in LCM didn't work"); return; }
    publishIfChanged("libstp/mag/value", magData);
}

void LcmDataWriter::setBemf(uint8_t mot, exlcm::scalar_i32_t valueData)
{
    if (!lcm.good()) { spdlog::warn("[LCM-WRITER] Set Bemf in LCM didn't work"); return; }
    publishIfChanged(std::format("libstp/bemf/{}/value", static_cast<int>(mot)), valueData);
}

void LcmDataWriter::setAnalog(uint16_t idx, exlcm::scalar_i32_t valueData)
{
    if (!lcm.good()) { spdlog::warn("[LCM-WRITER] Set Analog in LCM didn't work"); return; }
    if (idx < MIN_ANALOG || MAX_ANALOG < idx) return;

    publishIfChanged(std::format("libstp/analog/{}/value", std::to_string(idx)), valueData);
}

void LcmDataWriter::setDigital(uint16_t idx, exlcm::scalar_i32_t valueData)
{
    if (!lcm.good()) { spdlog::warn("[LCM-WRITER] Set Digital in LCM didn't work"); return; }
    if (idx < MIN_DIGITAL || MAX_DIGITAL < idx) return;

    publishIfChanged(std::format("libstp/digital/{}/value", std::to_string(idx)), valueData);
}

void LcmDataWriter::setTemp(exlcm::scalar_f_t value)
{
    if (!lcm.good()) { spdlog::warn("[LCM-WRITER] Set Temp in LCM didn't work"); return; }
    publishIfChanged("libstp/temp/value", value);
}

LcmDataWriter::~LcmDataWriter() = default;

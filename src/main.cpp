#include <thread>
#include <chrono>
#include <array>
#include <atomic>
#include <string>
#include <lcm/lcm-cpp.hpp>
#include <spdlog/spdlog.h>
#include <memory>

#include "spi/pi_buffer.h"

#ifdef USE_SPI_MOCK
#include "spi/mock/Spi.h"
#else
#include "spi/Spi.h"
#endif

#include "exlcm/vector3f_t.hpp"
#include "exlcm/scalar_f_t.hpp"
#include "exlcm/scalar_i32_t.hpp"
#include "lcm/LcmDataWriter.h"

using namespace platform::wombat::core;
using clk = std::chrono::steady_clock;

static std::array<std::atomic<int32_t>, 4> motorPowerCmd{};
static std::array<std::atomic<uint16_t>, 4> servoPosCmd{};

static std::string motChan(const char* what) { return std::string("motors_0_") + what; }
static std::string srvChan(const char* what) { return std::string("servos_0_") + what; }

int main()
{
    spdlog::set_pattern("[%H:%M:%S.%e] [%l] %v");
    spdlog::set_level(spdlog::level::info);
    spdlog::info("Starting Wombat‑Pi LCM interface (flat topics)…");

    lcm::LCM lc;
    if (!lc.good())
    {
        spdlog::error("LCM init failed");
        return 1;
    }

    Spi& spi = Spi::instance();
    spi.init();

    lc.subscribe<exlcm::scalar_i32_t>(motChan("power_cmd"),
        [&](const lcm::ReceiveBuffer*, const std::string&, const exlcm::scalar_i32_t* cmd)
        {
            motorPowerCmd[0] = cmd->value;
            setMotor(0, cmd->value >= 0 ? MotorDir::CW : MotorDir::CCW, std::abs(cmd->value));
            spi.forceUpdate();
#ifdef USE_SPI_MOCK
            spdlog::info("[MOCK] Received motor power_cmd: {}", cmd->value);
#endif
        });

    lc.subscribe<exlcm::scalar_i32_t>(srvChan("position_cmd"),
        [&](const lcm::ReceiveBuffer*, const std::string&, const exlcm::scalar_i32_t* cmd)
        {
            servoPosCmd[0] = static_cast<uint16_t>(cmd->value);
            setServoPos(0, static_cast<uint16_t>(cmd->value));
            spi.forceUpdate();
#ifdef USE_SPI_MOCK
            spdlog::info("[MOCK] Received servo position_cmd: {}", cmd->value);
#endif
        });

#ifdef USE_SPI_MOCK
    lc.subscribe<exlcm::vector3f_t>("libstp/gyro/value",
        [](const lcm::ReceiveBuffer*, const std::string&, const exlcm::vector3f_t* msg)
        {
            spdlog::info("[MOCK] Gyro: x={} y={} z={}", msg->x, msg->y, msg->z);
        });

    lc.subscribe<exlcm::vector3f_t>("libstp/accel/value",
        [](const lcm::ReceiveBuffer*, const std::string&, const exlcm::vector3f_t* msg)
        {
            spdlog::info("[MOCK] Accel: x={} y={} z={}", msg->x, msg->y, msg->z);
        });

    lc.subscribe<exlcm::vector3f_t>("libstp/mag/value",
        [](const lcm::ReceiveBuffer*, const std::string&, const exlcm::vector3f_t* msg)
        {
            spdlog::info("[MOCK] Mag: x={} y={} z={}", msg->x, msg->y, msg->z);
        });
#endif

    uint32_t lastTs = 0;

    while (true)
    {
        spi.update();
        lc.handleTimeout(0);

        const uint32_t ts = lastUpdateUs();
        if (ts == lastTs)
            continue;
        lastTs = ts;

        exlcm::vector3f_t v3{};
        v3.x = gyroX();
        v3.y = gyroY();
        v3.z = gyroZ();
        LcmDataWriter::instance().setGyro(v3);

        v3.x = accelX();
        v3.y = accelY();
        v3.z = accelZ();
        LcmDataWriter::instance().setAccel(v3);

        v3.x = magX();
        v3.y = magY();
        v3.z = magZ();
        LcmDataWriter::instance().setMag(v3);

        exlcm::scalar_f_t fmsg{};
        std::memcpy(&fmsg.value, &spi.rx()[RX_IMU_TEMPERATUR], sizeof(float));
        LcmDataWriter::instance().setTemp(fmsg);

        for (uint8_t i = 0; i < 6; ++i)
        {
            exlcm::scalar_i32_t msg{};
            msg.value = analog(i);
            LcmDataWriter::instance().setAnalog(i, msg);
        }

        const uint16_t dmask = digitalRaw();
        for (uint8_t bit = 0; bit < 11; ++bit)
        {
            exlcm::scalar_i32_t msg{};
            msg.value = dmask >> bit & 1u;
            LcmDataWriter::instance().setDigital(bit, msg);
        }

        exlcm::scalar_i32_t mmsg{};
        mmsg.value = motorPowerCmd[0].load();
        LcmDataWriter::instance().setMotor(0, MotorDir::CW, mmsg);

        mmsg.value = bemf(0);
        LcmDataWriter::instance().setBemf(0, mmsg);

        exlcm::scalar_i32_t smsg{};
        smsg.value = getServoPos(0);
        LcmDataWriter::instance().setServoPos(0, smsg);

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

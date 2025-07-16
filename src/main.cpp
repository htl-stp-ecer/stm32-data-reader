#include <thread>
#include <chrono>
#include <array>
#include <atomic>
#include <string>
#include <lcm/lcm-cpp.hpp>
#include <spdlog/spdlog.h>
#include <memory>

#include "spi/Spi.h"
#include "pub_if_changed.hpp"

#include "exlcm/vector3f_t.hpp"
#include "exlcm/scalar_f_t.hpp"
#include "exlcm/scalar_i32_t.hpp"

using namespace platform::wombat::core;
using clk = std::chrono::steady_clock;

static std::array<std::atomic<int32_t>, 4> motorPowerCmd{};
static std::array<std::atomic<uint16_t>, 4> servoPosCmd{};

static std::string adcChan(uint8_t idx) { return "sensors_analog_" + std::to_string(idx); }
static std::string dioChan(uint8_t bit) { return "sensors_digital_" + std::to_string(bit); }
static std::string motChan(const char* what) { return std::string("motors_0_") + what; }
static std::string srvChan(const char* what) { return std::string("servos_0_") + what; }

template <size_t N>
static std::array<PubIfChanged<exlcm::scalar_i32_t>, N> make_digital_pubs(lcm::LCM& lcm)
{
    std::array<PubIfChanged<exlcm::scalar_i32_t>, N> pubs;
    for (size_t i = 0; i < N; ++i)
    {
        pubs[i] = PubIfChanged<exlcm::scalar_i32_t>(lcm, dioChan(i));
    }
    return pubs;
}

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
                                      [&](const lcm::ReceiveBuffer*, const std::string&,
                                          const exlcm::scalar_i32_t* cmd)
                                      {
                                          motorPowerCmd[0] = cmd->value;
                                          setMotor(0,
                                                   cmd->value >= 0 ? MotorDir::CW : MotorDir::CCW,
                                                   std::abs(cmd->value));
                                          spi.forceUpdate();
                                      });

    lc.subscribe<exlcm::scalar_i32_t>(srvChan("position_cmd"),
                                      [&](const lcm::ReceiveBuffer*, const std::string&,
                                          const exlcm::scalar_i32_t* cmd)
                                      {
                                          servoPosCmd[0] = static_cast<uint16_t>(cmd->value);
                                          setServoPos(0, static_cast<uint16_t>(cmd->value));
                                          spi.forceUpdate();
                                      });

    PubIfChanged<exlcm::vector3f_t> pubGyro(lc, "sensors_imu_gyro");
    PubIfChanged<exlcm::vector3f_t> pubAccel(lc, "sensors_imu_accel");
    PubIfChanged<exlcm::vector3f_t> pubMag(lc, "sensors_imu_magneto");
    PubIfChanged<exlcm::scalar_f_t> pubTemp(lc, "sensors_imu_temp");

    std::array<PubIfChanged<exlcm::scalar_i32_t>, 6> pubAnalog = {
        {
            {lc, adcChan(0)}, {lc, adcChan(1)}, {lc, adcChan(2)},
            {lc, adcChan(3)}, {lc, adcChan(4)}, {lc, adcChan(5)}
        }
    };
    auto pubDigital = make_digital_pubs<11>(lc);

    PubIfChanged<exlcm::scalar_i32_t> pubMotorPower(lc, motChan("power"));
    PubIfChanged<exlcm::scalar_i32_t> pubMotorBemf(lc, motChan("bemf"));
    PubIfChanged<exlcm::scalar_i32_t> pubServoPos(lc, srvChan("position"));

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
        v3.timestamp = ts;

        v3.x = gyroX();
        v3.y = gyroY();
        v3.z = gyroZ();
        pubGyro(v3);
        v3.x = accelX();
        v3.y = accelY();
        v3.z = accelZ();
        pubAccel(v3);
        v3.x = magX();
        v3.y = magY();
        v3.z = magZ();
        pubMag(v3);

        exlcm::scalar_f_t fmsg{};
        fmsg.timestamp = ts;
        std::memcpy(&fmsg.value, &spi.rx()[RX_IMU_TEMPERATUR], sizeof(float));
        pubTemp(fmsg);

        for (uint8_t i = 0; i < 6; ++i)
        {
            exlcm::scalar_i32_t msg{};
            msg.timestamp = ts;
            msg.value = analog(i);
            pubAnalog[i](msg);
        }

        const uint16_t dmask = digitalRaw();
        for (uint8_t bit = 0; bit < 11; ++bit)
        {
            exlcm::scalar_i32_t msg{};
            msg.timestamp = ts;
            msg.value = dmask >> bit & 1u;
            pubDigital[bit](msg);
        }

        exlcm::scalar_i32_t mmsg{};
        mmsg.timestamp = ts;

        mmsg.value = motorPowerCmd[0].load();
        pubMotorPower(mmsg);

        mmsg.value = bemf(0);
        pubMotorBemf(mmsg);

        exlcm::scalar_i32_t smsg{};
        smsg.timestamp = ts;
        smsg.value = getServoPos(0);
        pubServoPos(smsg);

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

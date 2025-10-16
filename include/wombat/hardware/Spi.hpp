#pragma once
#include <memory>
#include <array>
#include "wombat/core/Types.h"
#include "wombat/core/Result.h"
#include "wombat/core/Logger.h"

// C API
extern "C" {
#include "wombat/hardware/Spi.h"
}

namespace wombat
{
    class Spi
    {
    public:
        explicit Spi(const Configuration::Spi& cfg, std::shared_ptr<Logger> logger,
                     std::shared_ptr<LcmBroker> /*unused*/)
            : cfg_(cfg), logger_(std::move(logger))
        {
        }

        Result<void> initialize()
        {
            if (!spi_init(cfg_.speedHz))
            {
                return Result<void>::failure("spi_init() failed");
            }
            set_spi_mode(true);
            return Result<void>::success();
        }

        Result<void> shutdown()
        {
            set_spi_mode(false);
            spi_close();
            return Result<void>::success();
        }

        Result<void> forceUpdate()
        {
            if (!spi_force_update()) return Result<void>::failure("spi_force_update() failed");
            return Result<void>::success();
        }

        Result<SensorData> readSensorData()
        {
            SensorData d{};
            if (!spi_update()) return Result<SensorData>::failure("spi_update() failed");

            d.gyro.x = gyroX();
            d.gyro.y = gyroY();
            d.gyro.z = gyroZ();
            d.accelerometer.x = accelX();
            d.accelerometer.y = accelY();
            d.accelerometer.z = accelZ();
            d.magnetometer.x = magX();
            d.magnetometer.y = magY();
            d.magnetometer.z = magZ();
            d.orientation.w = quatW();
            d.orientation.x = quatX();
            d.orientation.y = quatY();
            d.orientation.z = quatZ();
            d.temperature = imuTemperature();
            d.batteryVoltage = battery_voltage();
            for (uint8_t i = 0; i < 6; ++i) d.analogValues[i] = analog_in(i);
            d.digitalBits = digital_raw();
            d.lastUpdate = last_update_us();

            for (uint8_t i = 0; i < 4; i++) motors_[i].backEmf = bemf(i);
            return Result<SensorData>::success(d);
        }

        Result<void> setMotorState(PortId port, const MotorState& st)
        {
            if (port >= MAX_MOTOR_PORTS) return Result<void>::failure("motor port out of range");
            MotorDir dir = MOTOR_DIR_OFF;
            switch (st.direction)
            {
            case MotorDirection::Off: dir = MOTOR_DIR_OFF;
                break;
            case MotorDirection::CounterClockwise: dir = MOTOR_DIR_CCW;
                break;
            case MotorDirection::Clockwise: dir = MOTOR_DIR_CW;
                break;
            case MotorDirection::ServoLike: dir = MOTOR_DIR_SERVO_LIKE;
                break;

            }

            const double percentAbs = std::min(st.speed / 100.0, 1.0);

            uint32_t duty = 0;
            if (percentAbs > 0.01f) {
                constexpr double minDutyFrac = 0.1f;
                const double dutyFrac = minDutyFrac + (1.0f - minDutyFrac) * percentAbs;
                duty = static_cast<uint32_t>(dutyFrac * 400.0f);
            }

            set_motor(port, dir, duty);
            st.backEmf = motors_[port].backEmf;
            motors_[port] = st;
            return Result<void>::success();
        }

        Result<MotorState> getMotorState(PortId port) const
        {
            if (port >= MAX_MOTOR_PORTS) return Result<MotorState>::failure("motor port out of range");
            return Result<MotorState>::success(motors_[port]);
        }

        Result<void> setServoState(PortId port, const ServoState& st)
        {
            if (port >= MAX_SERVO_PORTS) return Result<void>::failure("servo port out of range");
            ::ServoMode mode = SERVO_MODE_DISABLED;
            switch (st.mode)
            {
            case ServoMode::Disabled: mode = SERVO_MODE_DISABLED;
                break;
            case ServoMode::Enabled: mode = SERVO_MODE_ENABLED;
                break;
            case ServoMode::FullyDisabled: mode = SERVO_MODE_FULLY_DISABLED;
                break;
            }
            set_servo_mode(port, mode);
            set_servo_pos(port, st.position);
            servos_[port] = st;
            return Result<void>::success();
        }

        Result<ServoState> getServoState(PortId port) const
        {
            if (port >= MAX_SERVO_PORTS) return Result<ServoState>::failure("servo port out of range");
            return Result<ServoState>::success(servos_[port]);
        }

    private:
        Configuration::Spi cfg_;
        std::shared_ptr<Logger> logger_;
        std::array<MotorState, MAX_MOTOR_PORTS> motors_{};
        std::array<ServoState, MAX_SERVO_PORTS> servos_{};
    };
} // namespace wombat

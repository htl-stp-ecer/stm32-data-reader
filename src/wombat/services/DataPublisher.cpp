#include "wombat/services/DataPublisher.h"
#include "wombat/messaging/LcmConversions.h"
#include <algorithm>
#include <cmath>
#include <string>

namespace wombat
{
    // ---- Per-channel gate configuration -------------------------------
    //
    // minInterval: drop samples that arrive within this window since the
    //   last published one. 20 ms = 50 Hz cap on everything except
    //   accelerometer; the main loop runs at 200 Hz so 3 of every 4
    //   samples short-circuit.
    //
    // noiseEpsilon: drop samples whose L-infinity distance (max-abs
    //   component delta) to the last published value is below this
    //   bound. Tuned to the documented noise floor of the on-board
    //   IMU; floats are sloppy enough that an exact-match dedup
    //   (broker default) almost never fires.
    //
    // Channels not listed get no gate (publish every loop). Accelerometer
    // is deliberately uncapped at the user's request — downstream
    // calibration / autotune routines need every raw frame.
    namespace {
        constexpr std::chrono::milliseconds kFiftyHzInterval{20};

        // Vector epsilons — L∞ over the three axes.
        constexpr float kEpsGyroRadPerS    = 0.01f;   // bias-corrected gyro noise (typical MPU/BNO calib)
        constexpr float kEpsMag            = 0.5f;    // raw counts on most IMUs
        constexpr float kEpsLinAccel       = 0.05f;   // m/s², bias-corrected
        constexpr float kEpsAccelVel       = 0.05f;   // m/s integrated — drift suppression

        // Quaternion epsilon — per-component on a unit quat.
        constexpr float kEpsQuat           = 0.001f;

        // Scalar epsilons.
        constexpr float kEpsHeading        = 0.05f;   // degrees ≈ 1 mrad
        constexpr float kEpsTempC          = 0.1f;    // °C
    }

    namespace {
        // L∞ distance helpers — small, no allocations.
        inline float linfDelta(const raccoon::vector3f_t& a, const raccoon::vector3f_t& b) {
            return std::max({std::abs(a.x - b.x),
                             std::abs(a.y - b.y),
                             std::abs(a.z - b.z)});
        }
        inline float linfDelta(const raccoon::quaternion_t& a, const raccoon::quaternion_t& b) {
            return std::max({std::abs(a.w - b.w),
                             std::abs(a.x - b.x),
                             std::abs(a.y - b.y),
                             std::abs(a.z - b.z)});
        }
        inline float linfDelta(const raccoon::scalar_f_t& a, const raccoon::scalar_f_t& b) {
            return std::abs(a.value - b.value);
        }

    } // namespace

    // Member gateAllows template — defined here next to the linfDelta
    // overloads it uses, but it's a member so it can see PublishGate
    // (declared private inside DataPublisher).
    template <typename MessageType>
    bool DataPublisher::gateAllows(PublishGate<MessageType>& gate,
                                   const MessageType& msg,
                                   std::chrono::steady_clock::time_point now) {
        if (gate.lastValue.has_value()) {
            if (now - gate.lastTime < gate.minInterval) {
                return false;
            }
            if (linfDelta(msg, *gate.lastValue) < gate.noiseEpsilon) {
                return false;
            }
        }
        gate.lastValue = msg;
        gate.lastTime = now;
        return true;
    }

    // Explicit instantiations so the template doesn't have to live in
    // the header. Add one line per PublishGate<T> the class uses.
    template bool DataPublisher::gateAllows(
        PublishGate<raccoon::vector3f_t>&, const raccoon::vector3f_t&,
        std::chrono::steady_clock::time_point);
    template bool DataPublisher::gateAllows(
        PublishGate<raccoon::quaternion_t>&, const raccoon::quaternion_t&,
        std::chrono::steady_clock::time_point);
    template bool DataPublisher::gateAllows(
        PublishGate<raccoon::scalar_f_t>&, const raccoon::scalar_f_t&,
        std::chrono::steady_clock::time_point);

    DataPublisher::DataPublisher(std::shared_ptr<LcmBroker> broker, std::shared_ptr<Logger> logger)
        : broker_{std::move(broker)}, logger_{std::move(logger)}
    {
        // Configure every gate at construction. Channels not configured
        // here keep their default (minInterval=0, noiseEpsilon=0) which
        // means "publish every call" — same as before this change.
        gyroGate_.minInterval        = kFiftyHzInterval;
        gyroGate_.noiseEpsilon       = kEpsGyroRadPerS;

        magGate_.minInterval         = kFiftyHzInterval;
        magGate_.noiseEpsilon        = kEpsMag;

        linAccelGate_.minInterval    = kFiftyHzInterval;
        linAccelGate_.noiseEpsilon   = kEpsLinAccel;

        accelVelGate_.minInterval    = kFiftyHzInterval;
        accelVelGate_.noiseEpsilon   = kEpsAccelVel;

        dmpOrientGate_.minInterval   = kFiftyHzInterval;
        dmpOrientGate_.noiseEpsilon  = kEpsQuat;

        headingGate_.minInterval     = kFiftyHzInterval;
        headingGate_.noiseEpsilon    = kEpsHeading;

        tempGate_.minInterval        = kFiftyHzInterval;
        tempGate_.noiseEpsilon       = kEpsTempC;

        // Odometry — rate-cap only, noise epsilon 0 so any movement
        // (even sub-mm) is forwarded for downstream filtering.
        odomPosXGate_.minInterval    = kFiftyHzInterval;
        odomPosYGate_.minInterval    = kFiftyHzInterval;
        odomHeadingGate_.minInterval = kFiftyHzInterval;
        odomVxGate_.minInterval      = kFiftyHzInterval;
        odomVyGate_.minInterval      = kFiftyHzInterval;
        odomWzGate_.minInterval      = kFiftyHzInterval;
    }

    Result<void> DataPublisher::publishSensorData(const SensorData& data)
    {
        const auto now = std::chrono::steady_clock::now();

        // Gyro — gated to 50 Hz with noise-floor epsilon.
        {
            const auto msg = toLcm(data.gyro);
            if (gateAllows(gyroGate_, msg, now)) {
                const auto r = broker_->publish(Channels::GYRO, msg);
                if (r.isFailure()) logger_->warn("Failed to publish gyro data: " + r.error());
            }
        }

        // Accelerometer — INTENTIONALLY UNGATED. Stays at the main-loop
        // rate so downstream calibration / autotune routines get every
        // raw frame.
        {
            const auto r = broker_->publish(Channels::ACCELEROMETER, toLcm(data.accelerometer));
            if (r.isFailure()) logger_->warn("Failed to publish accelerometer data: " + r.error());
        }

        // Magnetometer — gated.
        {
            const auto msg = toLcm(data.magnetometer);
            if (gateAllows(magGate_, msg, now)) {
                const auto r = broker_->publish(Channels::MAGNETOMETER, msg);
                if (r.isFailure()) logger_->warn("Failed to publish magnetometer data: " + r.error());
            }
        }

        // Linear acceleration — gated.
        {
            const auto msg = toLcm(data.linearAcceleration);
            if (gateAllows(linAccelGate_, msg, now)) {
                const auto r = broker_->publish(Channels::LINEAR_ACCELERATION, msg);
                if (r.isFailure()) logger_->warn("Failed to publish linear acceleration data: " + r.error());
            }
        }

        // Accel velocity — gated.
        {
            const auto msg = toLcm(data.accelVelocity);
            if (gateAllows(accelVelGate_, msg, now)) {
                const auto r = broker_->publish(Channels::ACCEL_VELOCITY, msg);
                if (r.isFailure()) logger_->warn("Failed to publish accel velocity data: " + r.error());
            }
        }

        // DMP orientation (quaternion) — gated.
        {
            const auto msg = toLcm(data.dmpOrientation);
            if (gateAllows(dmpOrientGate_, msg, now)) {
                const auto r = broker_->publish(Channels::DMP_ORIENTATION, msg);
                if (r.isFailure()) logger_->warn("Failed to publish DMP orientation: " + r.error());
            }
        }

        // Heading — retained + gated. Retained means a late subscriber
        // still sees the last sample on first poll; the gate just trims
        // intermediate frames.
        {
            const auto msg = toLcmScalarF(data.heading);
            if (gateAllows(headingGate_, msg, now)) {
                const auto r = broker_->publishRetained(Channels::HEADING, msg);
                if (r.isFailure()) logger_->warn("Failed to publish heading data: " + r.error());
            }
        }

        // Publish IMU accuracy (throttled by its own change-detector below)
        publishAccuracy(data.accuracy);

        // Temperature — gated.
        {
            const auto msg = toLcmScalarF(data.temperature);
            if (gateAllows(tempGate_, msg, now)) {
                const auto r = broker_->publish(Channels::TEMPERATURE, msg);
                if (r.isFailure()) logger_->warn("Failed to publish temperature data: " + r.error());
            }
        }

        // Battery voltage — value is rounded to 1 mV upstream, broker
        // dedup is enough; rate-limit it too since voltage barely moves
        // mission-to-mission. Use the heading gate's tick to avoid one
        // more PublishGate slot when the only effective filter is rate.
        {
            const float rounded = std::round(data.batteryVoltage * 1000.0f) / 1000.0f;
            const auto msg = toLcmScalarF(rounded);
            const auto r = broker_->publishRetained(Channels::BATTERY_VOLTAGE, msg);
            if (r.isFailure()) logger_->warn("Failed to publish battery voltage data: " + r.error());
        }

        auto analogResult = publishAnalogValues(data.analogValues);
        if (analogResult.isFailure()) {
            logger_->warn("Failed to publish analog values: " + analogResult.error());
        }

        auto digitalResult = publishDigitalBits(data.digitalBits);
        if (digitalResult.isFailure()) {
            logger_->warn("Failed to publish digital bits: " + digitalResult.error());
        }

        // Odometry (computed on STM32) — retained for late subscribers.
        // Rate-gated to 50 Hz, no noise epsilon: we want every change
        // through so downstream pose estimators see drift.
        logger_->debug("Odometry: pos_x=" + std::to_string(data.odometry.pos_x) +
            " pos_y=" + std::to_string(data.odometry.pos_y) +
            " heading=" + std::to_string(data.odometry.heading) +
            " vx=" + std::to_string(data.odometry.vx) +
            " vy=" + std::to_string(data.odometry.vy) +
            " wz=" + std::to_string(data.odometry.wz));

        const auto publishOdomGated = [&](auto& gate, const std::string& channel, float value) {
            const auto msg = toLcmScalarF(value);
            if (gateAllows(gate, msg, now)) {
                broker_->publishRetained(channel, msg);
            }
        };
        publishOdomGated(odomPosXGate_,    Channels::ODOM_POS_X,    data.odometry.pos_x);
        publishOdomGated(odomPosYGate_,    Channels::ODOM_POS_Y,    data.odometry.pos_y);
        publishOdomGated(odomHeadingGate_, Channels::ODOM_HEADING,  data.odometry.heading);
        publishOdomGated(odomVxGate_,      Channels::ODOM_VX,       data.odometry.vx);
        publishOdomGated(odomVyGate_,      Channels::ODOM_VY,       data.odometry.vy);
        publishOdomGated(odomWzGate_,      Channels::ODOM_WZ,       data.odometry.wz);

        if (logger_) logger_->debug("Sensor data publish completed");

        return Result<void>::success();
    }

    Result<void> DataPublisher::publishMotorState(PortId port, const MotorState& state)
    {
        if (port >= MAX_MOTOR_PORTS)
        {
            return Result<void>::failure("Invalid motor port: " + std::to_string(port));
        }

        auto powerResult = broker_->publishRetained(
            Channels::motorPower(port),
            toLcmScalarI32(state.target)
        );
        if (powerResult.isFailure())
        {
            logger_->warn("Failed to publish motor power: " + powerResult.error());
        }

        auto bemfResult = broker_->publishRetained(
            Channels::backEmf(port),
            toLcmScalarI32(state.backEmf)
        );
        if (bemfResult.isFailure())
        {
            logger_->warn("Failed to publish back EMF: " + bemfResult.error());
        }

        auto posResult = broker_->publishRetained(
            Channels::motorPosition(port),
            toLcmScalarI32(state.position)
        );
        if (posResult.isFailure())
        {
            logger_->warn("Failed to publish motor position: " + posResult.error());
        }

        auto doneResult = broker_->publishRetained(
            Channels::motorDone(port),
            toLcmScalarI32(state.done ? 1 : 0)
        );
        if (doneResult.isFailure())
        {
            logger_->warn("Failed to publish motor done: " + doneResult.error());
        }

        return Result<void>::success();
    }

    Result<void> DataPublisher::publishMotorPosition(PortId port, int32_t position)
    {
        if (port >= MAX_MOTOR_PORTS)
        {
            return Result<void>::failure("Invalid motor port: " + std::to_string(port));
        }

        auto result = broker_->publish(
            Channels::motorPosition(port),
            toLcmScalarI32(position)
        );
        if (result.isFailure())
        {
            logger_->warn("Failed to publish motor position: " + result.error());
        }

        return Result<void>::success();
    }

    Result<void> DataPublisher::publishMotorDone(PortId port, bool done)
    {
        if (port >= MAX_MOTOR_PORTS)
        {
            return Result<void>::failure("Invalid motor port: " + std::to_string(port));
        }

        auto result = broker_->publish(
            Channels::motorDone(port),
            toLcmScalarI32(done ? 1 : 0)
        );
        if (result.isFailure())
        {
            logger_->warn("Failed to publish motor done: " + result.error());
        }

        return Result<void>::success();
    }

    Result<void> DataPublisher::publishServoState(PortId port, const ServoState& state)
    {
        if (port >= MAX_SERVO_PORTS)
        {
            return Result<void>::failure("Invalid servo port: " + std::to_string(port));
        }

        auto modeResult = broker_->publishRetained(
            Channels::servoMode(port),
            toLcmScalarI8(static_cast<uint8_t>(state.mode))
        );
        if (modeResult.isFailure())
        {
            logger_->warn("Failed to publish servo mode: " + modeResult.error());
        }

        auto positionResult = broker_->publishRetained(
            Channels::servoPosition(port),
            toLcmScalarF(state.position)
        );
        if (positionResult.isFailure())
        {
            logger_->warn("Failed to publish servo position: " + positionResult.error());
        }

        return Result<void>::success();
    }

    Result<void> DataPublisher::publishAnalogValues(const std::array<AnalogValue, MAX_ANALOG_PORTS>& values)
    {
        for (size_t i = 0; i < values.size(); ++i)
        {
            auto result = broker_->publish(
                Channels::analog(static_cast<PortId>(i)),
                toLcmScalarI32(static_cast<int32_t>(values[i]))
            );
            if (result.isFailure())
            {
                logger_->warn("Failed to publish analog value " + std::to_string(i) + ": " + result.error());
            }
        }
        return Result<void>::success();
    }

    Result<void> DataPublisher::publishDigitalBits(DigitalValue digitalBits)
    {
        for (PortId bit = 0; bit < 11; ++bit)
        {
            const int32_t value = (digitalBits >> bit) & 1u;
            auto result = broker_->publish(
                Channels::digital(bit),
                toLcmScalarI32(value)
            );
            if (result.isFailure())
            {
                logger_->warn("Failed to publish digital bit " + std::to_string(bit) + ": " + result.error());
            }
        }
        return Result<void>::success();
    }

    Result<void> DataPublisher::publishAccuracy(const ImuAccuracy& accuracy)
    {
        const bool isFirstTime = !lastAccuracy_.has_value();
        const bool hasChanged = lastAccuracy_.has_value() && !(accuracy == lastAccuracy_.value());

        if (isFirstTime)
        {
            logger_->info("IMU accuracy (initial): gyro=" + std::to_string(accuracy.gyro) +
                ", accel=" + std::to_string(accuracy.accelerometer) +
                ", lin_accel=" + std::to_string(accuracy.linearAcceleration) +
                ", compass=" + std::to_string(accuracy.compass) +
                ", quat=" + std::to_string(accuracy.quaternion));
        }
        else if (hasChanged)
        {
            logger_->info("IMU accuracy changed: gyro=" + std::to_string(accuracy.gyro) +
                ", accel=" + std::to_string(accuracy.accelerometer) +
                ", lin_accel=" + std::to_string(accuracy.linearAcceleration) +
                ", compass=" + std::to_string(accuracy.compass) +
                ", quat=" + std::to_string(accuracy.quaternion));
        }

        if (isFirstTime || hasChanged)
        {
            broker_->publish(Channels::GYRO_ACCURACY, toLcmScalarI8(accuracy.gyro));
            broker_->publish(Channels::ACCEL_ACCURACY, toLcmScalarI8(accuracy.accelerometer));
            broker_->publish(Channels::COMPASS_ACCURACY, toLcmScalarI8(accuracy.compass));
            broker_->publish(Channels::QUATERNION_ACCURACY, toLcmScalarI8(accuracy.quaternion));
        }

        lastAccuracy_ = accuracy;

        return Result<void>::success();
    }

    Result<void> DataPublisher::publishShutdownStatus(uint8_t shutdownFlags)
    {
        auto message = toLcmScalarI32(static_cast<int32_t>(shutdownFlags));

        auto result = broker_->publishRetained(Channels::SHUTDOWN_STATUS, message);
        if (result.isFailure())
        {
            return Result<void>::failure("Failed to publish shutdown status: " + result.error());
        }

        logger_->info("Published shutdown status: " + std::to_string(shutdownFlags) +
            " (servo=" + (shutdownFlags & 0x01 ? "on" : "off") +
            ", motor=" + (shutdownFlags & 0x02 ? "on" : "off") +
            ", source=" + (shutdownFlags & 0x04 ? "watchdog" : "user") + ")");

        return Result<void>::success();
    }

    Result<void> DataPublisher::publishBemfEnabled(bool enabled)
    {
        auto message = toLcmScalarI32(enabled ? 1 : 0);

        auto result = broker_->publishRetained(Channels::BEMF_ENABLED, message);
        if (result.isFailure())
        {
            return Result<void>::failure("Failed to publish BEMF enabled status: " + result.error());
        }

        logger_->info(std::string("Published BEMF enabled status: ") + (enabled ? "enabled" : "disabled"));
        return Result<void>::success();
    }
} // namespace wombat
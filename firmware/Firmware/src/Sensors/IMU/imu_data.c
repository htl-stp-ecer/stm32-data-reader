#include "Sensors/IMU/imu.h"
#include "Sensors/IMU/imu_internal.h"

#include "mpl.h"
#include "mltypes.h"
#include "ml_math_func.h"
#include "invensense.h"
#include "eMPL_outputs.h"
#include "motion_driver_hal.h"
#include "Storage/gyro_tc_persist.h"

#include <math.h>
#include <stdio.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ── Heading fusion (gyro re-integration + ZUPT + temperature compensation) ──
// The DMP quaternion yaw (imu.heading) drifts at rest (slow gyro bias) and has
// an accel-induced scale error during fast rotation. We instead integrate the
// raw world-vertical yaw rate and remove its slow bias in TWO layers:
//
//   (1) ZUPT (zero-rate update): while the yaw rate is small the true rate is 0,
//       so the reading IS the bias. An EMA learns it; a dwell rejects post-motion
//       settling. This is our live bias correction — faster (tau=0.3s) than the
//       DMP's own ~8s no-motion GYRO_CAL, and it acts on our raw path directly.
//
//   (2) Temperature compensation: ZUPT only corrects while the robot is AT REST.
//       During a long drive the chip warms, the gyro bias drifts with it, and we
//       never ZUPT — so the drift integrates into the heading. The MPU9250 DMP
//       does NOT do gyro temp-comp (only the closed MPL host lib does, and only
//       on its own fused output, which our raw path bypasses). So we model it
//       ourselves: at each rest we log a (temperature, bias) point and fit a line
//       bias = f(T) online. Between rests we PREDICT the bias from the current
//       temperature: bias = hf_bias + slope*(T - T_anchor). Degrades gracefully
//       to pure ZUPT until enough temperature spread exists to trust the slope.
//
// Everything runs once per raw-gyro sample (readGyroFast @~200Hz) on ONE timebase
// (HAL ms tick) with the true measured dt, so integrate and bias-subtract share
// the same dt and the cancellation at rest is exact. GT = calib-board ICM.
#define HF_ZUPT_TAU        0.3f     // bias EMA time constant at rest (s)
#define HF_REST_RATE_MAX   0.5f     // deg/s: |rate-bias| below this = at rest
#define HF_REST_DWELL_MS   1000u    // continuous rest before ZUPT engages (ms)
#define HF_BIAS_MAX        1.0f     // deg/s: clamp bias estimate (safety)
// Temperature-compensation model (online linear fit of bias vs chip temp):
#define TC_DELTA_T         0.25f    // degC: min temp move to log a new fit point
                                    // (prevents a long rest at one temp from
                                    //  swamping the fit with duplicate points)
#define TC_FORGET          0.99f    // per-point forgetting (~100-point memory)
#define TC_MIN_POINTS      4.0f     // effective fit points before trusting slope
#define TC_MIN_TVAR        0.25f    // degC^2: min temp spread before trusting the
                                    // slope. ~1.7 degC range: with at-rest bias
                                    // noise ~0.05 dps and a target slope ~0.03
                                    // dps/degC, that's the range needed to fit
                                    // the slope above the noise. Combined with the
                                    // +/-0.15 clamp and >=4 points, safe vs noise;
                                    // a full cold-boot ramp (~26 degC) engages at
                                    // once, and realistic mid-run swings can too.
#define TC_SLOPE_MAX       0.15f    // dps/degC: clamp (MPU9250 tempco is small)
#define TC_MIN_R2          0.5f     // significance gate: temperature must explain
                                    // >=50% of the at-rest bias variance before we
                                    // trust (and apply) the slope. A real tempco is
                                    // a strong linear relation (high R^2); a
                                    // thermally-stable gyro fits a near-zero slope
                                    // that is pure noise (low R^2, sign flips run to
                                    // run) — this keeps it dormant there instead of
                                    // injecting a wrong-signed correction on a long
                                    // warm drive. (Measured on this robot: tempco is
                                    // ~0, so the model correctly stays dormant.)
// Persistence: save the model to flash only on a material change, rate-limited
// (flash program is sub-ms but we keep it rare — see gyro_tc_persist.c).
#define TC_SAVE_MIN_MS     60000u   // >=1 min between flash saves
#define TC_SAVE_DSLOPE     0.003f   // dps/degC: min slope change worth saving
volatile float imuFusedHeading = 0.0f;   // deg, imu.heading/CW frame, continuous
static float hf_bias = 0.0f;             // estimated yaw-rate bias (deg/s)
static inv_time_t hf_rest_since_ms = 0;
static uint8_t hf_resting = 0;
static uint8_t hf_init = 0;
// temp-comp state: predicted bias = hf_bias + tc_slope*(T - tc_T_anchor)
static float tc_slope = 0.0f;        // learned bias-vs-temp slope (dps/degC)
static float tc_T_anchor = 0.0f;     // chip temp at the last ZUPT (degC)
static float tc_last_pt_T = 0.0f;    // temp of the last logged fit point (degC)
static uint8_t tc_valid = 0;         // slope trustworthy (enough temp spread)
static uint8_t tc_seeded = 0;        // a valid temperature has been observed
// forgetting covariance accumulators for the bias-on-temp linear fit
static float tc_n = 0.0f, tc_sT = 0.0f, tc_sB = 0.0f, tc_sTT = 0.0f, tc_sTB = 0.0f;
static float tc_sBB = 0.0f;          // sum w*bias^2 — for the R^2 significance gate
// flash-persistence bookkeeping
static float tc_last_saved_slope = 0.0f;
static inv_time_t tc_last_save_ms = 0;
static uint8_t tc_saved_once = 0;

// Load any reboot-persisted temperature model so the fusion is pre-trained on
// the cold-boot warmup instead of starting empty each boot. Call once at setup,
// before motion (the persistence layer may erase-compact its flash log here).
void heading_fusion_load_persisted(void)
{
    gyro_tc_state_t st;
    if (gyro_tc_persist_load(&st))
    {
        tc_slope = st.slope;
        tc_n     = st.n;
        tc_sT    = st.sT;
        tc_sB    = st.sB;
        tc_sTT   = st.sTT;
        tc_sTB   = st.sTB;
        tc_sBB   = st.sBB;
        tc_valid = st.valid;
        tc_last_saved_slope = st.slope;
    }
}

// Persist the current model to flash if the slope has changed materially since
// the last save and the rate limit has elapsed. Called from the ZUPT branch.
static void tc_maybe_save(inv_time_t now_ms)
{
    if (!tc_valid)
        return;
    if (tc_saved_once)
    {
        // Subsequent saves need BOTH a material slope change and the rate limit.
        if (fabsf(tc_slope - tc_last_saved_slope) < TC_SAVE_DSLOPE)
            return;
        if ((inv_time_t)(now_ms - tc_last_save_ms) < TC_SAVE_MIN_MS)
            return;
    }
    // First save fires as soon as the model engages, to capture it promptly.

    gyro_tc_state_t st = { tc_slope, tc_n, tc_sT, tc_sB, tc_sTT, tc_sTB, tc_sBB, tc_valid };
    gyro_tc_persist_save(&st);
    tc_last_saved_slope = tc_slope;
    tc_last_save_ms = now_ms;
    tc_saved_once = 1;
}

// Fold one (temperature, bias) rest observation into the online linear fit and,
// once there is enough temperature spread, update the trusted slope.
static void tc_learn_point(float T, float B)
{
    tc_n   = tc_n   * TC_FORGET + 1.0f;
    tc_sT  = tc_sT  * TC_FORGET + T;
    tc_sB  = tc_sB  * TC_FORGET + B;
    tc_sTT = tc_sTT * TC_FORGET + T * T;
    tc_sTB = tc_sTB * TC_FORGET + T * B;
    tc_sBB = tc_sBB * TC_FORGET + B * B;

    if (tc_n < TC_MIN_POINTS)
        return;

    float mT = tc_sT / tc_n;
    float varT = tc_sTT / tc_n - mT * mT;
    float mB = tc_sB / tc_n;
    float varB = tc_sBB / tc_n - mB * mB;
    float covTB = tc_sTB / tc_n - mT * mB;

    // Two gates: enough temperature spread AND the fit is significant (R^2). The
    // R^2 test is what distinguishes a real tempco (bias tracks temperature
    // linearly, high R^2) from a thermally-stable gyro (bias ~ constant + noise,
    // near-zero slope whose sign is meaningless). If either fails, keep the model
    // dormant (tc_valid=0 -> bias = hf_bias, pure ZUPT) rather than apply noise.
    float r2 = (varT > 1e-6f && varB > 1e-9f) ? (covTB * covTB) / (varT * varB) : 0.0f;
    if (varT < TC_MIN_TVAR || r2 < TC_MIN_R2)
    {
        tc_valid = 0;
        return;
    }

    float s = covTB / varT;
    if (s >  TC_SLOPE_MAX) s =  TC_SLOPE_MAX;
    if (s < -TC_SLOPE_MAX) s = -TC_SLOPE_MAX;

    uint8_t was_valid = tc_valid;
    tc_slope = s;
    tc_valid = 1;
    if (!was_valid)
        // Integer format (milli-dps/degC, centi-degC, centi-R^2) to avoid
        // depending on float printf support in the newlib-nano link.
        printf("[imu] gyro tempco engaged: slope=%ld mdps/degC R2=%ld%% over %ld cdegC\r\n",
               (long)(tc_slope * 1000.0f), (long)(r2 * 100.0f),
               (long)(sqrtf(varT) * 100.0f));
}

// Advance the heading fusion by one sample: integrate the world-vertical yaw
// rate and remove its (temperature-predicted, ZUPT-anchored) bias. Fed by
// readGyroFast() (imu.c) from the raw gyro+accel registers at ~200Hz.
//   rate : yaw rate about the world vertical (deg/s), = gyro . gravity_unit
//   dt   : seconds since the previous call (measured, not fixed)
void heading_fusion_update(float rate, float dt)
{
    inv_time_t now_ms;
    hal_get_tick_count(&now_ms);       // used only for the rest-dwell timer

    if (!hf_init)
    {
        imuFusedHeading = 0.0f;        // relative; odometry captures the baseline
        hf_init = 1;
        return;
    }
    if (dt <= 0.0f || dt > 0.1f)
        return;

    float T = imu.temperature;         // chip temperature (degC), 0 until first read
    if (!tc_seeded && T != 0.0f)
    {
        tc_seeded = 1;
        tc_T_anchor = T;
        tc_last_pt_T = T;
    }

    // Bias applied this sample: the last ZUPT bias plus the temperature-predicted
    // drift since that rest. Falls back to plain hf_bias until the slope is valid.
    float bias = hf_bias;
    if (tc_valid && tc_seeded)
        bias += tc_slope * (T - tc_T_anchor);

    imuFusedHeading += (rate - bias) * dt;             // integrate, remove bias

    // At rest iff the bias-corrected yaw rate stays small for a full 1s dwell.
    // Robustness against absorbing a slow rotation as bias comes from the LONG
    // dwell + tight 0.5 dps gate: a real rest holds for seconds, a slow-spin
    // ramp never dwells <0.5 dps for a full second. Using the temp-predicted
    // bias here (not the stale hf_bias) keeps rest detection correct after the
    // temperature — and thus the true at-rest reading — has drifted mid-run.
    if (fabsf(rate - bias) >= HF_REST_RATE_MAX)
    {
        hf_resting = 0;
    }
    else
    {
        if (!hf_resting)
        {
            hf_resting = 1;
            hf_rest_since_ms = now_ms;
        }
        if ((inv_time_t)(now_ms - hf_rest_since_ms) >= HF_REST_DWELL_MS)
        {
            float g = dt / HF_ZUPT_TAU;
            if (g > 1.0f) g = 1.0f;
            hf_bias += g * (rate - hf_bias);           // true rate is 0 at rest
            if (hf_bias > HF_BIAS_MAX) hf_bias = HF_BIAS_MAX;
            if (hf_bias < -HF_BIAS_MAX) hf_bias = -HF_BIAS_MAX;

            if (tc_seeded)
            {
                tc_T_anchor = T;                       // re-anchor prediction here
                // Log a fit point only when the temperature has moved enough, so
                // long rests at one temp don't flood the fit with duplicates —
                // the spread comes from rests at DIFFERENT temperatures over time.
                if (fabsf(T - tc_last_pt_T) >= TC_DELTA_T)
                {
                    tc_learn_point(T, hf_bias);
                    tc_last_pt_T = T;
                    tc_maybe_save(now_ms);   // persist across reboots (gated)
                }
            }
        }
    }
}

static void rotateBodyToWorld(const long rot_q30[9], const long body[3], long world[3])
{
    for (int i = 0; i < 3; i++)
    {
        world[i] = inv_q30_mult(rot_q30[i * 3 + 0], body[0])
            + inv_q30_mult(rot_q30[i * 3 + 1], body[1])
            + inv_q30_mult(rot_q30[i * 3 + 2], body[2]);
    }
}

static void read_heading(const long quat[4])
{
    long q00, q03, q12, q22;
    long t1, t2;
    float heading_deg;

    q00 = inv_q29_mult(quat[0], quat[0]);
    q03 = inv_q29_mult(quat[0], quat[3]);
    q12 = inv_q29_mult(quat[1], quat[2]);
    q22 = inv_q29_mult(quat[2], quat[2]);
    t1 = q12 - q03;
    t2 = q22 + q00 - (1L << 30);
    heading_deg = atan2f((float)t1, (float)t2) * 180.f / (float)M_PI;
    if (heading_deg < 0.f)
        heading_deg += 360.f;
    imu.heading = heading_deg;
}

// ── Externally correct the published DMP quaternion's yaw ────────────────────
// The DMP quaternion's tilt (roll/pitch) is gravity-referenced and drift-free,
// but its yaw drifts (~0.7deg at rest; the DMP's own GYRO_CAL can't fix it — the
// residual is algorithmic, not a static bias, so feeding it our bias does
// nothing). So instead of correcting the DMP from the inside, we fix its OUTPUT:
// keep the good tilt, replace the drifting yaw with our ZUPT-locked fused heading
// by rotating the whole orientation about the world vertical by the yaw error.
// The constant offset between the two yaw zeroes is locked once; thereafter dyaw
// is exactly the DMP's accumulated yaw error, which the rotation cancels — making
// the published quaternion's yaw track our drift-free heading (0.4deg vs GT).
#define DMP_YAW_CORRECT   1
#define DMP_YAW_SIGN      (-1.0f)   // world-vertical rotation sense (hw-validated:
                                    // +1 doubled the drift, -1 cancels it)
static float dmp_yaw_off = 0.0f;
static uint8_t dmp_yaw_off_locked = 0;

static void correct_dmp_quat_yaw(const long dmp_quat[4])
{
    if (!hf_init)
        return;                              // wait for the fusion to initialise

    float fused = imuFusedHeading;
    if (!dmp_yaw_off_locked)
    {
        dmp_yaw_off = imu.heading - fused;   // lock the DMP-vs-fused zero offset
        dmp_yaw_off_locked = 1;
    }
    float corrected_yaw = fused + dmp_yaw_off;
    float dyaw = corrected_yaw - imu.heading; // the DMP yaw error to remove (deg)
    while (dyaw > 180.0f)  dyaw -= 360.0f;
    while (dyaw < -180.0f) dyaw += 360.0f;

    // q_corr = q_z(dyaw) (x) q_dmp  — a world-vertical (yaw) rotation, tilt intact.
    float h = DMP_YAW_SIGN * dyaw * 0.5f * (float)M_PI / 180.0f;
    float cz = cosf(h), sz = sinf(h);        // q_z = (cz, 0, 0, sz)
    float qw = inv_q30_to_float(dmp_quat[0]);
    float qx = inv_q30_to_float(dmp_quat[1]);
    float qy = inv_q30_to_float(dmp_quat[2]);
    float qz = inv_q30_to_float(dmp_quat[3]);
    imu.dmpQuat.data[0] = cz * qw - sz * qz;
    imu.dmpQuat.data[1] = cz * qx - sz * qy;
    imu.dmpQuat.data[2] = cz * qy + sz * qx;
    imu.dmpQuat.data[3] = cz * qz + sz * qw;
    imu.dmpQuat.accuracy = 3;
}

static void integrate_linear_accel(void)
{
    static float filtered_accel[3] = {0, 0, 0};
    static uint32_t last_integration_time = 0;

    const float alpha = 0.3f;
    for (int i = 0; i < 3; i++)
    {
        filtered_accel[i] = alpha * imu.linearAccel.data[i] + (1.0f - alpha) * filtered_accel[i];
    }

    inv_time_t now;
    hal_get_tick_count(&now);
    if (last_integration_time != 0)
    {
        float dt = (float)(now - last_integration_time) / 1000.0f;
        if (dt > 0.0f && dt < 0.1f)
        {
            for (int i = 0; i < 2; i++)
            {
                imu.accelVelocity.data[i] += filtered_accel[i] * dt;
                imu.accelVelocity.data[i] *= 0.998f;
            }
        }
    }
    last_integration_time = now;
    imu.accelVelocity.accuracy = imu.linearAccel.accuracy;
}

void imu_read_from_mpl(void)
{
    long data[9];
    long quat[4];
    long rot_mat[9] = {
        ROT_MATRIX_SCALE_LONG, 0, 0,
        0, ROT_MATRIX_SCALE_LONG, 0,
        0, 0, ROT_MATRIX_SCALE_LONG
    };
    unsigned long timestamp;
    long world[3];
    int quat_accuracy = 0;

    // DMP 6-axis quaternion for body-to-world rotation
    long dmp_quat[4];
    inv_get_6axis_quaternion(dmp_quat);

    // /* 9-axis MPL quaternion (gyro+accel+compass) */
    // inv_get_quaternion_set(quat, &quat_accuracy, (inv_time_t*)&timestamp);
    // imu.quat.data[0] = inv_q30_to_float(quat[0]);
    // imu.quat.data[1] = inv_q30_to_float(quat[1]);
    // imu.quat.data[2] = inv_q30_to_float(quat[2]);
    // imu.quat.data[3] = inv_q30_to_float(quat[3]);
    // imu.quat.accuracy = (int8_t)quat_accuracy;

    // Use DMP 6-axis quat for body-to-world rotation (stable, no mag influence)
    inv_quaternion_to_rotation(dmp_quat, rot_mat);

    if (inv_get_sensor_type_gyro(data, &imu.gyro.accuracy, (inv_time_t*)&timestamp))
    {
        rotateBodyToWorld(rot_mat, data, world);
        imu.gyro.data[0] = inv_q16_to_float(world[0]);
        imu.gyro.data[1] = inv_q16_to_float(world[1]);
        imu.gyro.data[2] = inv_q16_to_float(world[2]);
        // (heading fusion now runs in the DMP-free high-rate path: readGyroFast)
    }

    if (inv_get_sensor_type_accel(data, &imu.accel.accuracy, (inv_time_t*)&timestamp))
    {
        rotateBodyToWorld(rot_mat, data, world);
        imu.accel.data[0] = EARTHS_GRAVITY * inv_q16_to_float(world[0]);
        imu.accel.data[1] = EARTHS_GRAVITY * inv_q16_to_float(world[1]);
        imu.accel.data[2] = EARTHS_GRAVITY * inv_q16_to_float(world[2]);
    }

    read_heading(dmp_quat);

#if DMP_YAW_CORRECT
    // Replace the drifting DMP yaw with our ZUPT-locked heading (tilt kept).
    correct_dmp_quat_yaw(dmp_quat);
#endif

    if (inv_get_sensor_type_compass(data, &imu.compass.accuracy, (inv_time_t*)&timestamp))
    {
        imu.compass.data[0] = inv_q16_to_float(data[0]);
        imu.compass.data[1] = inv_q16_to_float(data[1]);
        imu.compass.data[2] = inv_q16_to_float(data[2]);
    }

    if (inv_get_linear_accel(data) == INV_SUCCESS)
    {
        rotateBodyToWorld(rot_mat, data, world);
        imu.linearAccel.data[0] = inv_q16_to_float(world[0]);
        imu.linearAccel.data[1] = inv_q16_to_float(world[1]);
        imu.linearAccel.data[2] = inv_q16_to_float(world[2]);

        integrate_linear_accel();
    }
}

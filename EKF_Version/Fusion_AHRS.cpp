/**
 * @file Fusion_AHRS.cpp
 * @brief AHRS implementation for EKF_Version.
 */

#include <float.h>
#include <math.h>
#include "Fusion_AHRS.h"

#ifdef __cplusplus
extern "C" {
#endif

MadgwickParam madgwickParam;
int use_grad = 1;

#define INITIAL_GAIN (10.0f)
#define INITIALISATION_PERIOD (3.0f)
#define CUTOFF_FREQUENCY (0.02f)
#define TIMEOUT (5)
#define THRESHOLD (3.0f)

static inline FusionVector HalfGravity(const FusionAhrs *const ahrs);
static inline FusionVector Feedback(const FusionVector sensor, const FusionVector reference);
static inline int Clamp(const int value, const int min, const int max);

void FusionAhrsInitialise(FusionAhrs *const ahrs) {
    const FusionAhrsSettings settings = {
        .convention = FusionConventionNwu,
        .gain = 0.5f,
        .gyroscopeRange = 2000.0f,
        .accelerationRejection = 30.0f,
        .recoveryTriggerPeriod = 0,
    };
    FusionAhrsSetSettings(ahrs, &settings);
    FusionAhrsReset(ahrs);
}

void FusionAhrsReset(FusionAhrs *const ahrs) {
    ahrs->quaternion = FUSION_IDENTITY_QUATERNION;
    ahrs->accelerometer = FUSION_VECTOR_ZERO;
    ahrs->angularRateRecovery = false;
    ahrs->halfAccelerometerFeedback = FUSION_VECTOR_ZERO;
    ahrs->accelerometerIgnored = false;
    ahrs->accelerationRecoveryTrigger = 0;
    ahrs->accelerationRecoveryTimeout = ahrs->settings.recoveryTriggerPeriod;
    ahrs->rampedGain = INITIAL_GAIN;
    ahrs->initialising = true;
    ahrs->rampedGainStep = (INITIAL_GAIN - ahrs->settings.gain) / INITIALISATION_PERIOD;

    madgwickParam.beta = BETA_DEF;
    madgwickParam.invSampleFreq = 1.0f / SAMPLE_FREQ_DEF;
    madgwickParam.q0 = 1.0f;
    madgwickParam.q1 = 0.0f;
    madgwickParam.q2 = 0.0f;
    madgwickParam.q3 = 0.0f;
    madgwickParam.minFusionPower = 0.01f;
    madgwickParam.maxFusionPower = 0.5f;
    madgwickParam.Fusion_Power = 0.1f;
    madgwickParam.filterAlpha = 0.1f;
    madgwickParam.fq0 = 1.0f;
    madgwickParam.fq1 = 0.0f;
    madgwickParam.fq2 = 0.0f;
    madgwickParam.fq3 = 0.0f;
    madgwickParam.v_s0 = 0.0f;
    madgwickParam.v_s1 = 0.0f;
    madgwickParam.v_s2 = 0.0f;
    madgwickParam.v_s3 = 0.0f;
    madgwickParam.momentum = 0.0f;
}

void FusionAhrsSetSettings(FusionAhrs *const ahrs, const FusionAhrsSettings *const settings) {
    ahrs->settings.convention = settings->convention;
    ahrs->settings.gain = settings->gain;
    ahrs->settings.gyroscopeRange = settings->gyroscopeRange == 0.0f ? FLT_MAX : 0.98f * settings->gyroscopeRange;
    ahrs->settings.accelerationRejection = settings->accelerationRejection == 0.0f ? FLT_MAX :
        powf(0.5f * sinf(FusionDegreesToRadians(settings->accelerationRejection)), 2.0f);
    ahrs->settings.recoveryTriggerPeriod = settings->recoveryTriggerPeriod;
    ahrs->accelerationRecoveryTimeout = ahrs->settings.recoveryTriggerPeriod;

    if ((settings->gain == 0.0f) || (settings->recoveryTriggerPeriod == 0)) {
        ahrs->settings.accelerationRejection = FLT_MAX;
    }

    if (ahrs->initialising == false) {
        ahrs->rampedGain = ahrs->settings.gain;
    }

    ahrs->rampedGainStep = (INITIAL_GAIN - ahrs->settings.gain) / INITIALISATION_PERIOD;
}

void FusionAhrsUpdateNoMagnetometer(FusionAhrs *const ahrs, const FusionVector gyroscope,
                                   const FusionVector accelerometer, const float deltaTime) {
    FusionAhrsUpdate(ahrs, gyroscope, accelerometer, deltaTime);
}

void FusionAhrsUpdate(FusionAhrs *const ahrs, const FusionVector gyroscope,
                      const FusionVector accelerometer, const float deltaTime) {
#define Q ahrs->quaternion.element
    ahrs->accelerometer = accelerometer;

    if ((fabsf(gyroscope.axis.x) > ahrs->settings.gyroscopeRange) ||
        (fabsf(gyroscope.axis.y) > ahrs->settings.gyroscopeRange) ||
        (fabsf(gyroscope.axis.z) > ahrs->settings.gyroscopeRange)) {
        const FusionQuaternion quaternion = ahrs->quaternion;
        FusionAhrsReset(ahrs);
        ahrs->quaternion = quaternion;
        ahrs->angularRateRecovery = true;
    }

    if (ahrs->initialising) {
        ahrs->rampedGain -= ahrs->rampedGainStep * deltaTime;
        if (ahrs->rampedGain < ahrs->settings.gain) {
            ahrs->rampedGain = ahrs->settings.gain;
            ahrs->initialising = false;
        }
    }

    const FusionVector halfGravity = HalfGravity(ahrs);

    FusionVector halfAccelerometerFeedback = FUSION_VECTOR_ZERO;
    ahrs->accelerometerIgnored = true;

    if (FusionVectorIsZero(accelerometer) == false) {
        ahrs->halfAccelerometerFeedback = Feedback(FusionVectorNormalise(accelerometer), halfGravity);

        if ((FusionVectorMagnitudeSquared(ahrs->halfAccelerometerFeedback) <= ahrs->settings.accelerationRejection)) {
            ahrs->accelerometerIgnored = false;
            ahrs->accelerationRecoveryTrigger -= 9;
        } else {
            ahrs->accelerationRecoveryTrigger += 1;
        }

        if (ahrs->accelerationRecoveryTrigger > ahrs->accelerationRecoveryTimeout) {
            ahrs->accelerationRecoveryTimeout = 0;
            ahrs->accelerometerIgnored = false;
        } else {
            ahrs->accelerationRecoveryTimeout = ahrs->settings.recoveryTriggerPeriod;
        }

        ahrs->accelerationRecoveryTrigger = Clamp(ahrs->accelerationRecoveryTrigger, 0,
                                                  ahrs->settings.recoveryTriggerPeriod);

        if (ahrs->accelerometerIgnored == false) {
            halfAccelerometerFeedback = ahrs->halfAccelerometerFeedback;
        }
    }

    const FusionVector halfGyroscope = FusionVectorMultiplyScalar(gyroscope, FusionDegreesToRadians(0.5f));
    const FusionVector adjustedHalfGyroscope = FusionVectorAdd(halfGyroscope,
        FusionVectorMultiplyScalar(halfAccelerometerFeedback, ahrs->rampedGain));

    ahrs->quaternion = FusionQuaternionAdd(ahrs->quaternion,
        FusionQuaternionMultiplyVector(ahrs->quaternion,
            FusionVectorMultiplyScalar(adjustedHalfGyroscope, deltaTime)));
    ahrs->quaternion = FusionQuaternionNormalise(ahrs->quaternion);

    if (use_grad) {
        madgwickParam.invSampleFreq = deltaTime;
        Madgwick_updateIMU(gyroscope.axis.x, gyroscope.axis.y, gyroscope.axis.z,
                           accelerometer.axis.x, accelerometer.axis.y, accelerometer.axis.z);

        const float dot_product = ahrs->quaternion.element.w * madgwickParam.q0 +
                                  ahrs->quaternion.element.x * madgwickParam.q1 +
                                  ahrs->quaternion.element.y * madgwickParam.q2 +
                                  ahrs->quaternion.element.z * madgwickParam.q3;

        float similarity_error = 1.0f - fabsf(dot_product);
        float dynamicFusionPower = madgwickParam.minFusionPower +
                                   similarity_error * (madgwickParam.maxFusionPower -
                                                       madgwickParam.minFusionPower);
        if (dynamicFusionPower > madgwickParam.maxFusionPower) {
            dynamicFusionPower = madgwickParam.maxFusionPower;
        }

        madgwickParam.fq0 = madgwickParam.filterAlpha * madgwickParam.q0 +
                            (1.0f - madgwickParam.filterAlpha) * madgwickParam.fq0;
        madgwickParam.fq1 = madgwickParam.filterAlpha * madgwickParam.q1 +
                            (1.0f - madgwickParam.filterAlpha) * madgwickParam.fq1;
        madgwickParam.fq2 = madgwickParam.filterAlpha * madgwickParam.q2 +
                            (1.0f - madgwickParam.filterAlpha) * madgwickParam.fq2;
        madgwickParam.fq3 = madgwickParam.filterAlpha * madgwickParam.q3 +
                            (1.0f - madgwickParam.filterAlpha) * madgwickParam.fq3;

        float recipNorm = FusionFastInverseSqrt(
            madgwickParam.fq0 * madgwickParam.fq0 +
            madgwickParam.fq1 * madgwickParam.fq1 +
            madgwickParam.fq2 * madgwickParam.fq2 +
            madgwickParam.fq3 * madgwickParam.fq3);

        madgwickParam.fq0 *= recipNorm;
        madgwickParam.fq1 *= recipNorm;
        madgwickParam.fq2 *= recipNorm;
        madgwickParam.fq3 *= recipNorm;

        Q.w = (1.0f - dynamicFusionPower) * Q.w + dynamicFusionPower * madgwickParam.fq0;
        Q.x = (1.0f - dynamicFusionPower) * Q.x + dynamicFusionPower * madgwickParam.fq1;
        Q.y = (1.0f - dynamicFusionPower) * Q.y + dynamicFusionPower * madgwickParam.fq2;
        Q.z = (1.0f - dynamicFusionPower) * Q.z + dynamicFusionPower * madgwickParam.fq3;

        ahrs->quaternion = FusionQuaternionNormalise(ahrs->quaternion);
    }
#undef Q
}

static inline FusionVector HalfGravity(const FusionAhrs *const ahrs) {
#define Q ahrs->quaternion.element
    switch (ahrs->settings.convention) {
        case FusionConventionNwu:
        case FusionConventionEnu: {
            const FusionVector halfGravity = {.axis = {
                .x = Q.x * Q.z - Q.w * Q.y,
                .y = Q.y * Q.z + Q.w * Q.x,
                .z = Q.w * Q.w - 0.5f + Q.z * Q.z,
            }};
            return halfGravity;
        }
        case FusionConventionNed: {
            const FusionVector halfGravity = {.axis = {
                .x = Q.w * Q.y - Q.x * Q.z,
                .y = -1.0f * (Q.y * Q.z + Q.w * Q.x),
                .z = 0.5f - Q.w * Q.w - Q.z * Q.z,
            }};
            return halfGravity;
        }
    }
    return FUSION_VECTOR_ZERO;
#undef Q
}

static inline FusionVector Feedback(const FusionVector sensor, const FusionVector reference) {
    if (FusionVectorDotProduct(sensor, reference) < 0.0f) {
        return FusionVectorNormalise(FusionVectorCrossProduct(sensor, reference));
    }
    return FusionVectorCrossProduct(sensor, reference);
}

static inline int Clamp(const int value, const int min, const int max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

FusionQuaternion FusionAhrsGetQuaternion(const FusionAhrs *const ahrs) {
    return ahrs->quaternion;
}

void FusionAhrsSetQuaternion(FusionAhrs *const ahrs, const FusionQuaternion quaternion) {
    ahrs->quaternion = quaternion;
}

FusionVector FusionAhrsGetGravity(const FusionAhrs *const ahrs) {
#define Q ahrs->quaternion.element
    const FusionVector gravity = {.axis = {
        .x = 2.0f * (Q.x * Q.z - Q.w * Q.y),
        .y = 2.0f * (Q.y * Q.z + Q.w * Q.x),
        .z = 2.0f * (Q.w * Q.w - 0.5f + Q.z * Q.z),
    }};
    return gravity;
#undef Q
}

FusionVector FusionAhrsGetLinearAcceleration(const FusionAhrs *const ahrs) {
    switch (ahrs->settings.convention) {
        case FusionConventionNwu:
        case FusionConventionEnu:
            return FusionVectorSubtract(ahrs->accelerometer, FusionAhrsGetGravity(ahrs));
        case FusionConventionNed:
            return FusionVectorAdd(ahrs->accelerometer, FusionAhrsGetGravity(ahrs));
    }
    return FUSION_VECTOR_ZERO;
}

FusionVector FusionAhrsGetEarthAcceleration(const FusionAhrs *const ahrs) {
#define Q ahrs->quaternion.element
#define A ahrs->accelerometer.axis
    const float qwqw = Q.w * Q.w;
    const float qwqx = Q.w * Q.x;
    const float qwqy = Q.w * Q.y;
    const float qwqz = Q.w * Q.z;
    const float qxqy = Q.x * Q.y;
    const float qxqz = Q.x * Q.z;
    const float qyqz = Q.y * Q.z;

    FusionVector accelerometer = {.axis = {
        .x = 2.0f * ((qwqw - 0.5f + Q.x * Q.x) * A.x + (qxqy - qwqz) * A.y + (qxqz + qwqy) * A.z),
        .y = 2.0f * ((qxqy + qwqz) * A.x + (qwqw - 0.5f + Q.y * Q.y) * A.y + (qyqz - qwqx) * A.z),
        .z = 2.0f * ((qxqz - qwqy) * A.x + (qyqz + qwqx) * A.y + (qwqw - 0.5f + Q.z * Q.z) * A.z),
    }};

    switch (ahrs->settings.convention) {
        case FusionConventionNwu:
        case FusionConventionEnu:
            accelerometer.axis.z -= 1.0f;
            break;
        case FusionConventionNed:
            accelerometer.axis.z += 1.0f;
            break;
    }

    return accelerometer;
#undef Q
#undef A
}

FusionAhrsInternalStates FusionAhrsGetInternalStates(const FusionAhrs *const ahrs) {
    const FusionAhrsInternalStates internalStates = {
        .accelerationError = FusionRadiansToDegrees(
            FusionAsin(2.0f * FusionVectorMagnitude(ahrs->halfAccelerometerFeedback))),
        .accelerometerIgnored = ahrs->accelerometerIgnored,
        .accelerationRecoveryTrigger = ahrs->settings.recoveryTriggerPeriod == 0 ? 0.0f :
            (float) ahrs->accelerationRecoveryTrigger / (float) ahrs->settings.recoveryTriggerPeriod,
    };
    return internalStates;
}

FusionAhrsFlags FusionAhrsGetFlags(const FusionAhrs *const ahrs) {
    const FusionAhrsFlags flags = {
        .angularRateRecovery = ahrs->angularRateRecovery,
        .accelerationRecovery = ahrs->accelerationRecoveryTrigger > ahrs->accelerationRecoveryTimeout,
    };
    return flags;
}

void FusionAhrsSetHeading(FusionAhrs *const ahrs, const float heading) {
#define Q ahrs->quaternion.element
    const float yaw = atan2f(Q.w * Q.z + Q.x * Q.y, 0.5f - Q.y * Q.y - Q.z * Q.z);
    const float halfYawMinusHeading = 0.5f * (yaw - FusionDegreesToRadians(heading));
    const FusionQuaternion rotation = {.element = {
        .w = cosf(halfYawMinusHeading),
        .x = 0.0f,
        .y = 0.0f,
        .z = -1.0f * sinf(halfYawMinusHeading),
    }};
    ahrs->quaternion = FusionQuaternionMultiply(rotation, ahrs->quaternion);
#undef Q
}

void FusionOffsetInitialise(FusionOffset *const offset, const unsigned int sampleRate) {
    offset->filterCoefficient = 2.0f * (float) M_PI * CUTOFF_FREQUENCY * (1.0f / (float) sampleRate);
    offset->timeout = TIMEOUT * sampleRate;
    offset->timer = 0;
    offset->gyroscopeOffset = FUSION_VECTOR_ZERO;
}

FusionVector FusionOffsetUpdate(FusionOffset *const offset, FusionVector gyroscope) {
    gyroscope = FusionVectorSubtract(gyroscope, offset->gyroscopeOffset);

    if ((fabsf(gyroscope.axis.x) > THRESHOLD) ||
        (fabsf(gyroscope.axis.y) > THRESHOLD) ||
        (fabsf(gyroscope.axis.z) > THRESHOLD)) {
        offset->timer = 0;
        return gyroscope;
    }

    if (offset->timer < offset->timeout) {
        offset->timer++;
        return gyroscope;
    }

    offset->gyroscopeOffset = FusionVectorAdd(offset->gyroscopeOffset,
        FusionVectorMultiplyScalar(gyroscope, offset->filterCoefficient));

    return gyroscope;
}

void Madgwick_updateIMU(float gx, float gy, float gz, float ax, float ay, float az) {
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

    gx *= 0.0174533f;
    gy *= 0.0174533f;
    gz *= 0.0174533f;

    qDot1 = 0.5f * (-madgwickParam.q1 * gx - madgwickParam.q2 * gy - madgwickParam.q3 * gz);
    qDot2 = 0.5f * (madgwickParam.q0 * gx + madgwickParam.q2 * gz - madgwickParam.q3 * gy);
    qDot3 = 0.5f * (madgwickParam.q0 * gy - madgwickParam.q1 * gz + madgwickParam.q3 * gx);
    qDot4 = 0.5f * (madgwickParam.q0 * gz + madgwickParam.q1 * gy - madgwickParam.q2 * gx);

    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        recipNorm = FusionFastInverseSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        _2q0 = 2.0f * madgwickParam.q0;
        _2q1 = 2.0f * madgwickParam.q1;
        _2q2 = 2.0f * madgwickParam.q2;
        _2q3 = 2.0f * madgwickParam.q3;
        _4q0 = 4.0f * madgwickParam.q0;
        _4q1 = 4.0f * madgwickParam.q1;
        _4q2 = 4.0f * madgwickParam.q2;
        _8q1 = 8.0f * madgwickParam.q1;
        _8q2 = 8.0f * madgwickParam.q2;
        q0q0 = madgwickParam.q0 * madgwickParam.q0;
        q1q1 = madgwickParam.q1 * madgwickParam.q1;
        q2q2 = madgwickParam.q2 * madgwickParam.q2;
        q3q3 = madgwickParam.q3 * madgwickParam.q3;

        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * madgwickParam.q1 - _2q0 * ay - _4q1 +
             _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        s2 = 4.0f * q0q0 * madgwickParam.q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 +
             _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        s3 = 4.0f * q1q1 * madgwickParam.q3 - _2q1 * ax + 4.0f * q2q2 * madgwickParam.q3 - _2q2 * ay;
        recipNorm = FusionFastInverseSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        qDot1 -= madgwickParam.beta * s0;
        qDot2 -= madgwickParam.beta * s1;
        qDot3 -= madgwickParam.beta * s2;
        qDot4 -= madgwickParam.beta * s3;
    }

    madgwickParam.q0 += qDot1 * madgwickParam.invSampleFreq;
    madgwickParam.q1 += qDot2 * madgwickParam.invSampleFreq;
    madgwickParam.q2 += qDot3 * madgwickParam.invSampleFreq;
    madgwickParam.q3 += qDot4 * madgwickParam.invSampleFreq;

    recipNorm = FusionFastInverseSqrt(
        madgwickParam.q0 * madgwickParam.q0 +
        madgwickParam.q1 * madgwickParam.q1 +
        madgwickParam.q2 * madgwickParam.q2 +
        madgwickParam.q3 * madgwickParam.q3);

    madgwickParam.q0 *= recipNorm;
    madgwickParam.q1 *= recipNorm;
    madgwickParam.q2 *= recipNorm;
    madgwickParam.q3 *= recipNorm;
}

#ifdef __cplusplus
}
#endif

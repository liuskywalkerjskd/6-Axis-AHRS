/**
 * @file Tactical_Fusion.cpp
 * @brief Tactical IMU fusion implementation.
 */

#include "Tactical_Fusion.h"
#include <math.h>
#include <string.h>

#define G_CONST 9.80665f
#define SQRT2_F 1.41421356237f

static float ClampFloat(const float value, const float minValue, const float maxValue) {
    if (value < minValue) return minValue;
    if (value > maxValue) return maxValue;
    return value;
}

static void DigitalFilterInitHighPass(DigitalFilter *const filter, const float cutoffHz, const float sampleRate) {
    if (filter == NULL) {
        return;
    }

    filter->x1 = 0.0f;
    filter->x2 = 0.0f;
    filter->y1 = 0.0f;
    filter->y2 = 0.0f;

    if ((cutoffHz <= 0.0f) || (sampleRate <= 0.0f)) {
        filter->b0 = 1.0f;
        filter->b1 = 0.0f;
        filter->b2 = 0.0f;
        filter->a1 = 0.0f;
        filter->a2 = 0.0f;
        return;
    }

    const float k = tanf((float) M_PI * cutoffHz / sampleRate);
    const float norm = 1.0f / (1.0f + SQRT2_F * k + k * k);

    filter->b0 = 1.0f * norm;
    filter->b1 = -2.0f * norm;
    filter->b2 = 1.0f * norm;
    filter->a1 = 2.0f * (k * k - 1.0f) * norm;
    filter->a2 = (1.0f - SQRT2_F * k + k * k) * norm;
}

static float DigitalFilterApply(DigitalFilter *const filter, const float input) {
    const float output = filter->b0 * input +
                         filter->b1 * filter->x1 +
                         filter->b2 * filter->x2 -
                         filter->a1 * filter->y1 -
                         filter->a2 * filter->y2;

    filter->x2 = filter->x1;
    filter->x1 = input;
    filter->y2 = filter->y1;
    filter->y1 = output;

    return output;
}

static FusionVector GravityBody(const FusionQuaternion q, const FusionConvention convention) {
    const float qw = q.element.w;
    const float qx = q.element.x;
    const float qy = q.element.y;
    const float qz = q.element.z;

    FusionVector halfGravity;
    if (convention == FusionConventionNed) {
        halfGravity.axis.x = qw * qy - qx * qz;
        halfGravity.axis.y = -(qy * qz + qw * qx);
        halfGravity.axis.z = 0.5f - qw * qw - qz * qz;
    } else {
        halfGravity.axis.x = qx * qz - qw * qy;
        halfGravity.axis.y = qy * qz + qw * qx;
        halfGravity.axis.z = qw * qw - 0.5f + qz * qz;
    }

    return FusionVectorMultiplyScalar(halfGravity, 2.0f);
}

static void UpdateMeanVar(const float value, const float alpha, float *const mean, float *const var) {
    const float delta = value - *mean;
    *mean += alpha * delta;
    *var += alpha * (delta * delta - *var);
}

static void UpdateNoiseStats(TacticalSystem *const sys, const FusionVector acc, const FusionVector gyro, const float dt) {
    const float tau = sys->params.noiseTau;
    const float alpha = (tau <= 0.0f) ? 1.0f : (dt / (tau + dt));

    UpdateMeanVar(acc.axis.x, alpha, &sys->noise.accMean.axis.x, &sys->noise.accVar.axis.x);
    UpdateMeanVar(acc.axis.y, alpha, &sys->noise.accMean.axis.y, &sys->noise.accVar.axis.y);
    UpdateMeanVar(acc.axis.z, alpha, &sys->noise.accMean.axis.z, &sys->noise.accVar.axis.z);

    UpdateMeanVar(gyro.axis.x, alpha, &sys->noise.gyroMean.axis.x, &sys->noise.gyroVar.axis.x);
    UpdateMeanVar(gyro.axis.y, alpha, &sys->noise.gyroMean.axis.y, &sys->noise.gyroVar.axis.y);
    UpdateMeanVar(gyro.axis.z, alpha, &sys->noise.gyroMean.axis.z, &sys->noise.gyroVar.axis.z);

    const float accMag = FusionVectorMagnitude(acc);
    const float gyroMag = FusionVectorMagnitude(gyro);

    UpdateMeanVar(accMag, alpha, &sys->noise.accMagMean, &sys->noise.accMagVar);
    UpdateMeanVar(gyroMag, alpha, &sys->noise.gyroMagMean, &sys->noise.gyroMagVar);

    const float accVarMag = sys->noise.accVar.axis.x + sys->noise.accVar.axis.y + sys->noise.accVar.axis.z;
    const float gyroVarMag = sys->noise.gyroVar.axis.x + sys->noise.gyroVar.axis.y + sys->noise.gyroVar.axis.z;

    sys->noise.accNoise = sqrtf(fmaxf(accVarMag, 0.0f));
    sys->noise.gyroNoise = sqrtf(fmaxf(gyroVarMag, 0.0f));
}

static TacticalMotionState AnalyzeMotion(TacticalSystem *const sys, const FusionVector acc, const FusionVector gyro) {
    const float accMag = FusionVectorMagnitude(acc);
    const float gyroMag = FusionVectorMagnitude(gyro);
    const float accErr = fabsf(accMag - 1.0f);

    if ((accErr < sys->params.accStaticThreshold) &&
        (gyroMag < sys->params.gyroStaticThreshold) &&
        (sys->noise.accMagVar < sys->params.accVarStatic) &&
        (sys->noise.gyroMagVar < sys->params.gyroVarStatic)) {
        return TACTICAL_MOTION_STATIC;
    }

    if ((accErr < sys->params.accStableThreshold) &&
        (gyroMag < sys->params.gyroStableThreshold)) {
        return TACTICAL_MOTION_STABLE;
    }

    return TACTICAL_MOTION_DYNAMIC;
}

static bool Mat3Inverse(const float m[3][3], float inv[3][3]) {
    const float det =
        m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
        m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
        m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);

    if (fabsf(det) < 1e-9f) {
        return false;
    }

    const float invDet = 1.0f / det;

    inv[0][0] = (m[1][1] * m[2][2] - m[1][2] * m[2][1]) * invDet;
    inv[0][1] = (m[0][2] * m[2][1] - m[0][1] * m[2][2]) * invDet;
    inv[0][2] = (m[0][1] * m[1][2] - m[0][2] * m[1][1]) * invDet;

    inv[1][0] = (m[1][2] * m[2][0] - m[1][0] * m[2][2]) * invDet;
    inv[1][1] = (m[0][0] * m[2][2] - m[0][2] * m[2][0]) * invDet;
    inv[1][2] = (m[0][2] * m[1][0] - m[0][0] * m[1][2]) * invDet;

    inv[2][0] = (m[1][0] * m[2][1] - m[1][1] * m[2][0]) * invDet;
    inv[2][1] = (m[0][1] * m[2][0] - m[0][0] * m[2][1]) * invDet;
    inv[2][2] = (m[0][0] * m[1][1] - m[0][1] * m[1][0]) * invDet;

    return true;
}

static void ComputeAdaptiveNoise(const TacticalSystem *const sys, const float accError, float *const qAngle,
                                 float *const qBias, float *const rAcc) {
    float accNoise = sys->noise.accNoise;
    float gyroNoise = sys->noise.gyroNoise;

    if (accNoise <= 0.0f) {
        accNoise = 0.01f;
    }
    if (gyroNoise <= 0.0f) {
        gyroNoise = 0.05f;
    }

    const float gyroNoiseRad = FusionDegreesToRadians(gyroNoise);
    const float gyroNoiseVar = gyroNoiseRad * gyroNoiseRad;
    const float accNoiseVar = accNoise * accNoise;

    float qAngleLocal = sys->params.processNoiseAngle + sys->params.gyroNoiseScale * gyroNoiseVar;
    float qBiasLocal = sys->params.processNoiseBias + sys->params.biasNoiseScale * gyroNoiseVar;
    float rAccLocal = sys->params.measureNoiseAcc + sys->params.accNoiseScale * accNoiseVar +
                      sys->params.accErrorScale * accError * accError;

    if (sys->motionState == TACTICAL_MOTION_STABLE) {
        rAccLocal *= 3.0f;
    } else if (sys->motionState == TACTICAL_MOTION_DYNAMIC) {
        rAccLocal *= 12.0f;
    }

    qAngleLocal = ClampFloat(qAngleLocal, 1e-9f, 1.0f);
    qBiasLocal = ClampFloat(qBiasLocal, 1e-12f, 1.0f);
    rAccLocal = ClampFloat(rAccLocal, 1e-6f, 1.0f);

    *qAngle = qAngleLocal;
    *qBias = qBiasLocal;
    *rAcc = rAccLocal;
}

static void EkfPredict(TacticalSystem *const sys, const FusionVector gyro, const float dt) {
    const FusionVector gyroCorrected = FusionVectorSubtract(gyro, sys->gyroBias);
    const FusionVector omega = FusionVectorMultiplyScalar(gyroCorrected, FusionDegreesToRadians(1.0f));

    const FusionVector delta = FusionVectorMultiplyScalar(omega, 0.5f * dt);
    sys->quaternion = FusionQuaternionAdd(sys->quaternion,
                                          FusionQuaternionMultiplyVector(sys->quaternion, delta));
    sys->quaternion = FusionQuaternionNormalise(sys->quaternion);

    float F[6][6] = {0};
    for (int i = 0; i < 6; ++i) {
        F[i][i] = 1.0f;
    }

    const float wx = omega.axis.x;
    const float wy = omega.axis.y;
    const float wz = omega.axis.z;

    const float skew[3][3] = {
        {0.0f, -wz, wy},
        {wz, 0.0f, -wx},
        {-wy, wx, 0.0f}
    };

    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            F[r][c] -= skew[r][c] * dt;
        }
        F[r][r + 3] -= dt;
    }

    const float accError = fabsf(sys->accMagnitude - 1.0f);
    float qAngle = 0.0f;
    float qBias = 0.0f;
    float rAccUnused = 0.0f;
    ComputeAdaptiveNoise(sys, accError, &qAngle, &qBias, &rAccUnused);

    float FP[6][6] = {0};
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) {
            float sum = 0.0f;
            for (int k = 0; k < 6; ++k) {
                sum += F[i][k] * sys->P[k][j];
            }
            FP[i][j] = sum;
        }
    }

    float FPFt[6][6] = {0};
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) {
            float sum = 0.0f;
            for (int k = 0; k < 6; ++k) {
                sum += FP[i][k] * F[j][k];
            }
            FPFt[i][j] = sum;
        }
    }

    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) {
            sys->P[i][j] = FPFt[i][j];
        }
    }

    for (int i = 0; i < 3; ++i) {
        sys->P[i][i] += qAngle * dt;
    }
    for (int i = 3; i < 6; ++i) {
        sys->P[i][i] += qBias * dt;
    }
}

static void EkfUpdateAccel(TacticalSystem *const sys, const FusionVector acc) {
    const float accMagSq = FusionVectorMagnitudeSquared(acc);
    if (accMagSq < 1e-6f) {
        return;
    }

    const float accMag = sqrtf(accMagSq);
    FusionVector accNorm = FusionVectorMultiplyScalar(acc, 1.0f / accMag);

    FusionVector gravity = GravityBody(sys->quaternion, sys->convention);
    if (FusionVectorMagnitudeSquared(gravity) > 0.0f) {
        gravity = FusionVectorNormalise(gravity);
    }

    const FusionVector error = FusionVectorCrossProduct(accNorm, gravity);

    const float accError = fabsf(sys->accMagnitude - 1.0f);
    float qAngle = 0.0f;
    float qBias = 0.0f;
    float rAcc = 0.0f;
    ComputeAdaptiveNoise(sys, accError, &qAngle, &qBias, &rAcc);
    (void)qAngle;
    (void)qBias;

    float P11[3][3];
    float P21[3][3];

    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            P11[r][c] = sys->P[r][c];
            P21[r][c] = sys->P[r + 3][c];
        }
    }

    float S[3][3];
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            S[r][c] = P11[r][c];
        }
        S[r][r] += rAcc;
    }

    float SInv[3][3];
    if (!Mat3Inverse(S, SInv)) {
        return;
    }

    float K1[3][3] = {0};
    float K2[3][3] = {0};

    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            float sum1 = 0.0f;
            float sum2 = 0.0f;
            for (int k = 0; k < 3; ++k) {
                sum1 += P11[r][k] * SInv[k][c];
                sum2 += P21[r][k] * SInv[k][c];
            }
            K1[r][c] = sum1;
            K2[r][c] = sum2;
        }
    }

    FusionVector deltaTheta;
    deltaTheta.axis.x = K1[0][0] * error.axis.x + K1[0][1] * error.axis.y + K1[0][2] * error.axis.z;
    deltaTheta.axis.y = K1[1][0] * error.axis.x + K1[1][1] * error.axis.y + K1[1][2] * error.axis.z;
    deltaTheta.axis.z = K1[2][0] * error.axis.x + K1[2][1] * error.axis.y + K1[2][2] * error.axis.z;

    FusionVector deltaBias;
    deltaBias.axis.x = K2[0][0] * error.axis.x + K2[0][1] * error.axis.y + K2[0][2] * error.axis.z;
    deltaBias.axis.y = K2[1][0] * error.axis.x + K2[1][1] * error.axis.y + K2[1][2] * error.axis.z;
    deltaBias.axis.z = K2[2][0] * error.axis.x + K2[2][1] * error.axis.y + K2[2][2] * error.axis.z;

    const FusionVector correction = FusionVectorMultiplyScalar(deltaTheta, 0.5f);
    sys->quaternion = FusionQuaternionAdd(sys->quaternion,
                                          FusionQuaternionMultiplyVector(sys->quaternion, correction));
    sys->quaternion = FusionQuaternionNormalise(sys->quaternion);

    const FusionVector deltaBiasDeg = FusionVectorMultiplyScalar(deltaBias, FusionRadiansToDegrees(1.0f));
    sys->gyroBias = FusionVectorAdd(sys->gyroBias, deltaBiasDeg);

    float K[6][3] = {0};
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            K[r][c] = K1[r][c];
            K[r + 3][c] = K2[r][c];
        }
    }

    float I_KH[6][6] = {0};
    for (int i = 0; i < 6; ++i) {
        I_KH[i][i] = 1.0f;
    }
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            I_KH[r][c] -= K1[r][c];
            I_KH[r + 3][c] -= K2[r][c];
        }
    }

    float temp[6][6] = {0};
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) {
            float sum = 0.0f;
            for (int k = 0; k < 6; ++k) {
                sum += I_KH[i][k] * sys->P[k][j];
            }
            temp[i][j] = sum;
        }
    }

    float Pnew[6][6] = {0};
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) {
            float sum = 0.0f;
            for (int k = 0; k < 6; ++k) {
                sum += temp[i][k] * I_KH[j][k];
            }
            Pnew[i][j] = sum;
        }
    }

    float KKt[6][6] = {0};
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) {
            float sum = 0.0f;
            for (int k = 0; k < 3; ++k) {
                sum += K[i][k] * K[j][k];
            }
            KKt[i][j] = sum;
        }
    }

    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) {
            sys->P[i][j] = Pnew[i][j] + rAcc * KKt[i][j];
        }
    }
}

void Tactical_Init(TacticalSystem *const sys, const float sampleRate, const float heaveCutoffFreq) {
    memset(sys, 0, sizeof(TacticalSystem));

    sys->samplePeriod = (sampleRate > 0.0f) ? (1.0f / sampleRate) : 0.0f;
    sys->convention = FusionConventionNwu;
    sys->quaternion = FUSION_IDENTITY_QUATERNION;
    sys->motionState = TACTICAL_MOTION_STATIC;

    sys->params.processNoiseAngle = 1.0e-5f;
    sys->params.processNoiseBias = 1.0e-7f;
    sys->params.measureNoiseAcc = 1.0e-2f;
    sys->params.accNoiseScale = 2.0f;
    sys->params.gyroNoiseScale = 1.0f;
    sys->params.biasNoiseScale = 0.1f;
    sys->params.accErrorScale = 2.0f;
    sys->params.noiseTau = 1.0f;
    sys->params.accStaticThreshold = 0.02f;
    sys->params.gyroStaticThreshold = 0.5f;
    sys->params.accStableThreshold = 0.15f;
    sys->params.gyroStableThreshold = 50.0f;
    sys->params.accVarStatic = 0.0004f;
    sys->params.gyroVarStatic = 0.25f;
    sys->params.heaveCutoffFreq = heaveCutoffFreq;
    sys->params.verticalBiasAlpha = 0.01f;
    sys->params.verticalAccLp = 0.2f;
    sys->params.gyroBiasAlpha = 0.001f;

    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) {
            sys->P[i][j] = 0.0f;
        }
    }
    sys->P[0][0] = 1.0e-3f;
    sys->P[1][1] = 1.0e-3f;
    sys->P[2][2] = 1.0e-3f;
    sys->P[3][3] = 1.0e-2f;
    sys->P[4][4] = 1.0e-2f;
    sys->P[5][5] = 1.0e-2f;

    DigitalFilterInitHighPass(&sys->hpFilterVel, heaveCutoffFreq, sampleRate);
    DigitalFilterInitHighPass(&sys->hpFilterPos, heaveCutoffFreq, sampleRate);
}

FusionVector Tactical_GetCalibratedGyro(const TacticalSystem *const sys, const FusionVector rawGyro) {
    return FusionVectorSubtract(rawGyro, sys->gyroBias);
}

void Tactical_Update(TacticalSystem *const sys, const FusionVector gyro, const FusionVector acc) {
    const float dt = sys->samplePeriod;
    if (dt <= 0.0f) {
        return;
    }

    const FusionVector calibGyro = Tactical_GetCalibratedGyro(sys, gyro);

    sys->accMagnitude = FusionVectorMagnitude(acc);
    UpdateNoiseStats(sys, acc, calibGyro, dt);
    sys->motionState = AnalyzeMotion(sys, acc, calibGyro);

    EkfPredict(sys, gyro, dt);
    EkfUpdateAccel(sys, acc);

    if (sys->motionState == TACTICAL_MOTION_STATIC) {
        const float alpha = ClampFloat(sys->params.gyroBiasAlpha, 0.0f, 1.0f);
        sys->gyroBias.axis.x += (gyro.axis.x - sys->gyroBias.axis.x) * alpha;
        sys->gyroBias.axis.y += (gyro.axis.y - sys->gyroBias.axis.y) * alpha;
        sys->gyroBias.axis.z += (gyro.axis.z - sys->gyroBias.axis.z) * alpha;
    }

    FusionVector earthAcc = Tactical_GetEarthAcceleration(sys, acc);
    float verticalAcc = earthAcc.axis.z;

    if (sys->motionState != TACTICAL_MOTION_DYNAMIC) {
        const float beta = ClampFloat(sys->params.verticalBiasAlpha, 0.0f, 1.0f);
        sys->verticalAccBias += (verticalAcc - sys->verticalAccBias) * beta;
    }
    verticalAcc -= sys->verticalAccBias;

    const float accAlpha = ClampFloat(sys->params.verticalAccLp, 0.0f, 1.0f);
    sys->verticalAccFiltered += (verticalAcc - sys->verticalAccFiltered) * accAlpha;

    const float azMss = sys->verticalAccFiltered * G_CONST;
    sys->heaveVelocity += azMss * dt;
    sys->heaveVelocity = DigitalFilterApply(&sys->hpFilterVel, sys->heaveVelocity);

    sys->heavePosition += sys->heaveVelocity * dt;
    sys->heavePosition = DigitalFilterApply(&sys->hpFilterPos, sys->heavePosition);
}

FusionVector Tactical_GetEarthAcceleration(const TacticalSystem *const sys, const FusionVector acc) {
    const FusionQuaternion q = sys->quaternion;
    const float qw = q.element.w;
    const float qx = q.element.x;
    const float qy = q.element.y;
    const float qz = q.element.z;

    const float ax = acc.axis.x;
    const float ay = acc.axis.y;
    const float az = acc.axis.z;

    FusionVector earthAcc;
    earthAcc.axis.x = (qw * qw + qx * qx - qy * qy - qz * qz) * ax +
                      2.0f * (qx * qy - qw * qz) * ay +
                      2.0f * (qx * qz + qw * qy) * az;

    earthAcc.axis.y = 2.0f * (qx * qy + qw * qz) * ax +
                      (qw * qw - qx * qx + qy * qy - qz * qz) * ay +
                      2.0f * (qy * qz - qw * qx) * az;

    earthAcc.axis.z = 2.0f * (qx * qz - qw * qy) * ax +
                      2.0f * (qy * qz + qw * qx) * ay +
                      (qw * qw - qx * qx - qy * qy + qz * qz) * az;

    if (sys->convention == FusionConventionNed) {
        earthAcc.axis.z += 1.0f;
    } else {
        earthAcc.axis.z -= 1.0f;
    }

    return earthAcc;
}

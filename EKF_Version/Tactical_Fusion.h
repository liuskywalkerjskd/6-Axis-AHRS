/**
 * @file Tactical_Fusion.h
 * @brief Tactical IMU fusion (adaptive EKF + noise analysis + heave + motion).
 */

#ifndef TACTICAL_FUSION_H
#define TACTICAL_FUSION_H

#ifdef __cplusplus
extern "C" {
#endif

#include "Fusion_AHRS.h"
#include <stdbool.h>

typedef enum {
    TACTICAL_MOTION_STATIC,
    TACTICAL_MOTION_STABLE,
    TACTICAL_MOTION_DYNAMIC,
} TacticalMotionState;

typedef struct {
    float b0;
    float b1;
    float b2;
    float a1;
    float a2;
    float x1;
    float x2;
    float y1;
    float y2;
} DigitalFilter;

typedef struct {
    FusionVector accMean;
    FusionVector accVar;
    FusionVector gyroMean;
    FusionVector gyroVar;
    float accMagMean;
    float accMagVar;
    float gyroMagMean;
    float gyroMagVar;
    float accNoise;
    float gyroNoise;
} TacticalNoiseStats;

typedef struct {
    FusionQuaternion quaternion;
    FusionVector gyroBias;
    FusionVector velocity;
    float heavePosition;
    float heaveVelocity;

    float P[6][6];

    float samplePeriod;
    FusionConvention convention;
    TacticalMotionState motionState;
    float accMagnitude;

    DigitalFilter hpFilterVel;
    DigitalFilter hpFilterPos;
    float verticalAccBias;
    float verticalAccFiltered;

    TacticalNoiseStats noise;

    struct {
        float processNoiseAngle;
        float processNoiseBias;
        float measureNoiseAcc;
        float accNoiseScale;
        float gyroNoiseScale;
        float biasNoiseScale;
        float accErrorScale;
        float noiseTau;
        float accStaticThreshold;
        float gyroStaticThreshold;
        float accStableThreshold;
        float gyroStableThreshold;
        float accVarStatic;
        float gyroVarStatic;
        float heaveCutoffFreq;
        float verticalBiasAlpha;
        float verticalAccLp;
        float gyroBiasAlpha;
    } params;
} TacticalSystem;

void Tactical_Init(TacticalSystem *const sys, float sampleRate, float heaveCutoffFreq);
void Tactical_Update(TacticalSystem *const sys, FusionVector gyro, FusionVector acc);
FusionVector Tactical_GetCalibratedGyro(const TacticalSystem *const sys, FusionVector rawGyro);
FusionVector Tactical_GetEarthAcceleration(const TacticalSystem *const sys, FusionVector acc);

#ifdef __cplusplus
}
#endif

#endif // TACTICAL_FUSION_H

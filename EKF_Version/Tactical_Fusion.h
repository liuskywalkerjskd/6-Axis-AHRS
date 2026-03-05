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

typedef enum {
    TACTICAL_IMPACT_NONE,
    TACTICAL_IMPACT_DETECTED,
    TACTICAL_IMPACT_RECOVERING
} TacticalImpactState;

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

    TacticalImpactState impactState;    // Current impact state
    float impactRecoveryTimer;         // Timer for impact recovery
    float accWeight;                  // Current accelerometer weight (adapts based on motion/impact)

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

        // Impact detection parameters
        float impactAccThreshold;       // Acceleration change threshold for impact detection (g)
        float impactGyroThreshold;      // Gyro rate change threshold for impact detection (deg/s)
        float impactRecoveryGain;       // Gain during recovery phase
        float impactRecoveryDuration;   // Recovery duration in seconds
        float accWeightNormal;          // Normal accelerometer weight
        float accWeightImpact;         // Accelerometer weight during impact
        float prevAccMag;              // Previous acceleration magnitude for delta detection
        float prevGyroMag;             // Previous gyro magnitude for delta detection

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

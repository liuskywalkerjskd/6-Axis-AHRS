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

    // Linear acceleration compensation
    FusionVector linearAccBody;         // Estimated linear acceleration in body frame
    float linearAccMagnitude;          // Magnitude of linear acceleration
    bool isLinearMotion;              // Flag indicating linear motion detected

    // Direction-based translational acceleration detection
    float accDirectionError;           // Angle between measured acc and predicted gravity (rad)

    // Paddling / periodic vibration detection
    float accVarWindow;                // Short-window accel magnitude variance
    float accMeanWindow;               // Short-window accel magnitude mean
    float accVarWindowAlpha;           // EMA alpha for windowed variance
    bool isPaddling;                   // Periodic vibration detected

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

        // Linear motion (translation) detection parameters
        float linearAccThreshold;         // Threshold for detecting linear acceleration (g)
        float linearMotionDecay;         // Decay rate for linear acceleration estimation
        float accCompensationEnabled;    // Enable acceleration compensation

        // Direction-based detection parameters
        float accDirectionThreshold;     // Direction error threshold (rad) for translational acc
        float accDirectionScale;         // R multiplier scale for direction error

        // Paddling / periodic vibration parameters
        float paddlingVarThreshold;      // Variance threshold to detect periodic vibration
        float paddlingMeanTolerance;     // How close mean must be to 1g to qualify as paddling
        float paddlingRScale;            // R multiplier during paddling detection
        float paddlingWindowAlpha;       // EMA alpha for variance window (~0.5s window)

        // Sustained linear acceleration parameters
        float sustainedLinearAlpha;      // EMA alpha for residual-based sustained detection

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

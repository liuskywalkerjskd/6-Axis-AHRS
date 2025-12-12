/**
 * @file Fusion_AHRS.cpp
 * @author liuskywalkerjskd
 * @brief AHRS (Attitude and Heading Reference System) algorithm implementation based on accelerometer and gyroscope
 */

#include <float.h>
#include <math.h>
#include "Fusion_AHRS.h"

// Global parameter structure for Madgwick algorithm
MadgwickParam madgwickParam;

//------------------------------------------------------------------------------
// Constants Definition

/**
 * @brief Gain value used during initialization phase
 */
#define INITIAL_GAIN (10.0f)

/**
 * @brief Duration of initialization phase (seconds)
 */
#define INITIALISATION_PERIOD (3.0f)

/**
 * @brief Cutoff frequency for gyroscope bias correction (Hz)
 */
#define CUTOFF_FREQUENCY (0.02f)

/**
 * @brief Timeout duration for bias correction (seconds)
 */
#define TIMEOUT (5)

/**
 * @brief Threshold for bias correction (degrees/second)
 */
#define THRESHOLD (3.0f)

/**
 * @brief Flag to enable gradient descent compensation
 */
#define GRAD_DEC 1

//------------------------------------------------------------------------------
// Static Function Declarations

/**
 * @brief Calculate half of the gravity direction vector
 * @param ahrs Pointer to AHRS structure
 * @return Half gravity vector in the body frame
 */
static inline FusionVector HalfGravity(const FusionAhrs *const ahrs);

/**
 * @brief Calculate feedback vector from sensor and reference
 * @param sensor Sensor vector (normalized)
 * @param reference Reference vector (normalized)
 * @return Feedback vector for correction
 */
static inline FusionVector Feedback(const FusionVector sensor, const FusionVector reference);

/**
 * @brief Clamp integer value within specified range
 * @param value Input value to clamp
 * @param min Minimum allowed value
 * @param max Maximum allowed value
 * @return Clamped value within [min, max] range
 */
static inline int Clamp(const int value, const int min, const int max);

//------------------------------------------------------------------------------
// AHRS Algorithm Implementation

// Flag to enable/disable gradient descent fusion
int use_grad = 1;

/**
 * @brief Initialize AHRS algorithm structure
 * @param ahrs Pointer to AHRS algorithm structure
 */
void FusionAhrsInitialise(FusionAhrs *const ahrs) {
    const FusionAhrsSettings settings = {
            .convention = FusionConventionNwu,  // Default to NWU coordinate system
            .gain = 0.5f,                      // Default gain value
            .gyroscopeRange = 2000.0f,         // Default gyroscope range 2000��/s
            .accelerationRejection = 30.0f,    // Default acceleration rejection threshold 30��
            .recoveryTriggerPeriod = 0,        // Default recovery trigger period
    };
    FusionAhrsSetSettings(ahrs, &settings);
    FusionAhrsReset(ahrs);
}

/**
 * @brief Reset AHRS algorithm state to initial values
 * @param ahrs Pointer to AHRS algorithm structure
 */
void FusionAhrsReset(FusionAhrs *const ahrs) {
    ahrs->quaternion = FUSION_IDENTITY_QUATERNION;  // Reset to identity quaternion
    ahrs->accelerometer = FUSION_VECTOR_ZERO;       // Reset accelerometer data
    ahrs->angularRateRecovery = false;              // Reset angular rate recovery flag
    ahrs->halfAccelerometerFeedback = FUSION_VECTOR_ZERO; // Reset accelerometer feedback
    ahrs->accelerometerIgnored = false;             // Reset accelerometer ignore flag
    ahrs->accelerationRecoveryTrigger = 0;          // Reset acceleration recovery trigger
    ahrs->accelerationRecoveryTimeout = ahrs->settings.recoveryTriggerPeriod; // Reset timeout
    ahrs->rampedGain = INITIAL_GAIN; // Set initial ramped gain
    ahrs->initialising = true; // Set initializing flag
    ahrs->rampedGainStep = (INITIAL_GAIN - ahrs->settings.gain) / INITIALISATION_PERIOD; // Set ramping step

    madgwickParam.beta = BETA_DEF; // 0.1f
    madgwickParam.invSampleFreq = 1.0f / SAMPLE_FREQ_DEF; // 1.0 / 500.0
    madgwickParam.q0 = 1.0f;
    madgwickParam.q1 = 0.0f;
    madgwickParam.q2 = 0.0f;
    madgwickParam.q3 = 0.0f;
    
    // Initialize filtering and fusion related parameters to avoid issues with dynamicFusionPower calculation
    madgwickParam.minFusionPower = 0.01f; 
    madgwickParam.maxFusionPower = 0.5f;  
    madgwickParam.Fusion_Power = 0.1f;    
    madgwickParam.filterAlpha = 0.1f;     // Low-pass filter coefficient mustn't be 0
    
    // Filtered quaternion also needs initialization
    madgwickParam.fq0 = 1.0f; 
    madgwickParam.fq1 = 0.0f;
    madgwickParam.fq2 = 0.0f;
    madgwickParam.fq3 = 0.0f;
}

/**
 * @brief Set AHRS algorithm parameters
 * @param ahrs Pointer to AHRS algorithm structure
 * @param settings Pointer to settings structure
 */
void FusionAhrsSetSettings(FusionAhrs *const ahrs, const FusionAhrsSettings *const settings) {
    ahrs->settings.convention = settings->convention;
    ahrs->settings.gain = settings->gain;
    // Calculate effective gyroscope range (with 2% margin)
    ahrs->settings.gyroscopeRange = settings->gyroscopeRange == 0.0f ? FLT_MAX : 0.98f * settings->gyroscopeRange;
    // Calculate acceleration rejection threshold (convert to radians and compute half-angle sine squared)
    ahrs->settings.accelerationRejection = settings->accelerationRejection == 0.0f ? FLT_MAX : 
        powf(0.5f * sinf(FusionDegreesToRadians(settings->accelerationRejection)), 2);
    ahrs->settings.recoveryTriggerPeriod = settings->recoveryTriggerPeriod;
    ahrs->accelerationRecoveryTimeout = ahrs->settings.recoveryTriggerPeriod;
    
    // Disable acceleration rejection if gain is 0 or recovery period is 0
    if ((settings->gain == 0.0f) || (settings->recoveryTriggerPeriod == 0)) {
        ahrs->settings.accelerationRejection = FLT_MAX;
    }

    // Set ramped gain to target gain if not initializing
    if (ahrs->initialising == false) {
        ahrs->rampedGain = ahrs->settings.gain;
    }
    // Set ramped gain step for initialization phase
    ahrs->rampedGainStep = (INITIAL_GAIN - ahrs->settings.gain) / INITIALISATION_PERIOD;
}

/**
 * @brief Update AHRS algorithm state without magnetometer
 * @param ahrs Pointer to AHRS algorithm structure
 * @param gyroscope Gyroscope data (degrees/second)
 * @param accelerometer Accelerometer data (g)
 * @param deltaTime Time step (seconds)
 */
void FusionAhrsUpdateNoMagnetometer(FusionAhrs *const ahrs, const FusionVector gyroscope, 
                                   const FusionVector accelerometer, const float deltaTime) {
    // Call main update function
    FusionAhrsUpdate(ahrs, gyroscope, accelerometer, deltaTime);
}

/**
 * @brief Update AHRS algorithm state with gyroscope and accelerometer data
 * @param ahrs Pointer to AHRS algorithm structure
 * @param gyroscope Gyroscope data (degrees/second)
 * @param accelerometer Accelerometer data (g)
 * @param deltaTime Time step (seconds)
 */
void FusionAhrsUpdate(FusionAhrs *const ahrs, const FusionVector gyroscope, 
                     const FusionVector accelerometer, const float deltaTime) {
#define Q ahrs->quaternion.element

    // Store accelerometer data for later use
    ahrs->accelerometer = accelerometer;

    // Check if gyroscope data exceeds range and reinitialize if necessary
    if ((fabsf(gyroscope.axis.x) > ahrs->settings.gyroscopeRange) || 
        (fabsf(gyroscope.axis.y) > ahrs->settings.gyroscopeRange) || 
        (fabsf(gyroscope.axis.z) > ahrs->settings.gyroscopeRange)) {
        const FusionQuaternion quaternion = ahrs->quaternion;
        FusionAhrsReset(ahrs);
        ahrs->quaternion = quaternion;  // Preserve current attitude
        ahrs->angularRateRecovery = true; // Set angular rate recovery flag
    }

    // Handle initialization phase with gain ramping
    if (ahrs->initialising) {
        // Decrease ramped gain towards target gain
        ahrs->rampedGain -= ahrs->rampedGainStep * deltaTime;
        
        // Clamp ramped gain to target gain
        if (ahrs->rampedGain < ahrs->settings.gain) {
            ahrs->rampedGain = ahrs->settings.gain;
            ahrs->initialising = false;
        }
    }

    // Calculate algorithm-indicated gravity direction (half value)
    const FusionVector halfGravity = HalfGravity(ahrs);

    // Calculate accelerometer feedback
    FusionVector halfAccelerometerFeedback = FUSION_VECTOR_ZERO;
    ahrs->accelerometerIgnored = true;
    
    if (FusionVectorIsZero(accelerometer) == false) {
        // Calculate normalized accelerometer data feedback with gravity direction
        ahrs->halfAccelerometerFeedback = Feedback(FusionVectorNormalise(accelerometer), halfGravity);
        
        // Check if accelerometer data should be ignored
        if ((FusionVectorMagnitudeSquared(ahrs->halfAccelerometerFeedback) <= ahrs->settings.accelerationRejection)) {
            ahrs->accelerometerIgnored = false;
            ahrs->accelerationRecoveryTrigger -= 9; // Accelerate recovery
        } else {
            ahrs->accelerationRecoveryTrigger += 1; // Increment trigger count
        }
        
        // Check if recovery should be triggered
        if (ahrs->accelerationRecoveryTrigger > ahrs->accelerationRecoveryTimeout) {
            ahrs->accelerationRecoveryTimeout = 0;
            ahrs->accelerometerIgnored = false; // Force use of accelerometer
        } else {
            ahrs->accelerationRecoveryTimeout = ahrs->settings.recoveryTriggerPeriod;
        }
        
        // Clamp trigger count to valid range
        ahrs->accelerationRecoveryTrigger = Clamp(ahrs->accelerationRecoveryTrigger, 0, ahrs->settings.recoveryTriggerPeriod);
        
        // Use feedback if accelerometer is not ignored
        if (ahrs->accelerometerIgnored == false) {
            halfAccelerometerFeedback = ahrs->halfAccelerometerFeedback;
        }
    }

    // Convert gyroscope data to radians/second and scale by 0.5
    const FusionVector halfGyroscope = FusionVectorMultiplyScalar(gyroscope, FusionDegreesToRadians(0.5f));

    // Apply feedback to gyroscope data
    const FusionVector adjustedHalfGyroscope = FusionVectorAdd(halfGyroscope, 
        FusionVectorMultiplyScalar(halfAccelerometerFeedback, ahrs->rampedGain));

    // Integrate quaternion rate of change
    ahrs->quaternion = FusionQuaternionAdd(ahrs->quaternion, 
        FusionQuaternionMultiplyVector(ahrs->quaternion, 
            FusionVectorMultiplyScalar(adjustedHalfGyroscope, deltaTime)));

    // Normalize quaternion
    ahrs->quaternion = FusionQuaternionNormalise(ahrs->quaternion);
	
    if (use_grad) {
        // Use gradient descent method for fusion (increases noise but accelerates convergence)
        // Gradient descent algorithm to get raw q0-q3 values
        Madgwick_updateIMU(gyroscope.axis.x, gyroscope.axis.y, gyroscope.axis.z, 
                          accelerometer.axis.x, accelerometer.axis.y, accelerometer.axis.z);
        
        // Dot product between main algorithm quaternion and Madgwick quaternion
        float dot_product = ahrs->quaternion.element.w * madgwickParam.q0 +
                           ahrs->quaternion.element.x * madgwickParam.q1 +
                           ahrs->quaternion.element.y * madgwickParam.q2 +
                           ahrs->quaternion.element.z * madgwickParam.q3;
        
        // Calculate similarity error using 1.0f minus absolute value of dot product
        float similarity_error = 1.0f - fabsf(dot_product);

        // Dynamically calculate Fusion Power
        // When error is large, weight approaches maxFusionPower; when error is small, weight approaches minFusionPower, linear mapping
        float dynamicFusionPower = madgwickParam.minFusionPower + 
                                 similarity_error * (madgwickParam.maxFusionPower - madgwickParam.minFusionPower);
        // Prevent error from causing abnormal weight
        if (dynamicFusionPower > madgwickParam.maxFusionPower) {
            dynamicFusionPower = madgwickParam.maxFusionPower;
        }
            
        // Apply low-pass filter to Madgwick output
        // Formula: filtered_value = alpha * new_value + (1 - alpha) * last_filtered_value
        madgwickParam.fq0 = madgwickParam.filterAlpha * madgwickParam.q0 + (1.0f - madgwickParam.filterAlpha) * madgwickParam.fq0;
        madgwickParam.fq1 = madgwickParam.filterAlpha * madgwickParam.q1 + (1.0f - madgwickParam.filterAlpha) * madgwickParam.fq1;
        madgwickParam.fq2 = madgwickParam.filterAlpha * madgwickParam.q2 + (1.0f - madgwickParam.filterAlpha) * madgwickParam.fq2;
        madgwickParam.fq3 = madgwickParam.filterAlpha * madgwickParam.q3 + (1.0f - madgwickParam.filterAlpha) * madgwickParam.fq3;
        
        // Re-normalize after filtering to prevent accumulated errors
        float recipNorm = FusionFastInverseSqrt(
            madgwickParam.fq0 * madgwickParam.fq0 + 
            madgwickParam.fq1 * madgwickParam.fq1 + 
            madgwickParam.fq2 * madgwickParam.fq2 + 
            madgwickParam.fq3 * madgwickParam.fq3);
        madgwickParam.fq0 *= recipNorm;
        madgwickParam.fq1 *= recipNorm;
        madgwickParam.fq2 *= recipNorm;
        madgwickParam.fq3 *= recipNorm;

        // Use filtered quaternion for final fusion
        Q.w = (1 - dynamicFusionPower) * Q.w + dynamicFusionPower * madgwickParam.fq0;
        Q.x = (1 - dynamicFusionPower) * Q.x + dynamicFusionPower * madgwickParam.fq1;
        Q.y = (1 - dynamicFusionPower) * Q.y + dynamicFusionPower * madgwickParam.fq2;
        Q.z = (1 - dynamicFusionPower) * Q.z + dynamicFusionPower * madgwickParam.fq3;
        
        // Normalize quaternion
        ahrs->quaternion = FusionQuaternionNormalise(ahrs->quaternion);
    }
	
#undef Q
}

/**
 * @brief Calculate half of the gravity direction vector in body frame
 * @param ahrs Pointer to AHRS structure
 * @return Half gravity vector in body frame
 */
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

/**
 * @brief Calculate feedback vector from sensor and reference vectors
 * @param sensor Normalized sensor vector
 * @param reference Normalized reference vector
 * @return Feedback vector for correction
 */
static inline FusionVector Feedback(const FusionVector sensor, const FusionVector reference) {
    // If sensor and reference directions are opposite, normalize cross product result first
    if (FusionVectorDotProduct(sensor, reference) < 0.0f) {
        return FusionVectorNormalise(FusionVectorCrossProduct(sensor, reference));
    }
    return FusionVectorCrossProduct(sensor, reference);
}

/**
 * @brief Clamp integer value within specified range
 * @param value Input value to clamp
 * @param min Minimum allowed value
 * @param max Maximum allowed value
 * @return Clamped value within [min, max] range
 */
static inline int Clamp(const int value, const int min, const int max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

/**
 * @brief Get current attitude quaternion
 * @param ahrs Pointer to AHRS structure
 * @return Current attitude quaternion
 */
FusionQuaternion FusionAhrsGetQuaternion(const FusionAhrs *const ahrs) {
    return ahrs->quaternion;
}

/**
 * @brief Set current attitude quaternion
 * @param ahrs Pointer to AHRS structure
 * @param quaternion New attitude quaternion
 */
void FusionAhrsSetQuaternion(FusionAhrs *const ahrs, const FusionQuaternion quaternion) {
    ahrs->quaternion = quaternion;
}

/**
 * @brief Get gravity vector in body frame
 * @param ahrs Pointer to AHRS structure
 * @return Gravity vector in body frame
 */
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

/**
 * @brief Get linear acceleration (gravity removed)
 * @param ahrs Pointer to AHRS structure
 * @return Linear acceleration vector
 */
FusionVector FusionAhrsGetLinearAcceleration(const FusionAhrs *const ahrs) {
    switch (ahrs->settings.convention) {
        case FusionConventionNwu:
        case FusionConventionEnu: {
            return FusionVectorSubtract(ahrs->accelerometer, FusionAhrsGetGravity(ahrs));
        }
        case FusionConventionNed: {
            return FusionVectorAdd(ahrs->accelerometer, FusionAhrsGetGravity(ahrs));
        }
    }
    return FUSION_VECTOR_ZERO;
}

/**
 * @brief Get acceleration in Earth frame
 * @param ahrs Pointer to AHRS structure
 * @return Acceleration vector in Earth frame
 */
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
            accelerometer.axis.z -= 1.0f;  // Subtract gravity effect
            break;
        case FusionConventionNed:
            accelerometer.axis.z += 1.0f;  // Add gravity effect
            break;
    }
    return accelerometer;
#undef Q
#undef A
}

/**
 * @brief Get AHRS internal states
 * @param ahrs Pointer to AHRS structure
 * @return Internal states structure
 */
FusionAhrsInternalStates FusionAhrsGetInternalStates(const FusionAhrs *const ahrs) {
    const FusionAhrsInternalStates internalStates = {
            .accelerationError = FusionRadiansToDegrees(FusionAsin(2.0f * FusionVectorMagnitude(ahrs->halfAccelerometerFeedback))),
            .accelerometerIgnored = ahrs->accelerometerIgnored,
            .accelerationRecoveryTrigger = ahrs->settings.recoveryTriggerPeriod == 0 ? 0.0f : 
                (float) ahrs->accelerationRecoveryTrigger / (float) ahrs->settings.recoveryTriggerPeriod,
    };
    return internalStates;
}

/**
 * @brief Get AHRS status flags
 * @param ahrs Pointer to AHRS structure
 * @return Status flags structure
 */
FusionAhrsFlags FusionAhrsGetFlags(const FusionAhrs *const ahrs) {
    const FusionAhrsFlags flags = {
            .angularRateRecovery = ahrs->angularRateRecovery,
            .accelerationRecovery = ahrs->accelerationRecoveryTrigger > ahrs->accelerationRecoveryTimeout,
    };
    return flags;
}

/**
 * @brief Set heading angle
 * @param ahrs Pointer to AHRS structure
 * @param heading Heading angle in degrees
 */
void FusionAhrsSetHeading(FusionAhrs *const ahrs, const float heading) {
#define Q ahrs->quaternion.element
    // Calculate current yaw angle
    const float yaw = atan2f(Q.w * Q.z + Q.x * Q.y, 0.5f - Q.y * Q.y - Q.z * Q.z);
    // Calculate half difference between current and target heading
    const float halfYawMinusHeading = 0.5f * (yaw - FusionDegreesToRadians(heading));
    // Create rotation quaternion around Z axis
    const FusionQuaternion rotation = {.element = {
            .w = cosf(halfYawMinusHeading),
            .x = 0.0f,
            .y = 0.0f,
            .z = -1.0f * sinf(halfYawMinusHeading),
    }};
    // Apply rotation
    ahrs->quaternion = FusionQuaternionMultiply(rotation, ahrs->quaternion);
#undef Q
}

//------------------------------------------------------------------------------
// Gyroscope Bias Correction Functions

/**
 * @brief Initialize gyroscope bias correction
 * @param offset Pointer to offset structure
 * @param sampleRate Sampling rate (Hz)
 */
void FusionOffsetInitialise(FusionOffset *const offset, const unsigned int sampleRate) {
    // Calculate filter coefficient
    offset->filterCoefficient = 2.0f * (float) M_PI * CUTOFF_FREQUENCY * (1.0f / (float) sampleRate);
    offset->timeout = TIMEOUT * sampleRate;  // Calculate timeout count
    offset->timer = 0;                       // Reset timer
    offset->gyroscopeOffset = FUSION_VECTOR_ZERO; // Reset bias estimate
}

/**
 * @brief Update gyroscope bias correction
 * @param offset Pointer to offset structure
 * @param gyroscope Raw gyroscope data
 * @return Bias-corrected gyroscope data
 */
FusionVector FusionOffsetUpdate(FusionOffset *const offset, FusionVector gyroscope) {
    // Subtract current bias estimate first
    gyroscope = FusionVectorSubtract(gyroscope, offset->gyroscopeOffset);
    
    // Reset timer if gyroscope data exceeds threshold
    if ((fabsf(gyroscope.axis.x) > THRESHOLD) || 
       (fabsf(gyroscope.axis.y) > THRESHOLD) || 
       (fabsf(gyroscope.axis.z) > THRESHOLD)) {
        offset->timer = 0;
        return gyroscope;
    }
    
    // Return directly if timer hasn't timed out
    if (offset->timer < offset->timeout) {
        offset->timer++;
        return gyroscope;
    }
    
    // Update bias estimate (low-pass filter)
    offset->gyroscopeOffset = FusionVectorAdd(offset->gyroscopeOffset, 
        FusionVectorMultiplyScalar(gyroscope, offset->filterCoefficient));
    
    return gyroscope;
}

//------------------------------------------------------------------------------
// Gradient Descent Attitude Computation

/**
 * @brief Update IMU using Madgwick gradient descent algorithm
 * @param gx Gyroscope X-axis reading (degrees/second)
 * @param gy Gyroscope Y-axis reading (degrees/second)
 * @param gz Gyroscope Z-axis reading (degrees/second)
 * @param ax Accelerometer X-axis reading (g)
 * @param ay Accelerometer Y-axis reading (g)
 * @param az Accelerometer Z-axis reading (g)
 */
void Madgwick_updateIMU(float gx, float gy, float gz, float ax, float ay, float az) {
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

    // Convert gyroscope units from degrees/second to radians/second
    gx *= 0.0174533f;
    gy *= 0.0174533f;
    gz *= 0.0174533f;

    // Calculate quaternion derivative from gyroscope readings
    qDot1 = 0.5f * (-madgwickParam.q1 * gx - madgwickParam.q2 * gy - madgwickParam.q3 * gz);
    qDot2 = 0.5f * (madgwickParam.q0 * gx + madgwickParam.q2 * gz - madgwickParam.q3 * gy);
    qDot3 = 0.5f * (madgwickParam.q0 * gy - madgwickParam.q1 * gz + madgwickParam.q3 * gx);
    qDot4 = 0.5f * (madgwickParam.q0 * gz + madgwickParam.q1 * gy - madgwickParam.q2 * gx);

    // Only calculate feedback when accelerometer readings are valid (avoid NaN during normalization)
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        // Normalize accelerometer measurements
        recipNorm = FusionFastInverseSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Auxiliary variables to avoid repeated calculations
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

        // Gradient descent algorithm correction step
        // s0-s3 are gradients of the objective function (error function)
        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * madgwickParam.q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        s2 = 4.0f * q0q0 * madgwickParam.q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        s3 = 4.0f * q1q1 * madgwickParam.q3 - _2q1 * ax + 4.0f * q2q2 * madgwickParam.q3 - _2q2 * ay;
        recipNorm = FusionFastInverseSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // Normalize gradient
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;
        
        // Calculate current gradient update
        float grad_step0 = madgwickParam.beta * s0;
        float grad_step1 = madgwickParam.beta * s1;
        float grad_step2 = madgwickParam.beta * s2;
        float grad_step3 = madgwickParam.beta * s3;

        // Combine momentum term, calculate final update
        // Formula: v_t = gamma * v_{t-1} + learning_rate * gradient
        madgwickParam.v_s0 = madgwickParam.momentum * madgwickParam.v_s0 + grad_step0;
        madgwickParam.v_s1 = madgwickParam.momentum * madgwickParam.v_s1 + grad_step1;
        madgwickParam.v_s2 = madgwickParam.momentum * madgwickParam.v_s2 + grad_step2;
        madgwickParam.v_s3 = madgwickParam.momentum * madgwickParam.v_s3 + grad_step3;

        // Apply momentum-based feedback step
        qDot1 -= madgwickParam.v_s0;
        qDot2 -= madgwickParam.v_s1;
        qDot3 -= madgwickParam.v_s2;
        qDot4 -= madgwickParam.v_s3;

        // Apply feedback step, feed error gradient back to quaternion derivative
        qDot1 -= madgwickParam.beta * s0;
        qDot2 -= madgwickParam.beta * s1;
        qDot3 -= madgwickParam.beta * s2;
        qDot4 -= madgwickParam.beta * s3;
    }

    // Integrate quaternion derivative to get new quaternion
    madgwickParam.q0 += qDot1 * madgwickParam.invSampleFreq;
    madgwickParam.q1 += qDot2 * madgwickParam.invSampleFreq;
    madgwickParam.q2 += qDot3 * madgwickParam.invSampleFreq;
    madgwickParam.q3 += qDot4 * madgwickParam.invSampleFreq;

    // Normalize the new quaternion
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
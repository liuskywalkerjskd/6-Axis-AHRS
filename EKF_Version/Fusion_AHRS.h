/**
 * @file Fusion_AHRS.h
 * @author liuskywalkerjskd
 * @brief AHRS (Attitude and Heading Reference System) algorithm implementation based on accelerometer and gyroscope
 */

#ifndef FUSION_H
#define FUSION_H

#ifdef __cplusplus
extern "C" {
#endif

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

//------------------------------------------------------------------------------
// Mathematical Library Definitions

/**
 * @brief 3D vector structure
 */
typedef union {
    float array[3];
    struct {
        float x;  // X component
        float y;  // Y component
        float z;  // Z component
    } axis;
} FusionVector;

/**
 * @brief Quaternion structure
 */
typedef union {
    float array[4];
    struct {
        float w;  // Real part
        float x;  // i imaginary part
        float y;  // j imaginary part
        float z;  // k imaginary part
    } element;
} FusionQuaternion;

/**
 * @brief 3x3 matrix (stored in row-major order)
 */
typedef union {
    float array[3][3];
    struct {
        float xx; float xy; float xz;
        float yx; float yy; float yz;
        float zx; float zy; float zz;
    } element;
} FusionMatrix;

/**
 * @brief Euler angles structure
 * @note Roll, pitch, yaw correspond to rotations around X, Y, Z axes respectively
 */
typedef union {
    float array[3];
    struct {
        float roll;   // Roll angle (rotation around X-axis)
        float pitch;  // Pitch angle (rotation around Y-axis)
        float yaw;    // Yaw angle (rotation around Z-axis)
    } angle;
} FusionEuler;

/**
 * @brief Zero vector constant
 */
#define FUSION_VECTOR_ZERO ((FusionVector){ .array = {0.0f, 0.0f, 0.0f} })

/**
 * @brief Ones vector constant
 */
#define FUSION_VECTOR_ONES ((FusionVector){ .array = {1.0f, 1.0f, 1.0f} })

/**
 * @brief Identity quaternion constant
 */
#define FUSION_IDENTITY_QUATERNION ((FusionQuaternion){ .array = {1.0f, 0.0f, 0.0f, 0.0f} })

/**
 * @brief Identity matrix constant
 */
#define FUSION_IDENTITY_MATRIX ((FusionMatrix){ .array = {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}} })

/**
 * @brief Zero Euler angles constant
 */
#define FUSION_EULER_ZERO ((FusionEuler){ .array = {0.0f, 0.0f, 0.0f} })

#ifndef M_PI
#define M_PI (3.14159265358979323846)
#endif

//------------------------------------------------------------------------------
// Mathematical Library Inline Functions

/**
 * @brief Convert degrees to radians
 * @param degrees Angle in degrees
 * @return Angle in radians
 */
static inline float FusionDegreesToRadians(const float degrees) {
    return degrees * ((float) M_PI / 180.0f);
}

/**
 * @brief Convert radians to degrees
 * @param radians Angle in radians
 * @return Angle in degrees
 */
static inline float FusionRadiansToDegrees(const float radians) {
    return radians * (180.0f / (float) M_PI);
}

/**
 * @brief Safe arcsine function
 * @param value Input value in range [-1, 1]
 * @return Arcsine result in range [-��/2, ��/2]
 */
static inline float FusionAsin(const float value) {
    if (value <= -1.0f) return (float) M_PI / -2.0f;
    if (value >= 1.0f) return (float) M_PI / 2.0f;
    return asinf(value);
}

/**
 * @brief Fast inverse square root calculation (approximate algorithm)
 * @param x Input value
 * @return Approximate 1/��x
 */
static inline float FusionFastInverseSqrt(const float x) {
    typedef union { float f; int32_t i; } Union32;
    Union32 union32 = {.f = x};
    union32.i = 0x5F1F1412 - (union32.i >> 1);
    return union32.f * (1.69000231f - 0.714158168f * x * union32.f * union32.f);
}

/**
 * @brief Check if vector is zero
 * @param vector Input vector
 * @return true if all components are zero, false otherwise
 */
static inline bool FusionVectorIsZero(const FusionVector vector) {
    return (vector.axis.x == 0.0f) && (vector.axis.y == 0.0f) && (vector.axis.z == 0.0f);
}

/**
 * @brief Vector addition
 * @param vectorA First vector
 * @param vectorB Second vector
 * @return Sum of vectors
 */
static inline FusionVector FusionVectorAdd(const FusionVector vectorA, const FusionVector vectorB) {
    return (FusionVector){.axis = {
        .x = vectorA.axis.x + vectorB.axis.x,
        .y = vectorA.axis.y + vectorB.axis.y,
        .z = vectorA.axis.z + vectorB.axis.z,
    }};
}

/**
 * @brief Vector subtraction
 * @param vectorA First vector
 * @param vectorB Second vector
 * @return Difference of vectors
 */
static inline FusionVector FusionVectorSubtract(const FusionVector vectorA, const FusionVector vectorB) {
    return (FusionVector){.axis = {
        .x = vectorA.axis.x - vectorB.axis.x,
        .y = vectorA.axis.y - vectorB.axis.y,
        .z = vectorA.axis.z - vectorB.axis.z,
    }};
}

/**
 * @brief Sum of vector components
 * @param vector Input vector
 * @return Sum of all components
 */
static inline float FusionVectorSum(const FusionVector vector) {
    return vector.axis.x + vector.axis.y + vector.axis.z;
}

/**
 * @brief Vector-scalar multiplication
 * @param vector Input vector
 * @param scalar Scalar multiplier
 * @return Scaled vector
 */
static inline FusionVector FusionVectorMultiplyScalar(const FusionVector vector, const float scalar) {
    return (FusionVector){.axis = {
        .x = vector.axis.x * scalar,
        .y = vector.axis.y * scalar,
        .z = vector.axis.z * scalar,
    }};
}

/**
 * @brief Hadamard product (element-wise multiplication) of two vectors
 * @param vectorA First vector
 * @param vectorB Second vector
 * @return Element-wise product
 */
static inline FusionVector FusionVectorHadamardProduct(const FusionVector vectorA, const FusionVector vectorB) {
    return (FusionVector){.axis = {
        .x = vectorA.axis.x * vectorB.axis.x,
        .y = vectorA.axis.y * vectorB.axis.y,
        .z = vectorA.axis.z * vectorB.axis.z,
    }};
}

/**
 * @brief Cross product of two vectors
 * @param vectorA First vector
 * @param vectorB Second vector
 * @return Cross product A �� B
 */
static inline FusionVector FusionVectorCrossProduct(const FusionVector vectorA, const FusionVector vectorB) {
    return (FusionVector){.axis = {
        .x = vectorA.axis.y * vectorB.axis.z - vectorA.axis.z * vectorB.axis.y,
        .y = vectorA.axis.z * vectorB.axis.x - vectorA.axis.x * vectorB.axis.z,
        .z = vectorA.axis.x * vectorB.axis.y - vectorA.axis.y * vectorB.axis.x,
    }};
}

/**
 * @brief Dot product of two vectors
 * @param vectorA First vector
 * @param vectorB Second vector
 * @return Dot product A �� B
 */
static inline float FusionVectorDotProduct(const FusionVector vectorA, const FusionVector vectorB) {
    return FusionVectorSum(FusionVectorHadamardProduct(vectorA, vectorB));
}

/**
 * @brief Squared magnitude of vector
 * @param vector Input vector
 * @return Squared magnitude ||v||?
 */
static inline float FusionVectorMagnitudeSquared(const FusionVector vector) {
    return FusionVectorSum(FusionVectorHadamardProduct(vector, vector));
}

/**
 * @brief Magnitude of vector
 * @param vector Input vector
 * @return Magnitude ||v||
 */
static inline float FusionVectorMagnitude(const FusionVector vector) {
    return sqrtf(FusionVectorMagnitudeSquared(vector));
}

/**
 * @brief Normalize vector
 * @param vector Input vector
 * @return Normalized vector with unit magnitude
 */
static inline FusionVector FusionVectorNormalise(const FusionVector vector) {
    const float magnitudeReciprocal = FusionFastInverseSqrt(FusionVectorMagnitudeSquared(vector));
    return FusionVectorMultiplyScalar(vector, magnitudeReciprocal);
}

/**
 * @brief Quaternion addition
 * @param quaternionA First quaternion
 * @param quaternionB Second quaternion
 * @return Sum of quaternions
 */
static inline FusionQuaternion FusionQuaternionAdd(const FusionQuaternion quaternionA, const FusionQuaternion quaternionB) {
    return (FusionQuaternion){.element = {
        .w = quaternionA.element.w + quaternionB.element.w,
        .x = quaternionA.element.x + quaternionB.element.x,
        .y = quaternionA.element.y + quaternionB.element.y,
        .z = quaternionA.element.z + quaternionB.element.z,
    }};
}

/**
 * @brief Quaternion multiplication
 * @param quaternionA First quaternion
 * @param quaternionB Second quaternion
 * @return Product of quaternions (A * B)
 */
static inline FusionQuaternion FusionQuaternionMultiply(const FusionQuaternion quaternionA, const FusionQuaternion quaternionB) {
    return (FusionQuaternion){.element = {
        .w = quaternionA.element.w * quaternionB.element.w - quaternionA.element.x * quaternionB.element.x - quaternionA.element.y * quaternionB.element.y - quaternionA.element.z * quaternionB.element.z,
        .x = quaternionA.element.w * quaternionB.element.x + quaternionA.element.x * quaternionB.element.w + quaternionA.element.y * quaternionB.element.z - quaternionA.element.z * quaternionB.element.y,
        .y = quaternionA.element.w * quaternionB.element.y - quaternionA.element.x * quaternionB.element.z + quaternionA.element.y * quaternionB.element.w + quaternionA.element.z * quaternionB.element.x,
        .z = quaternionA.element.w * quaternionB.element.z + quaternionA.element.x * quaternionB.element.y - quaternionA.element.y * quaternionB.element.x + quaternionA.element.z * quaternionB.element.w,
    }};
}

/**
 * @brief Quaternion-vector multiplication
 * @param quaternion Input quaternion
 * @param vector Input vector
 * @return Result of quaternion-vector multiplication
 */
static inline FusionQuaternion FusionQuaternionMultiplyVector(const FusionQuaternion quaternion, const FusionVector vector) {
    return (FusionQuaternion){.element = {
        .w = -quaternion.element.x * vector.axis.x - quaternion.element.y * vector.axis.y - quaternion.element.z * vector.axis.z,
        .x = quaternion.element.w * vector.axis.x + quaternion.element.y * vector.axis.z - quaternion.element.z * vector.axis.y,
        .y = quaternion.element.w * vector.axis.y - quaternion.element.x * vector.axis.z + quaternion.element.z * vector.axis.x,
        .z = quaternion.element.w * vector.axis.z + quaternion.element.x * vector.axis.y - quaternion.element.y * vector.axis.x,
    }};
}

/**
 * @brief Normalize quaternion
 * @param quaternion Input quaternion
 * @return Normalized quaternion with unit magnitude
 */
static inline FusionQuaternion FusionQuaternionNormalise(const FusionQuaternion quaternion) {
    const float magnitudeReciprocal = FusionFastInverseSqrt(
        quaternion.element.w * quaternion.element.w + 
        quaternion.element.x * quaternion.element.x + 
        quaternion.element.y * quaternion.element.y + 
        quaternion.element.z * quaternion.element.z);
    return (FusionQuaternion){.element = {
        .w = quaternion.element.w * magnitudeReciprocal,
        .x = quaternion.element.x * magnitudeReciprocal,
        .y = quaternion.element.y * magnitudeReciprocal,
        .z = quaternion.element.z * magnitudeReciprocal,
    }};
}

/**
 * @brief Matrix-vector multiplication
 * @param matrix Input 3x3 matrix
 * @param vector Input 3D vector
 * @return Result of matrix-vector multiplication
 */
static inline FusionVector FusionMatrixMultiplyVector(const FusionMatrix matrix, const FusionVector vector) {
    return (FusionVector){.axis = {
        .x = matrix.element.xx * vector.axis.x + matrix.element.xy * vector.axis.y + matrix.element.xz * vector.axis.z,
        .y = matrix.element.yx * vector.axis.x + matrix.element.yy * vector.axis.y + matrix.element.yz * vector.axis.z,
        .z = matrix.element.zx * vector.axis.x + matrix.element.zy * vector.axis.y + matrix.element.zz * vector.axis.z,
    }};
}

/**
 * @brief Convert quaternion to rotation matrix
 * @param quaternion Input quaternion
 * @return Corresponding rotation matrix
 */
static inline FusionMatrix FusionQuaternionToMatrix(const FusionQuaternion quaternion) {
    const float qwqw = quaternion.element.w * quaternion.element.w;
    const float qwqx = quaternion.element.w * quaternion.element.x;
    const float qwqy = quaternion.element.w * quaternion.element.y;
    const float qwqz = quaternion.element.w * quaternion.element.z;
    const float qxqy = quaternion.element.x * quaternion.element.y;
    const float qxqz = quaternion.element.x * quaternion.element.z;
    const float qyqz = quaternion.element.y * quaternion.element.z;
    return (FusionMatrix){.element = {
        .xx = 2.0f * (qwqw - 0.5f + quaternion.element.x * quaternion.element.x),
        .xy = 2.0f * (qxqy - qwqz),
        .xz = 2.0f * (qxqz + qwqy),
        .yx = 2.0f * (qxqy + qwqz),
        .yy = 2.0f * (qwqw - 0.5f + quaternion.element.y * quaternion.element.y),
        .yz = 2.0f * (qyqz - qwqx),
        .zx = 2.0f * (qxqz - qwqy),
        .zy = 2.0f * (qyqz + qwqx),
        .zz = 2.0f * (qwqw - 0.5f + quaternion.element.z * quaternion.element.z),
    }};
}

/**
 * @brief Convert quaternion to Euler angles
 * @param quaternion Input quaternion
 * @return Corresponding Euler angles (roll, pitch, yaw)
 */
static inline FusionEuler FusionQuaternionToEuler(const FusionQuaternion quaternion) {
    const float halfMinusQySquared = 0.5f - quaternion.element.y * quaternion.element.y;
    return (FusionEuler){.angle = {
        .roll = FusionRadiansToDegrees(atan2f(quaternion.element.w * quaternion.element.x + quaternion.element.y * quaternion.element.z, halfMinusQySquared - quaternion.element.x * quaternion.element.x)),
        .pitch = FusionRadiansToDegrees(FusionAsin(2.0f * (quaternion.element.w * quaternion.element.y - quaternion.element.z * quaternion.element.x))),
        .yaw = FusionRadiansToDegrees(atan2f(quaternion.element.w * quaternion.element.z + quaternion.element.x * quaternion.element.y, halfMinusQySquared - quaternion.element.z * quaternion.element.z)),
    }};
}

//------------------------------------------------------------------------------
// Earth Coordinate System Definitions

typedef enum {
    FusionConventionNwu, /* North-West-Up coordinate system */
    FusionConventionEnu, /* East-North-Up coordinate system */
    FusionConventionNed, /* North-East-Down coordinate system */
} FusionConvention;

//------------------------------------------------------------------------------
// AHRS Algorithm Structures

typedef struct {
    FusionConvention convention;        // Coordinate system convention used
    float gain;                        // Algorithm gain
    float gyroscopeRange;              // Gyroscope range limit (degrees/second)
    float accelerationRejection;       // Accelerometer rejection threshold
    unsigned int recoveryTriggerPeriod; // Recovery trigger period
} FusionAhrsSettings;

typedef struct {
    FusionAhrsSettings settings;        // Algorithm settings
    FusionQuaternion quaternion;        // Current attitude quaternion
    FusionVector accelerometer;         // Stored accelerometer data
    bool angularRateRecovery;           // Angular rate recovery flag
    FusionVector halfAccelerometerFeedback; // Accelerometer feedback (half value)
    bool accelerometerIgnored;          // Whether accelerometer data is ignored
    int accelerationRecoveryTrigger;    // Accelerometer recovery trigger counter
    int accelerationRecoveryTimeout;    // Accelerometer recovery timeout
    float rampedGain;       // Current ramped gain value
    float rampedGainStep;   // Gain ramping step value
    bool initialising;      // Whether the AHRS is in the initialization phase
} FusionAhrs;

typedef struct {
    float accelerationError;            // Accelerometer error (degrees)
    bool accelerometerIgnored;          // Whether accelerometer is ignored
    float accelerationRecoveryTrigger;  // Accelerometer recovery trigger progress
} FusionAhrsInternalStates;

typedef struct {
    bool angularRateRecovery;           // Whether in angular rate recovery state
    bool accelerationRecovery;          // Whether in accelerometer recovery state
} FusionAhrsFlags;

// AHRS Algorithm Function Declarations
/**
 * @brief Initialize AHRS algorithm structure
 * @param ahrs Pointer to AHRS structure to initialize
 */
void FusionAhrsInitialise(FusionAhrs *const ahrs);

/**
 * @brief Reset AHRS algorithm state to initial values
 * @param ahrs Pointer to AHRS structure to reset
 */
void FusionAhrsReset(FusionAhrs *const ahrs);

/**
 * @brief Set AHRS algorithm parameters
 * @param ahrs Pointer to AHRS structure
 * @param settings Pointer to settings structure
 */
void FusionAhrsSetSettings(FusionAhrs *const ahrs, const FusionAhrsSettings *const settings);

/**
 * @brief Update AHRS algorithm state with gyroscope and accelerometer data
 * @param ahrs Pointer to AHRS structure
 * @param gyroscope Gyroscope data (degrees/second)
 * @param accelerometer Accelerometer data (g)
 * @param deltaTime Time step (seconds)
 */
void FusionAhrsUpdate(FusionAhrs *const ahrs, const FusionVector gyroscope, const FusionVector accelerometer, const float deltaTime);

/**
 * @brief Update AHRS algorithm state without magnetometer
 * @param ahrs Pointer to AHRS structure
 * @param gyroscope Gyroscope data (degrees/second)
 * @param accelerometer Accelerometer data (g)
 * @param deltaTime Time step (seconds)
 */
void FusionAhrsUpdateNoMagnetometer(FusionAhrs *const ahrs, const FusionVector gyroscope, const FusionVector accelerometer, const float deltaTime);

/**
 * @brief Get current attitude quaternion
 * @param ahrs Pointer to AHRS structure
 * @return Current attitude quaternion
 */
FusionQuaternion FusionAhrsGetQuaternion(const FusionAhrs *const ahrs);

/**
 * @brief Set current attitude quaternion
 * @param ahrs Pointer to AHRS structure
 * @param quaternion New attitude quaternion
 */
void FusionAhrsSetQuaternion(FusionAhrs *const ahrs, const FusionQuaternion quaternion);

/**
 * @brief Get gravity vector in body frame
 * @param ahrs Pointer to AHRS structure
 * @return Gravity vector in body frame
 */
FusionVector FusionAhrsGetGravity(const FusionAhrs *const ahrs);

/**
 * @brief Get linear acceleration (gravity removed)
 * @param ahrs Pointer to AHRS structure
 * @return Linear acceleration vector
 */
FusionVector FusionAhrsGetLinearAcceleration(const FusionAhrs *const ahrs);

/**
 * @brief Get acceleration in Earth frame
 * @param ahrs Pointer to AHRS structure
 * @return Acceleration vector in Earth frame
 */
FusionVector FusionAhrsGetEarthAcceleration(const FusionAhrs *const ahrs);

/**
 * @brief Get AHRS internal states
 * @param ahrs Pointer to AHRS structure
 * @return Internal states structure
 */
FusionAhrsInternalStates FusionAhrsGetInternalStates(const FusionAhrs *const ahrs);

/**
 * @brief Get AHRS status flags
 * @param ahrs Pointer to AHRS structure
 * @return Status flags structure
 */
FusionAhrsFlags FusionAhrsGetFlags(const FusionAhrs *const ahrs);

/**
 * @brief Set heading angle
 * @param ahrs Pointer to AHRS structure
 * @param heading Heading angle in degrees
 */
void FusionAhrsSetHeading(FusionAhrs *const ahrs, const float heading);

//------------------------------------------------------------------------------
// Gyroscope Bias Correction Algorithm

typedef struct {
    float filterCoefficient;            // Filter coefficient
    unsigned int timeout;               // Timeout counter
    unsigned int timer;                 // Timer counter
    FusionVector gyroscopeOffset;       // Gyroscope bias estimate
} FusionOffset;

/**
 * @brief Initialize gyroscope bias correction
 * @param offset Pointer to offset structure
 * @param sampleRate Sampling rate (Hz)
 */
void FusionOffsetInitialise(FusionOffset *const offset, const unsigned int sampleRate);

/**
 * @brief Update gyroscope bias correction
 * @param offset Pointer to offset structure
 * @param gyroscope Raw gyroscope data
 * @return Bias-corrected gyroscope data
 */
FusionVector FusionOffsetUpdate(FusionOffset *const offset, FusionVector gyroscope);

//------------------------------------------------------------------------------
// Gradient Descent Algorithm Parameters

#define SAMPLE_FREQ_DEF   500.0f       // Default sampling frequency (Hz)
#define BETA_DEF          0.1f         // Default algorithm gain (2 * Kp)

typedef struct {
    float beta;                         // Algorithm gain
    float q0;                           // Quaternion component w (real part)
    float q1;                           // Quaternion component x (i imaginary part)
    float q2;                           // Quaternion component y (j imaginary part)
    float q3;                           // Quaternion component z (k imaginary part)
    float invSampleFreq;                // Inverse of sampling frequency
    float Fusion_Power;                 // Weight for gradient method and Fusion result fusion
    
    float maxFusionPower;               // Upper limit for fusion weight
    float minFusionPower;               // Lower limit for fusion weight
    
    float fq0;                          // Filtered quaternion component w
    float fq1;                          // Filtered quaternion component x
    float fq2;                          // Filtered quaternion component y
    float fq3;                          // Filtered quaternion component z
    float filterAlpha;                  // Low-pass filter coefficient (alpha)
                                       // Closer to 1 means weaker filtering and faster response
    
    float v_s0;                         // Momentum term for gradient descent
    float v_s1;                         // Momentum term for gradient descent
    float v_s2;                         // Momentum term for gradient descent
    float v_s3;                         // Momentum term for gradient descent
    float momentum;                     // Momentum coefficient gamma
} MadgwickParam;

/**
 * @brief Update IMU using Madgwick gradient descent algorithm
 * @param gx Gyroscope X-axis reading (degrees/second)
 * @param gy Gyroscope Y-axis reading (degrees/second)
 * @param gz Gyroscope Z-axis reading (degrees/second)
 * @param ax Accelerometer X-axis reading (g)
 * @param ay Accelerometer Y-axis reading (g)
 * @param az Accelerometer Z-axis reading (g)
 */
void Madgwick_updateIMU(float gx, float gy, float gz, float ax, float ay, float az);

#ifdef __cplusplus
}
#endif

#endif // FUSION_H
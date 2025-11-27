#pragma once

#include <cmath>
#include <cstdint>
#include <algorithm>
#include <limits>

namespace fusion {

// Constants
inline constexpr float kPi = 3.14159265358979323846f;

// -------------------------------------------------------
// Mathematical Types (with operator overloads)
// -------------------------------------------------------

struct Vector {
    float x = 0.0f, y = 0.0f, z = 0.0f;

    [[nodiscard]] constexpr bool is_zero() const noexcept {
        return x == 0.0f && y == 0.0f && z == 0.0f;
    }

    [[nodiscard]] constexpr float magnitude_squared() const noexcept {
        return x * x + y * y + z * z;
    }

    [[nodiscard]] float magnitude() const noexcept {
        return std::sqrt(magnitude_squared());
    }

    [[nodiscard]] Vector normalized() const noexcept {
        float inv_norm = fast_inverse_sqrt(magnitude_squared());
        return {x * inv_norm, y * inv_norm, z * inv_norm};
    }

    [[nodiscard]] static constexpr Vector zero() noexcept { return {}; }
    [[nodiscard]] static constexpr Vector ones() noexcept { return {1.0f, 1.0f, 1.0f}; }
};

[[nodiscard]] constexpr Vector operator+(const Vector& a, const Vector& b) noexcept {
    return {a.x + b.x, a.y + b.y, a.z + b.z};
}
[[nodiscard]] constexpr Vector operator-(const Vector& a, const Vector& b) noexcept {
    return {a.x - b.x, a.y - b.y, a.z - b.z};
}
[[nodiscard]] constexpr Vector operator*(const Vector& v, float s) noexcept {
    return {v.x * s, v.y * s, v.z * s};
}
[[nodiscard]] constexpr Vector operator*(float s, const Vector& v) noexcept { return v * s; }

[[nodiscard]] constexpr float dot(const Vector& a, const Vector& b) noexcept {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

[[nodiscard]] constexpr Vector cross(const Vector& a, const Vector& b) noexcept {
    return {
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    };
}

// -------------------------------------------------------

struct Quaternion {
    float w = 1.0f, x = 0.0f, y = 0.0f, z = 0.0f;

    [[nodiscard]] static constexpr Quaternion identity() noexcept { return {}; }

    [[nodiscard]] Quaternion normalized() const noexcept {
        float inv_norm = fast_inverse_sqrt(w * w + x * x + y * y + z * z);
        return {w * inv_norm, x * inv_norm, y * inv_norm, z * inv_norm};
    }

    // Quaternion multiplication: this * other
    [[nodiscard]] Quaternion operator*(const Quaternion& q) const noexcept {
        return {
            w * q.w - x * q.x - y * q.y - z * q.z,
            w * q.x + x * q.w + y * q.z - z * q.y,
            w * q.y - x * q.z + y * q.w + z * q.x,
            w * q.z + x * q.y - y * q.x + z * q.w
        };
    }

    // Quaternion * Vector (treat vector as pure imaginary quaternion)
    [[nodiscard]] Quaternion operator*(const Vector& v) const noexcept {
        return {
            -x * v.x - y * v.y - z * v.z,
            w * v.x + y * v.z - z * v.y,
            w * v.y - x * v.z + z * v.x,
            w * v.z + x * v.y - y * v.x
        };
    }
};

// -------------------------------------------------------

struct Euler {
    float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
    [[nodiscard]] static constexpr Euler zero() noexcept { return {}; }
};

// -------------------------------------------------------

enum class Convention {
    nwu,  // North-West-Up
    enu,  // East-North-Up
    ned   // North-East-Down
};

// -------------------------------------------------------
// Utility Functions
// -------------------------------------------------------

[[nodiscard]] constexpr float degrees_to_radians(float deg) noexcept {
    return deg * (kPi / 180.0f);
}

[[nodiscard]] constexpr float radians_to_degrees(float rad) noexcept {
    return rad * (180.0f / kPi);
}

[[nodiscard]] constexpr float safe_asin(float x) noexcept {
    if (x <= -1.0f) return -kPi / 2.0f;
    if (x >= 1.0f) return kPi / 2.0f;
    return std::asin(x);
}

// Fast inverse square root (approximate)
[[nodiscard]] inline float fast_inverse_sqrt(float x) noexcept {
    union { float f; std::int32_t i; } u;
    u.f = x;
    u.i = 0x5F1F1412 - (u.i >> 1);
    return u.f * (1.69000231f - 0.714158168f * x * u.f * u.f);
}

// -------------------------------------------------------
// AHRS Core Class
// -------------------------------------------------------

class Ahrs {
public:
    struct Settings {
        Convention convention = Convention::nwu;
        float gain = 0.5f;
        float gyroscope_range_deg_s = 2000.0f;
        float acceleration_rejection_deg = 30.0f;
        int recovery_trigger_period = 0;

        // Internal derived values
        float gyroscope_range = 0.0f;
        float acceleration_rejection = 0.0f;
    };

    struct InternalStates {
        float acceleration_error_deg = 0.0f;
        bool accelerometer_ignored = false;
        float recovery_ratio = 0.0f;
    };

    struct Flags {
        bool angular_rate_recovery = false;
        bool acceleration_recovery = false;
    };

    Ahrs();

    void set_settings(const Settings& settings);
    void reset();

    void update(const Vector& gyroscope_deg_s, const Vector& accelerometer_g, float delta_time_s);
    inline void update_no_magnetometer(const Vector& g, const Vector& a, float dt) {
        update(g, a, dt);
    }

    [[nodiscard]] const Quaternion& get_quaternion() const noexcept { return quaternion_; }
    void set_quaternion(const Quaternion& q) noexcept { quaternion_ = q; }

    [[nodiscard]] Vector get_gravity() const noexcept;
    [[nodiscard]] Vector get_linear_acceleration() const noexcept;
    [[nodiscard]] Vector get_earth_acceleration() const noexcept;
    [[nodiscard]] InternalStates get_internal_states() const noexcept;
    [[nodiscard]] Flags get_flags() const noexcept;

    void set_heading_deg(float heading_deg) noexcept;

    void enable_gradient_descent(bool enable) noexcept { use_gradient_descent_ = enable; }

private:
    [[nodiscard]] Vector half_gravity() const noexcept;
    [[nodiscard]] static Vector feedback(const Vector& sensor, const Vector& reference) noexcept;
    void madgwick_update_imu(float gx, float gy, float gz, float ax, float ay, float az) noexcept;

    Quaternion quaternion_ = Quaternion::identity();
    Vector accelerometer_ = Vector::zero();
    bool angular_rate_recovery_ = false;
    Vector half_accelerometer_feedback_ = Vector::zero();
    bool accelerometer_ignored_ = false;
    int acceleration_recovery_trigger_ = 0;
    int acceleration_recovery_timeout_ = 0;
    Settings settings_;

    struct MadgwickParams {
        float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
        float fq0 = 1.0f, fq1 = 0.0f, fq2 = 0.0f, fq3 = 0.0f;
        float beta = 0.1f;
        float inv_sample_freq = 1.0f / 500.0f; // 500 Hz default
        float filter_alpha = 0.1f;
        float min_fusion_power = 0.05f;
        float max_fusion_power = 0.3f;
        float momentum = 0.9f;
        float v_s0 = 0.0f, v_s1 = 0.0f, v_s2 = 0.0f, v_s3 = 0.0f;
    } madgwick_params_;

    bool use_gradient_descent_ = true;
};

// -------------------------------------------------------
// Gyroscope Bias Correction
// -------------------------------------------------------

class Offset {
public:
    void initialise(unsigned int sample_rate_hz) noexcept;
    [[nodiscard]] Vector update(const Vector& raw_gyro_deg_s) noexcept;

private:
    inline static constexpr float kCutoffFrequencyHz = 0.02f;
    inline static constexpr int kTimeoutSeconds = 5;
    inline static constexpr float kThresholdDegS = 3.0f;

    float filter_coefficient_ = 0.0f;
    unsigned int timeout_ = 0;
    unsigned int timer_ = 0;
    Vector gyroscope_offset_ = Vector::zero();
};

} // namespace fusion
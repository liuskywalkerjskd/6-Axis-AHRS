#include "fusion_ahrs.hpp"
#include <cmath>
#include <algorithm>

namespace fusion {

// -------------------------------------------------------
// Ahrs Implementation
// -------------------------------------------------------

Ahrs::Ahrs() {
    Settings default_settings{};
    set_settings(default_settings);
    reset();
}

void Ahrs::set_settings(const Settings& s) {
    settings_ = s;

    // Compute derived parameters
    settings_.gyroscope_range = (s.gyroscope_range_deg_s == 0.0f)
        ? std::numeric_limits<float>::max()
        : 0.98f * s.gyroscope_range_deg_s;

    if (s.acceleration_rejection_deg == 0.0f) {
        settings_.acceleration_rejection = std::numeric_limits<float>::max();
    } else {
        float half_angle_rad = 0.5f * degrees_to_radians(s.acceleration_rejection_deg);
        settings_.acceleration_rejection = std::pow(0.5f * std::sin(half_angle_rad), 2.0f);
    }

    if (s.gain == 0.0f || s.recovery_trigger_period == 0) {
        settings_.acceleration_rejection = std::numeric_limits<float>::max();
    }

    acceleration_recovery_timeout_ = s.recovery_trigger_period;
}

void Ahrs::reset() {
    quaternion_ = Quaternion::identity();
    accelerometer_ = Vector::zero();
    angular_rate_recovery_ = false;
    half_accelerometer_feedback_ = Vector::zero();
    accelerometer_ignored_ = false;
    acceleration_recovery_trigger_ = 0;
    acceleration_recovery_timeout_ = settings_.recovery_trigger_period;
}

Vector Ahrs::half_gravity() const noexcept {
    const auto& q = quaternion_;
    switch (settings_.convention) {
        case Convention::nwu:
        case Convention::enu:
            return {
                q.x * q.z - q.w * q.y,
                q.y * q.z + q.w * q.x,
                q.w * q.w - 0.5f + q.z * q.z
            };
        case Convention::ned:
            return {
                q.w * q.y - q.x * q.z,
                -(q.y * q.z + q.w * q.x),
                0.5f - q.w * q.w - q.z * q.z
            };
    }
    return Vector::zero();
}

Vector Ahrs::feedback(const Vector& sensor, const Vector& reference) noexcept {
    if (dot(sensor, reference) < 0.0f) {
        return cross(sensor, reference).normalized();
    }
    return cross(sensor, reference);
}

void Ahrs::update(const Vector& gyro, const Vector& accel, float dt) {
    accelerometer_ = accel;

    // Gyroscope range check
    if (std::abs(gyro.x) > settings_.gyroscope_range ||
        std::abs(gyro.y) > settings_.gyroscope_range ||
        std::abs(gyro.z) > settings_.gyroscope_range) {
        auto saved_quat = quaternion_;
        reset();
        quaternion_ = saved_quat;
        angular_rate_recovery_ = true;
    }

    Vector half_accel_feedback = Vector::zero();
    Vector half_grav = half_gravity();
    accelerometer_ignored_ = true;

    if (!accel.is_zero()) {
        Vector norm_accel = accel.normalized();
        half_accelerometer_feedback_ = feedback(norm_accel, half_grav);

        if (half_accelerometer_feedback_.magnitude_squared() <= settings_.acceleration_rejection) {
            accelerometer_ignored_ = false;
            acceleration_recovery_trigger_ -= 9;
        } else {
            acceleration_recovery_trigger_ += 1;
        }

        if (acceleration_recovery_trigger_ > acceleration_recovery_timeout_) {
            acceleration_recovery_timeout_ = 0;
            accelerometer_ignored_ = false;
        } else {
            acceleration_recovery_timeout_ = settings_.recovery_trigger_period;
        }

        acceleration_recovery_trigger_ = std::clamp(
            acceleration_recovery_trigger_, 0, settings_.recovery_trigger_period
        );

        if (!accelerometer_ignored_) {
            half_accel_feedback = half_accelerometer_feedback_;
        }
    }

    // Integrate: dq = 0.5 * q ? (¦Ø + gain * error) * dt
    float k = degrees_to_radians(0.5f);
    Vector adjusted_omega = {
        gyro.x * k + half_accel_feedback.x * settings_.gain,
        gyro.y * k + half_accel_feedback.y * settings_.gain,
        gyro.z * k + half_accel_feedback.z * settings_.gain
    };

    quaternion_ = (quaternion_ + (quaternion_ * adjusted_omega) * dt).normalized();

    // Gradient descent fusion
    if (use_gradient_descent_) {
        madgwick_update_imu(gyro.x, gyro.y, gyro.z, accel.x, accel.y, accel.z);

        float dot_product = quaternion_.w * madgwick_params_.q0 +
                            quaternion_.x * madgwick_params_.q1 +
                            quaternion_.y * madgwick_params_.q2 +
                            quaternion_.z * madgwick_params_.q3;
        float similarity_error = 1.0f - std::abs(dot_product);
        float dynamic_power = madgwick_params_.min_fusion_power +
                             similarity_error * (madgwick_params_.max_fusion_power - madgwick_params_.min_fusion_power);
        dynamic_power = std::min(dynamic_power, madgwick_params_.max_fusion_power);

        // Low-pass filter Madgwick output
        madgwick_params_.fq0 = madgwick_params_.filter_alpha * madgwick_params_.q0 + (1 - madgwick_params_.filter_alpha) * madgwick_params_.fq0;
        madgwick_params_.fq1 = madgwick_params_.filter_alpha * madgwick_params_.q1 + (1 - madgwick_params_.filter_alpha) * madgwick_params_.fq1;
        madgwick_params_.fq2 = madgwick_params_.filter_alpha * madgwick_params_.q2 + (1 - madgwick_params_.filter_alpha) * madgwick_params_.fq2;
        madgwick_params_.fq3 = madgwick_params_.filter_alpha * madgwick_params_.q3 + (1 - madgwick_params_.filter_alpha) * madgwick_params_.fq3;

        float inv_norm = fast_inverse_sqrt(
            madgwick_params_.fq0 * madgwick_params_.fq0 +
            madgwick_params_.fq1 * madgwick_params_.fq1 +
            madgwick_params_.fq2 * madgwick_params_.fq2 +
            madgwick_params_.fq3 * madgwick_params_.fq3
        );
        madgwick_params_.fq0 *= inv_norm;
        madgwick_params_.fq1 *= inv_norm;
        madgwick_params_.fq2 *= inv_norm;
        madgwick_params_.fq3 *= inv_norm;

        // Blend
        quaternion_.w = (1 - dynamic_power) * quaternion_.w + dynamic_power * madgwick_params_.fq0;
        quaternion_.x = (1 - dynamic_power) * quaternion_.x + dynamic_power * madgwick_params_.fq1;
        quaternion_.y = (1 - dynamic_power) * quaternion_.y + dynamic_power * madgwick_params_.fq2;
        quaternion_.z = (1 - dynamic_power) * quaternion_.z + dynamic_power * madgwick_params_.fq3;

        quaternion_ = quaternion_.normalized();
    }
}

void Ahrs::madgwick_update_imu(float gx, float gy, float gz, float ax, float ay, float az) noexcept {
    auto& p = madgwick_params_;
    const float k = degrees_to_radians(1.0f); // deg/s ¡ú rad/s
    gx *= k; gy *= k; gz *= k;

    float qDot1 = 0.5f * (-p.q1 * gx - p.q2 * gy - p.q3 * gz);
    float qDot2 = 0.5f * (p.q0 * gx + p.q2 * gz - p.q3 * gy);
    float qDot3 = 0.5f * (p.q0 * gy - p.q1 * gz + p.q3 * gx);
    float qDot4 = 0.5f * (p.q0 * gz + p.q1 * gy - p.q2 * gx);

    if (!(ax == 0.0f && ay == 0.0f && az == 0.0f)) {
        float recip_norm = fast_inverse_sqrt(ax * ax + ay * ay + az * az);
        ax *= recip_norm; ay *= recip_norm; az *= recip_norm;

        float _2q0 = 2.0f * p.q0;
        float _2q1 = 2.0f * p.q1;
        float _2q2 = 2.0f * p.q2;
        float _2q3 = 2.0f * p.q3;
        float _4q0 = 4.0f * p.q0;
        float _4q1 = 4.0f * p.q1;
        float _4q2 = 4.0f * p.q2;
        float _8q1 = 8.0f * p.q1;
        float _8q2 = 8.0f * p.q2;
        float q0q0 = p.q0 * p.q0;
        float q1q1 = p.q1 * p.q1;
        float q2q2 = p.q2 * p.q2;
        float q3q3 = p.q3 * p.q3;

        float s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        float s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * p.q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        float s2 = 4.0f * q0q0 * p.q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        float s3 = 4.0f * q1q1 * p.q3 - _2q1 * ax + 4.0f * q2q2 * p.q3 - _2q2 * ay;

        recip_norm = fast_inverse_sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        s0 *= recip_norm; s1 *= recip_norm; s2 *= recip_norm; s3 *= recip_norm;

        qDot1 -= p.beta * s0;
        qDot2 -= p.beta * s1;
        qDot3 -= p.beta * s2;
        qDot4 -= p.beta * s3;
    }

    p.q0 += qDot1 * p.inv_sample_freq;
    p.q1 += qDot2 * p.inv_sample_freq;
    p.q2 += qDot3 * p.inv_sample_freq;
    p.q3 += qDot4 * p.inv_sample_freq;

    float inv_norm = fast_inverse_sqrt(p.q0 * p.q0 + p.q1 * p.q1 + p.q2 * p.q2 + p.q3 * p.q3);
    p.q0 *= inv_norm; p.q1 *= inv_norm; p.q2 *= inv_norm; p.q3 *= inv_norm;
}

Vector Ahrs::get_gravity() const noexcept {
    const auto& q = quaternion_;
    return {
        2.0f * (q.x * q.z - q.w * q.y),
        2.0f * (q.y * q.z + q.w * q.x),
        2.0f * (q.w * q.w - 0.5f + q.z * q.z)
    };
}

Vector Ahrs::get_linear_acceleration() const noexcept {
    switch (settings_.convention) {
        case Convention::nwu:
        case Convention::enu:
            return accelerometer_ - get_gravity();
        case Convention::ned:
            return accelerometer_ + get_gravity();
    }
    return Vector::zero();
}

Vector Ahrs::get_earth_acceleration() const noexcept {
    const auto& q = quaternion_;
    const auto& a = accelerometer_;

    float qwqw = q.w * q.w;
    float qwqx = q.w * q.x;
    float qwqy = q.w * q.y;
    float qwqz = q.w * q.z;
    float qxqy = q.x * q.y;
    float qxqz = q.x * q.z;
    float qyqz = q.y * q.z;

    Vector earth_accel{
        2.0f * ((qwqw - 0.5f + q.x * q.x) * a.x + (qxqy - qwqz) * a.y + (qxqz + qwqy) * a.z),
        2.0f * ((qxqy + qwqz) * a.x + (qwqw - 0.5f + q.y * q.y) * a.y + (qyqz - qwqx) * a.z),
        2.0f * ((qxqz - qwqy) * a.x + (qyqz + qwqx) * a.y + (qwqw - 0.5f + q.z * q.z) * a.z)
    };

    switch (settings_.convention) {
        case Convention::nwu:
        case Convention::enu:
            earth_accel.z -= 1.0f;
            break;
        case Convention::ned:
            earth_accel.z += 1.0f;
            break;
    }
    return earth_accel;
}

Ahrs::InternalStates Ahrs::get_internal_states() const noexcept {
    float error_deg = radians_to_degrees(safe_asin(2.0f * half_accelerometer_feedback_.magnitude()));
    float ratio = (settings_.recovery_trigger_period == 0) ? 0.0f :
                  static_cast<float>(acceleration_recovery_trigger_) / settings_.recovery_trigger_period;
    return {error_deg, accelerometer_ignored_, ratio};
}

Ahrs::Flags Ahrs::get_flags() const noexcept {
    return {
        angular_rate_recovery_,
        acceleration_recovery_trigger_ > acceleration_recovery_timeout_
    };
}

void Ahrs::set_heading_deg(float heading_deg) noexcept {
    const auto& q = quaternion_;
    float yaw = std::atan2(q.w * q.z + q.x * q.y, 0.5f - q.y * q.y - q.z * q.z);
    float half_diff = 0.5f * (yaw - degrees_to_radians(heading_deg));
    Quaternion rot{
        std::cos(half_diff),
        0.0f,
        0.0f,
        -std::sin(half_diff)
    };
    quaternion_ = (rot * quaternion_).normalized();
}

// -------------------------------------------------------
// Offset Implementation
// -------------------------------------------------------

void Offset::initialise(unsigned int sample_rate_hz) noexcept {
    filter_coefficient_ = 2.0f * kPi * kCutoffFrequencyHz / static_cast<float>(sample_rate_hz);
    timeout_ = kTimeoutSeconds * sample_rate_hz;
    timer_ = 0;
    gyroscope_offset_ = Vector::zero();
}

Vector Offset::update(const Vector& raw_gyro) noexcept {
    Vector corrected = raw_gyro - gyroscope_offset_;

    if (std::abs(corrected.x) > kThresholdDegS ||
        std::abs(corrected.y) > kThresholdDegS ||
        std::abs(corrected.z) > kThresholdDegS) {
        timer_ = 0;
        return corrected;
    }

    if (timer_ < timeout_) {
        timer_++;
        return corrected;
    }

    gyroscope_offset_ = gyroscope_offset_ + corrected * filter_coefficient_;
    return corrected;
}

} // namespace fusion
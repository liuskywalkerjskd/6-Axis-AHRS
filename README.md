# Fusion AHRS with Adaptive Gradient Descent Fusion

A robust, standalone 6-Axis (Gyroscope and Accelerometer) Attitude and Heading Reference System (AHRS) implementation. It features a combined Madgwick gradient descent and standard quaternion fusion filter for fast convergence and stability.

[中文版 / Chinese Version](README_CH.md)

This library implements an enhanced **Attitude and Heading Reference System (AHRS)** for 6-DOF IMU (accelerometer + gyroscope). It builds upon the algorithm described in **Chapter 7 of Sebastian Madgwick's PhD thesis** ([link](https://x-io.co.uk/downloads/madgwick-phd-thesis.pdf)), with a key innovation: **adaptive fusion of a secondary gradient-descent (Madgwick) estimator** to dramatically accelerate convergence while maintaining robustness.

The core idea is to blend the primary AHRS output with a parallel Madgwick IMU solution using a **dynamic weight** that depends on the agreement (similarity) between the two quaternions. When the system is far from equilibrium (e.g., after a rapid motion), the weight increases to leverage the fast convergence of gradient descent. In steady state, the weight decreases, favoring the lower-noise primary algorithm.

> **Trade-off**: This acceleration comes at the cost of slightly increased noise during static periods. To provide full control, a runtime flag (`use_gradient_descent`) allows users to disable this feature and fall back to the standard, noise-optimized behavior.

## Features

- 🧭 Robust 6-DOF orientation estimation (no magnetometer required).
- ⚡ **Accelerated Convergence**: Adaptive fusion with a secondary Madgwick gradient-descent estimator.
- 🎚️ **Dynamic Weighting**: Fusion power automatically adjusts based on quaternion similarity.
- 🔇 **Configurable Noise**: Toggle gradient descent fusion on/off via `use_gradient_descent`.
- 🌍 Supports multiple Earth-frame conventions: NWU, ENU, NED.
- 📏 Provides gravity vector, linear acceleration, and Earth-frame acceleration.
- 🧹 Built-in gyroscope bias correction (offset calibration).
- 🧪 Modern C++17 implementation (classes, namespaces, operator overloading).
- 📦 Header-only or compiled library options.

## Usage (C++)

```cpp
#include "fusion_ahrs.hpp"

int main() {
    fusion::Ahrs ahrs;
    fusion::Offset offset;
  
    // Initialize bias correction at 100 Hz sample rate
    offset.initialise(100);
  
    // Optional: Disable gradient descent fusion to reduce static noise
    // ahrs.enable_gradient_descent(false);
  
    while (true) {
        // Read raw sensor data (deg/s, g)
        fusion::Vector raw_gyro = read_gyro();
        fusion::Vector accel = read_accelerometer();
  
        // Correct gyroscope bias
        fusion::Vector gyro = offset.update(raw_gyro);
  
        // Update AHRS (dt = 0.01s for 100 Hz)
        ahrs.update(gyro, accel, 0.01f);
  
        // Get final orientation
        auto quaternion = ahrs.get_quaternion();
    }
}
```

## Configuration

* **`Ahrs::enable_gradient_descent(bool)`** : Enable/disable the adaptive gradient descent fusion.
* **`Ahrs::set_settings(...)`** : Tune gain, rejection thresholds, and coordinate system.
* **`Offset::initialise(...)`** : Configure gyroscope bias correction parameters.

## Performance vs. Noise

| Mode                                    | Convergence Speed     | Static Noise        |
| --------------------------------------- | --------------------- | ------------------- |
| **Gradient Descent ON** (default) | ⚡**Very Fast** | 🔊 Slightly Higher  |
| **Gradient Descent OFF**          | 🚗 Fast               | 🔇**Minimal** |

Choose based on your application’s priority: **responsiveness** or  **steady-state precision** .

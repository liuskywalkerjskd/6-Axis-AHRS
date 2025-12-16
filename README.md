# Fusion AHRS with Adaptive Gradient Descent Fusion

A robust, standalone 6-Axis (Gyroscope and Accelerometer) Attitude and Heading Reference System (AHRS) implementation. It features a combined Madgwick gradient descent and standard quaternion fusion filter for fast convergence and stability.

[中文版 / Chinese Version](README_CH.md)

This library implements an enhanced **Attitude and Heading Reference System (AHRS)** for 6-DOF IMU (accelerometer + gyroscope). It builds upon the algorithm described in **Chapter 7 of Sebastian Madgwick's PhD thesis** ([link](https://x-io.co.uk/downloads/madgwick-phd-thesis.pdf)), with a key innovation: **adaptive fusion of a secondary gradient-descent (Madgwick) estimator** to dramatically accelerate convergence while maintaining robustness.
<img width="1024" height="559" alt="image" src="https://github.com/user-attachments/assets/d5992798-a5b3-470b-af7d-f27ab0807cb5" />

## Project Structure

This repository contains **two implementations** of the same algorithm, optimized for different use cases:

```
├── optimized_mcu/    # MCU-optimized version (C-style)
│   ├── Fusion_AHRS.h
│   └── Fusion_AHRS.cpp
├── readable_cpp/     # Modern C++ version (better readability)
│   ├── Fusion_AHRS.hpp
│   └── Fusion_AHRS.cpp
```

### 🔧 optimized_mcu/ - MCU-Optimized Version

- **Language**: C-style implementation (compatible with C and C++)
- **Target**: Embedded systems, MCUs (STM32, ESP32, etc.)
- **Characteristics**:
  - Highly optimized for resource-constrained environments
  - Uses inline functions and macros for performance
  - Minimizes memory allocations
  - Lower readability, but maximum performance
  - Provides C-compatible API with `extern "C"` linkage

### 📖 readable_cpp/ - Modern C++ Version

- **Language**: Modern C++17
- **Target**: Desktop applications, simulations, prototyping, educational purposes
- **Characteristics**:
  - Clean, well-structured code with classes and namespaces
  - Uses operator overloading for intuitive math operations
  - Better readability and maintainability
  - Full-featured with `[[nodiscard]]` and `noexcept` annotations
  - **Not recommended for MCU** due to potential overhead

> **Note**: Both versions implement the **exact same algorithm** and produce identical results. Choose based on your platform and development priorities.

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
- 🧪 Two implementations: MCU-optimized (C-style) and Modern C++17.
- 📦 Header-only or compiled library options.

## Usage

### Modern C++ Version (readable_cpp/)

```cpp
#include "Fusion_AHRS.hpp"

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

### MCU-Optimized Version (optimized_mcu/)

```c
#include "Fusion_AHRS.h"

// Global AHRS and offset instances
FusionAhrs ahrs;
FusionOffset offset;

int main(void) {
    // Initialize AHRS
    FusionAhrsInitialise(&ahrs);
    
    // Initialize bias correction at 100 Hz sample rate
    FusionOffsetInitialise(&offset, 100);
    
    while (1) {
        // Read raw sensor data (deg/s, g)
        FusionVector raw_gyro = {.axis = {gx, gy, gz}};
        FusionVector accel = {.axis = {ax, ay, az}};
        
        // Correct gyroscope bias
        FusionVector gyro = FusionOffsetUpdate(&offset, raw_gyro);
        
        // Update AHRS (dt = 0.01s for 100 Hz)
        FusionAhrsUpdate(&ahrs, gyro, accel, 0.01f);
        
        // Get final orientation
        FusionQuaternion quaternion = FusionAhrsGetQuaternion(&ahrs);
        
        // Convert to Euler angles if needed
        FusionEuler euler = FusionQuaternionToEuler(quaternion);
    }
}
```

## Configuration

### Modern C++ Version

* **`Ahrs::enable_gradient_descent(bool)`** : Enable/disable the adaptive gradient descent fusion.
* **`Ahrs::set_settings(...)`** : Tune gain, rejection thresholds, and coordinate system.
* **`Offset::initialise(...)`** : Configure gyroscope bias correction parameters.

### MCU-Optimized Version

* **`FusionAhrsSetSettings(...)`** : Set AHRS algorithm parameters (gain, gyroscope range, coordinate convention).
* **`FusionOffsetInitialise(...)`** : Configure gyroscope bias correction parameters.
* **`use_grad`** (global variable): Set to `0` to disable gradient descent fusion, `1` to enable (default).

## Version Comparison

| Feature | optimized_mcu/ | readable_cpp/ |
| --- | --- | --- |
| **Language** | C-style (C/C++ compatible) | Modern C++17 |
| **Target Platform** | MCU, Embedded Systems | Desktop, Simulation |
| **Performance** | ⚡ Highly Optimized | 🔧 Standard |
| **Readability** | 📖 Lower | 📖 **High** |
| **Memory Usage** | 💾 Minimal | 💾 Standard |
| **API Style** | Function-based | Class-based (OOP) |
| **Recommended For** | Production on MCU | Learning, Prototyping |

## Performance vs. Noise

| Mode                                    | Convergence Speed     | Static Noise        |
| --------------------------------------- | --------------------- | ------------------- |
| **Gradient Descent ON** (default) | ⚡**Very Fast** | 🔊 Slightly Higher  |
| **Gradient Descent OFF**          | 🚗 Fast               | 🔇**Minimal** |

Choose based on your application’s priority: **responsiveness** or  **steady-state precision** .

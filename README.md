# Fusion AHRS - 6-Axis IMU Attitude Estimation

A robust, standalone 6-Axis (Gyroscope + Accelerometer) Attitude and Heading Reference System (AHRS) implementation. This library provides two algorithms: the original **Madgwick-based fusion** and an advanced **Extended Kalman Filter (EKF)** with adaptive filtering for challenging environments.

[中文版 / 中文版本](README_CH.md)

## Table of Contents

- [Project Structure](#project-structure)
- [Algorithms](#algorithms)
  - [Madgwick Fusion](#1-madgwick-fusion)
  - [Extended Kalman Filter](#2-extended-kalman-filter-ekf)
- [Advanced Features](#advanced-features)
  - [Impact Detection & Recovery](#impact-detection--recovery)
  - [Linear Motion Compensation](#linear-motion-compensation)
- [Usage](#usage)
- [Configuration](#configuration)
- [Version Comparison](#version-comparison)

---

## Project Structure

```
├── optimized_mcu/        # MCU-optimized version (C-style)
│   ├── Fusion_AHRS.h
│   └── Fusion_AHRS.cpp
├── readable_cpp/       # Modern C++ version (better readability)
│   ├── Fusion_AHRS.hpp
│   └── Fusion_AHRS.cpp
└── EKF_Version/       # Extended Kalman Filter with adaptive features
    ├── Fusion_AHRS.h   # Base AHRS (same as optimized_mcu)
    ├── Fusion_AHRS.cpp
    ├── Tactical_Fusion.h
    └── Tactical_Fusion.cpp  # Advanced EKF implementation
```

### Module Description

| Directory | Description | Best For |
|-----------|-------------|----------|
| `optimized_mcu/` | C-style, highly optimized | Embedded systems (STM32, ESP32) |
| `readable_cpp/` | Modern C++17, OOP | Desktop, simulation, learning |
| `EKF_Version/` | EKF + adaptive filtering | High-precision, impact-prone applications |

---

## Algorithms

### 1. Madgwick Fusion

Based on Sebastian Madgwick's PhD thesis, Chapter 7.

#### Core Equations

**Quaternion Gradient Descent:**

$$
\dot{q} = \frac{1}{2} q \otimes \omega - \beta \nabla f
$$

Where:
- $q = [q_0, q_1, q_2, q_3]$ is the quaternion
- $\omega = [0, \omega_x, \omega_y, \omega_z]$ is the angular velocity
- $\beta$ is the gain parameter
- $\nabla f$ is the gradient of the objective function

**Objective Function:**

$$
f = f_d + \lambda f_g
$$

- $f_d = 2(q_1q_3 - q_0q_2) - d_x$ (direction cosine matrix error)
- $f_g = 2(q_0q_1 + q_2q_3) - d_y$ (gyroscope error)
- $\lambda$ balances the two constraints

**Discrete Update (Sample Rate $f$):**

$$
q_{k+1} = q_k + \dot{q} \cdot \frac{1}{f}
$$

**Adaptive Weighting:**

The fusion weight $w$ is dynamically adjusted based on quaternion similarity:

$$
w = w_{min} + (w_{max} - w_{min}) \cdot (1 - |q_1 \cdot q_2|)
$$

Where $q_1$ and $q_2$ are quaternions from two estimators.

---

### 2. Extended Kalman Filter (EKF)

A 6-state EKF that estimates attitude quaternion and gyroscope bias.

#### State Vector

$$
x = \begin{bmatrix} q \\ b \end{bmatrix} = \begin{bmatrix} q_0 & q_1 & q_2 & q_3 & b_x & b_y & b_z \end{bmatrix}^T
$$

Where:
- $q$: attitude quaternion
- $b$: gyroscope bias (rad/s)

#### State Transition (Prediction)

$$
\dot{q} = \frac{1}{2} q \otimes \begin{bmatrix} 0 \\ \omega - b \end{bmatrix}
$$

$$
\dot{b} = 0 \quad \text{(bias is modeled as random walk)}
$$

**Jacobian Matrix $F$:**

$$
F = \begin{bmatrix} I_3 & -\frac{1}{2}I_3 \cdot \Delta t \\ 0_{3\times3} & I_3 \end{bmatrix}
$$

**Covariance Prediction:**

$$
P_{k|k-1} = F P_{k-1} F^T + Q
$$

Where $Q$ is the process noise matrix:

$$
Q = \begin{bmatrix} q_{\theta} \cdot \Delta t \cdot I_3 & 0 \\ 0 & q_{bias} \cdot \Delta t \cdot I_3 \end{bmatrix}
$$

#### Measurement Update (Accelerometer)

The accelerometer measures gravity direction in body frame:

$$
a_{measured} = \frac{R(q)^T g}{\|R(q)^T g\|}
$$

**Innovation:**

$$
y = a_{normalized} - a_{predicted}
$$

**Kalman Gain:**

$$
K = P_{k|k-1} H^T (H P_{k|k-1} H^T + R)^{-1}
$$

Where $H$ is the Jacobian of measurement function and $R$ is observation noise.

**State Update:**

$$
\hat{x}_k = \hat{x}_{k|k-1} + K y
$$

**Covariance Update:**

$$
P_k = (I - K H) P_{k|k-1} (I - K H)^T + K R K^T
$$

---

## Advanced Features

### Impact Detection & Recovery

Detects high-acceleration events and rapidly recovers orientation estimation.

#### Detection Logic

```c
if (accDelta > accThreshold || gyroDelta > gyroThreshold) {
    impactDetected = true;
}
```

#### Recovery Mechanism

1. **Immediately reduce accelerometer weight** to ignore false readings during impact
2. **Ramp up gain** during recovery for fast convergence

```
Weight(t) = w_impact + (w_normal - w_impact) * (t / recovery_duration)
```

**Process Noise Scaling:**

During impact, process noise $Q$ is scaled inversely with weight:

$$
Q_{impact} = \frac{Q_{normal}}{w_{impact}}
$$

This makes the EKF rely more on gyro integration vs. accelerometer correction.

#### Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `impactAccThreshold` | 0.5g | Acceleration change threshold |
| `impactGyroThreshold` | 100°/s | Gyro rate change threshold |
| `impactRecoveryDuration` | 0.5s | Recovery time |
| `accWeightImpact` | 5% | Weight during impact |

---

### Linear Motion Compensation

Detects and compensates for linear acceleration (translation) interference.

#### Detection Logic

The key insight: **static acceleration magnitude ≈ 1g**, linear motion causes deviation.

$$
\text{deviation} = |\|a\| - 1|
$$

If deviation > threshold, linear motion is detected.

#### Compensation

During linear motion, accelerometer weight is further reduced:

$$
w_{linear} = w \cdot \frac{threshold}{deviation}
$$

#### Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `linearAccThreshold` | 0.15g | Deviation threshold |
| `linearMotionDecay` | 0.95 | State decay rate |
| `accCompensationEnabled` | 1 | Feature toggle |

---

## Usage

### Madgwick Version (optimized_mcu/)

```c
#include "Fusion_AHRS.h"

FusionAhrs ahrs;
FusionOffset offset;

void main() {
    FusionAhrsInitialise(&ahrs);
    FusionOffsetInitialise(&offset, 100);  // 100 Hz

    while (1) {
        FusionVector gyro = {gx, gy, gz};      // deg/s
        FusionVector acc = {ax, ay, az};       // g

        // Gyroscope bias correction
        gyro = FusionOffsetUpdate(&offset, gyro);

        // Update AHRS
        FusionAhrsUpdate(&ahrs, gyro, acc, 0.01f);

        // Get result
        FusionQuaternion q = FusionAhrsGetQuaternion(&ahrs);
        FusionEuler euler = FusionQuaternionToEuler(q);
    }
}
```

### EKF Version (EKF_Version/)

```c
#include "Tactical_Fusion.h"

TacticalSystem sys;

void main() {
    Tactical_Init(&sys, 100.0f, 0.1f);  // 100 Hz, 0.1 Hz heave cutoff

    while (1) {
        FusionVector gyro = {gx, gy, gz};      // deg/s
        FusionVector acc = {ax, ay, az};       // g

        Tactical_Update(&sys, gyro, acc);

        FusionQuaternion q = sys.quaternion;
    }
}
```

---

## Configuration

### Madgwick Parameters

| Parameter | Default | Range | Description |
|-----------|---------|-------|-------------|
| `gain` | 0.5 | 0-1 | Algorithm gain |
| `beta` | 0.1 | 0-1 | Gradient descent rate |
| `gyroscopeRange` | 2000°/s | - | Gyro range limit |
| `accelerationRejection` | 30° | - | Accel rejection threshold |

### EKF Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `processNoiseAngle` | 1e-5 | Angle process noise |
| `processNoiseBias` | 1e-7 | Bias process noise |
| `measureNoiseAcc` | 1e-2 | Accel measurement noise |
| `accStaticThreshold` | 0.02g | Static detection threshold |
| `gyroStaticThreshold` | 0.5°/s | Static gyro threshold |

---

## Version Comparison

| Feature | Madgwick | EKF (Tactical) |
|---------|----------|-----------------|
| **Algorithm** | Gradient Descent | Extended Kalman Filter |
| **Complexity** | Low | Medium |
| **Accuracy** | Good | **Excellent** |
| **Impact Recovery** | Basic | **Advanced** |
| **Linear Motion Filter** | No | **Yes** |
| **Heave Estimation** | No | **Yes** |
| **MCU Friendly** | ✅ | ✅ |

---

## Performance Characteristics

### Madgwick

| Mode | Convergence | Static Noise |
|------|-------------|--------------|
| GD ON (default) | ⚡ Very Fast | 🔊 Slightly Higher |
| GD OFF | 🚗 Fast | 🔇 Minimal |

### EKF (Tactical)

| Motion State | Behavior |
|--------------|----------|
| Static | High accel weight, ZUPT for bias correction |
| Linear Motion | Reduced accel weight, rely on gyro |
| Impact | Minimal accel weight, fast recovery |

---

## License

MIT License - See LICENSE file.

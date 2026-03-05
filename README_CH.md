# Fusion AHRS - 六轴IMU姿态估计库

一个鲁棒的、独立的六轴（陀螺仪 + 加速度计）姿态与航向参考系统（AHRS）实现。本库提供两种算法：原始的**Madgwick融合算法**和先进的**扩展卡尔曼滤波（EKF）**算法，支持自适应滤波以应对复杂环境。

[English Version / 英文版本](README.md)

## 目录

- [项目结构](#项目结构)
- [核心算法](#核心算法)
  - [Madgwick融合](#1-madgwick融合算法)
  - [扩展卡尔曼滤波](#2-扩展卡尔曼滤波ekf)
- [高级特性](#高级特性)
  - [撞击检测与恢复](#撞击检测与恢复)
  - [平动补偿](#平动补偿)
- [使用方法](#使用方法)
- [配置参数](#配置参数)
- [版本对比](#版本对比)

---

## 项目结构

```
├── optimized_mcu/        # MCU优化版本（C风格）
│   ├── Fusion_AHRS.h
│   └── Fusion_AHRS.cpp
├── readable_cpp/       # 现代C++版本（高可读性）
│   ├── Fusion_AHRS.hpp
│   └── Fusion_AHRS.cpp
└── EKF_Version/       # 扩展卡尔曼滤波+自适应特性
    ├── Fusion_AHRS.h   # 基础AHRS（与optimized_mcu相同）
    ├── Fusion_AHRS.cpp
    ├── Tactical_Fusion.h
    └── Tactical_Fusion.cpp  # 高级EKF实现
```

### 模块说明

| 目录 | 描述 | 适用场景 |
|------|------|----------|
| `optimized_mcu/` | C风格，高度优化 | 嵌入式系统（STM32、ESP32） |
| `readable_cpp/` | 现代C++17，面向对象 | 桌面、仿真、学习 |
| `EKF_Version/` | EKF + 自适应滤波 | 高精度、易受撞击的应用 |

---

## 核心算法

### 1. Madgwick融合算法

基于Sebastian Madgwick博士论文第7章。

#### 核心公式

**四元数梯度下降：**

$$
\dot{q} = \frac{1}{2} q \otimes \omega - \beta \nabla f
$$

其中：
- $q = [q_0, q_1, q_2, q_3]$ 为四元数
- $\omega = [0, \omega_x, \omega_y, \omega_z]$ 为角速度
- $\beta$ 为增益参数
- $\nabla f$ 为目标函数的梯度

**目标函数：**

$$
f = f_d + \lambda f_g
$$

- $f_d = 2(q_1q_3 - q_0q_2) - d_x$ （方向余弦矩阵误差）
- $f_g = 2(q_0q_1 + q_2q_3) - d_y$ （陀螺仪误差）
- $\lambda$ 平衡两个约束

**离散更新（采样率 $f$）：**

$$
q_{k+1} = q_k + \dot{q} \cdot \frac{1}{f}
$$

**自适应加权：**

融合权重 $w$ 根据四元数相似度动态调整：

$$
w = w_{min} + (w_{max} - w_{min}) \cdot (1 - |q_1 \cdot q_2|)
$$

其中 $q_1$ 和 $q_2$ 为两个估计器的四元数。

---

### 2. 扩展卡尔曼滤波（EKF）

6状态EKF，估计姿态四元数和陀螺仪零偏。

#### 状态向量

$$
x = \begin{bmatrix} q \\ b \end{bmatrix} = \begin{bmatrix} q_0 & q_1 & q_2 & q_3 & b_x & b_y & b_z \end{bmatrix}^T
$$

其中：
- $q$：姿态四元数
- $b$：陀螺仪零偏（rad/s）

#### 状态转移（预测）

$$
\dot{q} = \frac{1}{2} q \otimes \begin{bmatrix} 0 \\ \omega - b \end{bmatrix}
$$

$$
\dot{b} = 0 \quad \text{（零偏建模为随机游走）}
$$

**雅可比矩阵 $F$：**

$$
F = \begin{bmatrix} I_3 & -\frac{1}{2}I_3 \cdot \Delta t \\ 0_{3\times3} & I_3 \end{bmatrix}
$$

**协方差预测：**

$$
P_{k|k-1} = F P_{k-1} F^T + Q
$$

其中 $Q$ 为过程噪声矩阵：

$$
Q = \begin{bmatrix} q_{\theta} \cdot \Delta t \cdot I_3 & 0 \\ 0 & q_{bias} \cdot \Delta t \cdot I_3 \end{bmatrix}
$$

#### 量测更新（加速度计）

加速度计测量本体坐标系下的重力方向：

$$
a_{measured} = \frac{R(q)^T g}{\|R(q)^T g\|}
$$

**新息（Innovation）：**

$$
y = a_{normalized} - a_{predicted}
$$

**卡尔曼增益：**

$$
K = P_{k|k-1} H^T (H P_{k|k-1} H^T + R)^{-1}
$$

其中 $H$ 为量测函数的雅可比矩阵，$R$ 为观测噪声。

**状态更新：**

$$
\hat{x}_k = \hat{x}_{k|k-1} + K y
$$

**协方差更新：**

$$
P_k = (I - K H) P_{k|k-1} (I - K H)^T + K R K^T
$$

---

## 高级特性

### 撞击检测与恢复

检测高加速度事件并快速恢复姿态估计。

#### 检测逻辑

```c
if (accDelta > accThreshold || gyroDelta > gyroThreshold) {
    impactDetected = true;
}
```

#### 恢复机制

1. **立即降低加速度计权重**，忽略撞击期间的错误读数
2. **恢复期间斜坡上升增益**，实现快速收敛

```
Weight(t) = w_impact + (w_normal - w_impact) * (t / recovery_duration)
```

**过程噪声缩放：**

撞击期间，过程噪声 $Q$ 与权重成反比缩放：

$$
Q_{impact} = \frac{Q_{normal}}{w_{impact}}
$$

这使得EKF在撞击期间更多依赖陀螺仪积分而非加速度计校正。

#### 参数

| 参数 | 默认值 | 描述 |
|------|--------|------|
| `impactAccThreshold` | 0.5g | 加速度变化阈值 |
| `impactGyroThreshold` | 100°/s | 角速度变化阈值 |
| `impactRecoveryDuration` | 0.5s | 恢复时间 |
| `accWeightImpact` | 5% | 撞击期间权重 |

---

### 平动补偿

检测并补偿线性加速度（平移）干扰。

#### 检测逻辑

核心原理：**静止时加速度幅值 ≈ 1g**，线性运动会导致偏离。

$$
\text{deviation} = |\|a\| - 1|
$$

若偏离值超过阈值，则检测到线性运动。

#### 补偿方式

线性运动期间，进一步降低加速度计权重：

$$
w_{linear} = w \cdot \frac{threshold}{deviation}
$$

#### 参数

| 参数 | 默认值 | 描述 |
|------|--------|------|
| `linearAccThreshold` | 0.15g | 偏离阈值 |
| `linearMotionDecay` | 0.95 | 状态衰减率 |
| `accCompensationEnabled` | 1 | 功能开关 |

---

## 使用方法

### Madgwick版本（optimized_mcu/）

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

        // 陀螺仪零偏校正
        gyro = FusionOffsetUpdate(&offset, gyro);

        // 更新AHRS
        FusionAhrsUpdate(&ahrs, gyro, acc, 0.01f);

        // 获取结果
        FusionQuaternion q = FusionAhrsGetQuaternion(&ahrs);
        FusionEuler euler = FusionQuaternionToEuler(q);
    }
}
```

### EKF版本（EKF_Version/）

```c
#include "Tactical_Fusion.h"

TacticalSystem sys;

void main() {
    Tactical_Init(&sys, 100.0f, 0.1f);  // 100 Hz, 0.1 Hz 垂荡截止频率

    while (1) {
        FusionVector gyro = {gx, gy, gz};      // deg/s
        FusionVector acc = {ax, ay, az};       // g

        Tactical_Update(&sys, gyro, acc);

        FusionQuaternion q = sys.quaternion;
    }
}
```

---

## 配置参数

### Madgwick参数

| 参数 | 默认值 | 范围 | 描述 |
|------|--------|------|------|
| `gain` | 0.5 | 0-1 | 算法增益 |
| `beta` | 0.1 | 0-1 | 梯度下降率 |
| `gyroscopeRange` | 2000°/s | - | 陀螺仪量程 |
| `accelerationRejection` | 30° | - | 加速度计拒绝阈值 |

### EKF参数

| 参数 | 默认值 | 描述 |
|------|--------|------|
| `processNoiseAngle` | 1e-5 | 角度过程噪声 |
| `processNoiseBias` | 1e-7 | 零偏过程噪声 |
| `measureNoiseAcc` | 1e-2 | 加速度计观测噪声 |
| `accStaticThreshold` | 0.02g | 静止检测阈值 |
| `gyroStaticThreshold` | 0.5°/s | 静止陀螺仪阈值 |

---

## 版本对比

| 特性 | Madgwick | EKF (Tactical) |
|------|----------|----------------|
| **算法** | 梯度下降 | 扩展卡尔曼滤波 |
| **复杂度** | 低 | 中 |
| **精度** | 良好 | **优秀** |
| **撞击恢复** | 基础 | **高级** |
| **线性运动过滤** | 无 | **有** |
| **垂荡估计** | 无 | **有** |
| **MCU友好** | ✅ | ✅ |

---

## 性能特性

### Madgwick

| 模式 | 收敛速度 | 静态噪声 |
|------|----------|----------|
| GD开启（默认） | ⚡ 极快 | 🔊 略高 |
| GD关闭 | 🚗 快 | 🔇 较低 |

### EKF (Tactical)

| 运动状态 | 行为 |
|----------|------|
| 静止 | 高加速度计权重，ZUPT零偏校正 |
| 线性运动 | 降低加速度计权重，依赖陀螺仪 |
| 撞击 | 最小化加速度计权重，快速恢复 |

---

## 许可证

MIT许可证 - 见LICENSE文件。

---

**原始实现**：东北大学T-DT实验室
**本算法在RoboMaster超级对抗赛2024赛季使用至今**

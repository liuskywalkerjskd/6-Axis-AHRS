# 基于自适应梯度下降融合的 AHRS 算法

一个鲁棒、独立的六轴（陀螺仪与加速度计）姿态与航向参考系统（AHRS）实现。该系统融合了 Madgwick 梯度下降法与标准四元数融合滤波器，兼具快速收敛性与稳定性。

**本算法原始实现为C语言版本，在东北大学T-DT实验室从Robomaster超级对抗赛2024赛季使用至今，性能相对可靠。近期重构为CPP版本，若有问题可在issue中提出。**

> Q: 为什么不用EKF？
>
> A: https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=8453465 

[English Version](README.md)

本库实现了一个增强型 **姿态与航向参考系统** （AHRS），适用于 6 自由度 IMU（加速度计 + 陀螺仪）。它基于  **Sebastian Madgwick 博士论文第 7 章** （[链接 ](https://x-io.co.uk/downloads/madgwick-phd-thesis.pdf)）所述算法，并引入了**自适应融合一个辅助的梯度下降** （Madgwick）。

核心思想是：使用 **动态权重** ，将主 AHRS 输出与一个并行的 Madgwick IMU 解进行融合。该权重取决于两个四元数之间的一致性（相似度）。当系统远离平衡状态时（例如快速运动后），权重增大，以利用梯度下降法的快速收敛特性；在稳态时，权重减小，优先使用噪声更低的主算法。

> **权衡说明** ：这种加速是以**静态期间噪声略有增加**为代价的。为提供完全控制，用户可通过运行时标志 `use_gradient_descent` 禁用此功能，回退到标准的、噪声优化的行为。

## 特性

* 🧭 鲁棒的 6-DOF 姿态估计（无需磁力计）。
* ⚡  **加速收敛** ：自适应融合辅助 Madgwick 梯度下降估计器。
* 🎚️  **动态加权** ：融合强度根据四元数相似度自动调整。
* 🔇  **可配置噪声** ：通过 `use_gradient_descent` 开关控制是否启用梯度下降补偿。
* 🌍 支持多种地心坐标系：NWU、ENU、NED。
* 📏 提供重力向量、线性加速度和地心加速度。
* 🧹 内置陀螺仪偏置校正（零偏校准）。
* 🧪 现代 C++17 实现（类、命名空间、运算符重载）。
* 📦 支持头文件包含或编译为库。

## 使用示例（C++）

```cpp
#include "fusion_ahrs.hpp"

int main() {
    fusion::Ahrs ahrs;
    fusion::Offset offset;
  
    // 在 100 Hz 采样率下初始化偏置校正
    offset.initialise(100);
  
    / // 可选：禁用梯度下降融合以降低静态噪声
    // ahrs.enable_gradient_descent(false);
  
    while (true) {
        // 读取原始传感器数据（单位：deg/s, g）
        fusion::Vector raw_gyro = read_gyro();
        fusion::Vector accel = read_accelerometer();
  
        // 校正陀螺仪偏置
        fusion::Vector gyro = offset.update(raw_gyro);
  
        // 更新 AHRS（dt = 0.01s 对应 100 Hz）
        ahrs.update(gyro, accel, 0.01f);
  
        // 获取最终姿态
        auto quaternion = ahrs.get_quaternion();
    }
}
```

## 配置选项

* **`Ahrs::enable_gradient_descent(bool)`** ：启用/禁用自适应梯度下降融合。
* **`Ahrs::set_settings(...)`** ：调整增益、拒绝阈值和坐标系。
* **`Offset::initialise(...)`** ：配置陀螺仪偏置校正参数。

## 性能与噪声权衡

| 模式                            | 收敛速度         |                |
| ------------------------------- | ---------------- | -------------- |
| **梯度下降开启** （默认） | ⚡**极快** | 🔊 略高        |
| **梯度下降关闭**          | 🚗 快            | 🔇**低** |

根据你的应用需求选择：**响应速度** 或  **稳态精度** 。

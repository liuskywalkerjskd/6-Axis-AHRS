# 基于自适应梯度下降融合的 AHRS 算法

一个鲁棒、独立的六轴（陀螺仪与加速度计）姿态与航向参考系统（AHRS）实现。该系统融合了 Madgwick 梯度下降法与标准四元数融合滤波器，兼具快速收敛性与稳定性。

**本算法原始实现为C语言版本，在东北大学T-DT实验室从Robomaster超级对抗赛2024赛季使用至今，性能相对可靠。近期重构为CPP版本，若有问题可在issue中提出。**

> Q: 为什么不用EKF？
>
> A: https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=8453465 

[English Version](README.md)

本库实现了一个增强型 **姿态与航向参考系统** （AHRS），适用于 6 自由度 IMU（加速度计 + 陀螺仪）。它基于  **Sebastian Madgwick 博士论文第 7 章** （[链接 ](https://x-io.co.uk/downloads/madgwick-phd-thesis.pdf)）所述算法，并引入了**自适应融合一个辅助的梯度下降** （Madgwick）。

## 项目结构

本仓库包含**同一算法的两种实现**，针对不同的使用场景进行了优化：

```
├── optimized_mcu/    # MCU优化版本（C风格）
│   ├── Fusion_AHRS.h
│   └── Fusion_AHRS.cpp
├── readable_cpp/     # 现代C++版本（更好的可读性）
│   ├── Fusion_AHRS.hpp
│   └── Fusion_AHRS.cpp
```

### 🔧 optimized_mcu/ - MCU优化版本

- **语言**：C风格实现（兼容C和C++）
- **目标平台**：嵌入式系统、MCU（STM32、ESP32等）
- **特点**：
  - 针对资源受限环境进行高度优化
  - 使用内联函数和宏以提升性能
  - 最小化内存分配
  - 可读性较低，但性能最佳
  - 提供兼容C的API，使用 `extern "C"` 链接

### 📖 readable_cpp/ - 现代C++版本

- **语言**：现代C++17
- **目标平台**：桌面应用、仿真、原型开发、教学用途
- **特点**：
  - 使用类和命名空间的清晰、结构良好的代码
  - 使用运算符重载实现直观的数学运算
  - 更好的可读性和可维护性
  - 完整的 `[[nodiscard]]` 和 `noexcept` 注解
  - **不建议用于MCU**，因为可能存在额外开销

> **注意**：两个版本实现的是**完全相同的算法**，产生相同的结果。请根据你的平台和开发优先级进行选择。

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
* 🧪 两种实现：MCU优化版（C风格）和现代C++17版。
* 📦 支持头文件包含或编译为库。

## 使用示例

### 现代C++版本（readable_cpp/）

```cpp
#include "Fusion_AHRS.hpp"

int main() {
    fusion::Ahrs ahrs;
    fusion::Offset offset;
  
    // 在 100 Hz 采样率下初始化偏置校正
    offset.initialise(100);
  
    // 可选：禁用梯度下降融合以降低静态噪声
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

### MCU优化版本（optimized_mcu/）

```c
#include "Fusion_AHRS.h"

// 全局 AHRS 和偏置校正实例
FusionAhrs ahrs;
FusionOffset offset;

int main(void) {
    // 初始化 AHRS
    FusionAhrsInitialise(&ahrs);
    
    // 在 100 Hz 采样率下初始化偏置校正
    FusionOffsetInitialise(&offset, 100);
    
    while (1) {
        // 读取原始传感器数据（单位：deg/s, g）
        FusionVector raw_gyro = {.axis = {gx, gy, gz}};
        FusionVector accel = {.axis = {ax, ay, az}};
        
        // 校正陀螺仪偏置
        FusionVector gyro = FusionOffsetUpdate(&offset, raw_gyro);
        
        // 更新 AHRS（dt = 0.01s 对应 100 Hz）
        FusionAhrsUpdate(&ahrs, gyro, accel, 0.01f);
        
        // 获取最终姿态
        FusionQuaternion quaternion = FusionAhrsGetQuaternion(&ahrs);
        
        // 如需要可转换为欧拉角
        FusionEuler euler = FusionQuaternionToEuler(quaternion);
    }
}
```

## 配置选项

### 现代C++版本

* **`Ahrs::enable_gradient_descent(bool)`** ：启用/禁用自适应梯度下降融合。
* **`Ahrs::set_settings(...)`** ：调整增益、拒绝阈值和坐标系。
* **`Offset::initialise(...)`** ：配置陀螺仪偏置校正参数。

### MCU优化版本

* **`FusionAhrsSetSettings(...)`** ：设置 AHRS 算法参数（增益、陀螺仪量程、坐标系约定）。
* **`FusionOffsetInitialise(...)`** ：配置陀螺仪偏置校正参数。
* **`use_grad`** （全局变量）：设为 `0` 禁用梯度下降融合，设为 `1` 启用（默认）。

## 版本对比

| 特性 | optimized_mcu/ | readable_cpp/ |
| --- | --- | --- |
| **语言** | C风格（兼容C/C++） | 现代C++17 |
| **目标平台** | MCU、嵌入式系统 | 桌面、仿真 |
| **性能** | ⚡ 高度优化 | 🔧 标准 |
| **可读性** | 📖 较低 | 📖 **高** |
| **内存占用** | 💾 最小化 | 💾 标准 |
| **API风格** | 函数式 | 面向对象（OOP） |
| **推荐用途** | MCU生产环境 | 学习、原型开发 |

## 性能与噪声权衡

| 模式                            | 收敛速度         | 静态噪声       |
| ------------------------------- | ---------------- | -------------- |
| **梯度下降开启** （默认） | ⚡**极快** | 🔊 略高        |
| **梯度下降关闭**          | 🚗 快            | 🔇**低** |

根据你的应用需求选择：**响应速度** 或  **稳态精度** 。

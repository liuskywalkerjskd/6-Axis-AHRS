# Tactical EKF 使用与调参指南

## 目录

1. [算法概述](#1-算法概述)
2. [快速上手](#2-快速上手)
3. [系统架构](#3-系统架构)
4. [参数调优](#4-参数调优)
5. [工程场景处理](#5-工程场景处理)
6. [调试与诊断](#6-调试与诊断)
7. [平台移植](#7-平台移植)

---

## 1. 算法概述

Tactical EKF 是一个面向 6 轴 IMU（加速度计 + 陀螺仪）的自适应姿态解算系统。核心是一个 **6 状态误差状态 EKF**（3 个姿态角误差 + 3 个陀螺零偏），在此基础上叠加了多层自适应机制来应对工程实际中的各类干扰。

### 与传统互补滤波的区别

| 特性 | 互补滤波 | Tactical EKF |
|------|---------|-------------|
| 噪声参数 | 手动固定 | 运行时自适应估计 |
| 陀螺零偏 | 需要外部校准 | 滤波器内部自动估计 |
| 平移加速度处理 | 简单幅值门限 | 方向检测 + 周期检测 + 残差检测 |
| 冲击处理 | 无 | 冲击检测 + 恢复斜坡 |
| 浮力/升沉 | 无 | 高通滤波升沉估计 |

### 状态空间

```
状态向量 x = [δθ_x, δθ_y, δθ_z, δb_x, δb_y, δb_z]^T

δθ: 姿态角误差 (rad)
δb: 陀螺零偏误差 (rad/s)

协方差矩阵 P: 6×6
```

---

## 2. 快速上手

### 最小示例代码

```c
#include "Tactical_Fusion.h"

TacticalSystem sys;

void setup(void) {
    // 参数：采样率 (Hz), 升沉高通截止频率 (Hz)
    // 升沉截止频率：船舶 0.05~0.1Hz，车辆 0.5~1.0Hz，不需要升沉填 0
    Tactical_Init(&sys, 500.0f, 0.1f);
}

void loop(void) {
    // 从传感器读取数据
    FusionVector gyro = {.axis = {gx, gy, gz}};   // 单位：°/s
    FusionVector acc  = {.axis = {ax, ay, az}};    // 单位：g (1g = 9.8m/s²)

    // 更新 EKF
    Tactical_Update(&sys, gyro, acc);

    // 获取姿态 (四元数 → 欧拉角)
    FusionEuler euler = FusionQuaternionToEuler(sys.quaternion);
    float roll  = euler.angle.roll;   // 横滚角 (°)
    float pitch = euler.angle.pitch;  // 俯仰角 (°)
    float yaw   = euler.angle.yaw;    // 偏航角 (°) — 6轴无绝对航向，会漂移

    // 获取升沉信息
    float heavePos = sys.heavePosition;  // 升沉位移 (m)
    float heaveVel = sys.heaveVelocity;  // 升沉速度 (m/s)
}
```

### 坐标系约定

默认使用 NWU（North-West-Up）：
- X 轴指向前方
- Y 轴指向左方
- Z 轴指向上方
- 重力方向为 [0, 0, -1g]（NWU）或 [0, 0, +1g]（NED）

如需 NED 坐标系：
```c
sys.convention = FusionConventionNed;
```

---

## 3. 系统架构

### 数据流

```
原始传感器数据
    │
    ├── 陀螺仪 (°/s)
    │      │
    │      ├── 零偏校正 (减去 gyroBias)
    │      └── 噪声统计 (运行时方差估计)
    │
    └── 加速度计 (g)
           │
           ├── 噪声统计
           ├── 运动状态分析 ──→ STATIC / STABLE / DYNAMIC
           ├── 冲击检测     ──→ NONE / DETECTED / RECOVERING
           ├── 方向偏差检测  ──→ 离心力、转弯
           ├── 划桨检测     ──→ 周期性振动
           └── 持续平移检测  ──→ 车辆加减速

                    │ accWeight (自适应权重)
                    ▼
            ┌───────────────┐
            │   EKF 预测    │  ← 陀螺仪积分四元数
            │   (Predict)   │     更新 P = FPF' + Q
            └───────┬───────┘
                    │
            ┌───────▼───────┐
            │   EKF 更新    │  ← 加速度计观测
            │   (Update)    │     K = PH'(HPH'+R)^(-1)
            └───────┬───────┘     x += K·(z - h(x))
                    │             P = (I-KH)P(I-KH)' + KRK'
                    ▼
            四元数 + 陀螺零偏 + 升沉
```

### 自适应机制层级

```
第 1 层：基础自适应 (始终生效)
    ├── 运行时噪声估计 → Q, R 自动缩放
    └── 运动状态检测   → R 乘以 1x / 3x / 12x

第 2 层：冲击保护 (事件触发)
    └── 冲击检测 → 瞬时降低加速度计权重 → 斜坡恢复

第 3 层：平移加速度补偿 (持续监测)
    ├── 方向偏差检测 → 抓住 |a|≈1g 但方向错误的情况
    ├── 划桨检测     → 高方差 + 均值≈1g = 周期振动
    └── 持续残差检测  → 基于陀螺预测重力的残差跟踪
```

---

## 4. 参数调优

### 4.1 必须调整的参数（传感器相关）

这 3 个参数决定 EKF 的基础性能，必须根据你的传感器来设置：

| 参数 | 默认值 | 含义 | 如何确定 |
|------|--------|------|---------|
| `processNoiseAngle` | `1e-5` | 姿态过程噪声 Q_angle | 见下方标定方法 |
| `processNoiseBias` | `1e-7` | 零偏过程噪声 Q_bias | 通常为 Q_angle 的 1/100 |
| `measureNoiseAcc` | `1e-2` | 加速度计观测噪声 R | 见下方标定方法 |

#### 从数据手册快速估算

```
Q_angle ≈ (陀螺噪声密度 × √采样率)² × dt
         例: 0.005 °/s/√Hz, 500Hz, dt=0.002s
         = (0.005 × √500)² × 0.002 ≈ 2.5e-5

Q_bias  ≈ Q_angle × 0.01

R_acc   ≈ (加速度计噪声密度 × √采样率)²
         例: 150 µg/√Hz, 500Hz
         = (0.00015 × √500)² ≈ 1.1e-5
```

#### 从静态数据自动标定（推荐）

将 IMU 静止放置 5 秒，采集约 2500 个样本：

```c
// 采集完成后：
float gyro_var = 方差(所有陀螺样本);  // 单位：(°/s)²
float acc_var  = 方差(所有加速度计样本);  // 单位：g²

sys.params.processNoiseAngle = deg2rad(sqrt(gyro_var))² * dt;
sys.params.processNoiseBias  = sys.params.processNoiseAngle * 0.01f;
sys.params.measureNoiseAcc   = acc_var;
```

系统内部的 `UpdateNoiseStats` 实际上已经在做这件事（运行时 EMA 方差估计），所以如果你无法做静态标定，默认值在大多数消费级 IMU（MPU6050, ICM20948, BMI088 等）上也能工作。

#### 调参方向

```
姿态响应太慢、跟不上快速运动 → 增大 processNoiseAngle (更信任陀螺)
姿态静止时抖动大            → 减小 processNoiseAngle
零偏收敛太慢                → 增大 processNoiseBias
加速度计修正太激进（平移时姿态偏） → 增大 measureNoiseAcc
加速度计修正太弱（静止时不水平）   → 减小 measureNoiseAcc
```

### 4.2 可能需要调整的参数（应用相关）

| 参数 | 默认值 | 适用场景 | 调整建议 |
|------|--------|---------|---------|
| `impactAccThreshold` | `0.5g` | 冲击检测灵敏度 | 机器人 0.3g, 车辆 0.5g, 高g环境 2.0g |
| `impactRecoveryDuration` | `0.5s` | 冲击恢复时间 | 快速系统 0.2s, 缓慢系统 1.0s |
| `linearAccThreshold` | `0.15g` | 平移加速度检测门限 | 低动态 0.05g, 高动态 0.3g |
| `noiseTau` | `1.0s` | 噪声估计时间常数 | 短 = 快速适应但估计不稳, 长 = 稳定但慢 |
| `heaveCutoffFreq` | 初始化参数 | 升沉高通截止 | 船舶 0.05Hz, 车辆 0.5Hz |

### 4.3 一般不需要调整的参数

以下参数的默认值在绝大多数情况下都是合适的：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `accNoiseScale` | `2.0` | 噪声到 R 的缩放系数 |
| `gyroNoiseScale` | `1.0` | 噪声到 Q 的缩放系数 |
| `biasNoiseScale` | `0.1` | 噪声到 Q_bias 的缩放系数 |
| `accErrorScale` | `2.0` | 加速度偏差到 R 的缩放 |
| `accStaticThreshold` | `0.02g` | 静止检测加速度门限 |
| `gyroStaticThreshold` | `0.5°/s` | 静止检测陀螺门限 |
| `accStableThreshold` | `0.15g` | 稳定检测加速度门限 |
| `gyroStableThreshold` | `50°/s` | 稳定检测陀螺门限 |
| `accDirectionThreshold` | `0.15` | 方向偏差检测门限 (1-cosθ) |
| `accDirectionScale` | `10.0` | 方向偏差到 R 的缩放 |
| `paddlingVarThreshold` | `0.05 g²` | 划桨方差门限 |
| `paddlingMeanTolerance` | `0.1g` | 划桨均值容差 |
| `paddlingRScale` | `20.0` | 划桨时 R 放大倍数 |
| `paddlingWindowAlpha` | `0.01` | 划桨方差窗口 EMA 系数 |
| `sustainedLinearAlpha` | `0.05` | 持续加速度检测 EMA 系数 |

---

## 5. 工程场景处理

### 5.1 平移加速度干扰（核心难点）

**问题**：6 轴 IMU 无法直接区分重力和平移加速度（等效原理）。车辆加速、电梯升降、离心力等都会使加速度计读数偏离真实重力方向，导致姿态角误差。

**系统内置的三重检测机制**：

#### 机制 1：方向偏差检测

```
原理：利用陀螺积分预测的重力方向作为参考
      比较加速度计实测方向与预测方向的夹角
      夹角大 → 存在平移加速度 → 提高 R

优势：能检出 |a|≈1g 但方向错误的情况
      (例如匀速转弯时的离心力)

相关参数：
  accDirectionThreshold = 0.15  (1-cos 约 8.6°)
  accDirectionScale     = 10.0
```

#### 机制 2：划桨/周期振动检测

```
原理：短时窗口内统计加速度幅值的方差和均值
      高方差 + 均值≈1g = 周期性振动（划桨、跑步、骑车）
      此时大幅提高 R，基本忽略加速度计

优势：自动识别对称振动，无需知道振动频率

相关参数：
  paddlingVarThreshold  = 0.05 g²  (高于此判定为振动)
  paddlingMeanTolerance = 0.1g     (均值需在 1±0.1g 内)
  paddlingRScale        = 20.0     (R 放大 20 倍)
  paddlingWindowAlpha   = 0.01     (统计窗口约 2 秒)
```

#### 机制 3：持续平移加速度检测

```
原理：用陀螺预测的重力方向减去加速度计读数
      残差 = 平移加速度估计值
      对残差做 EMA 平滑，残差大 → 提高 R

优势：不依赖固定衰减时间，只要平移加速度真正消失
      才会恢复对加速度计的信任

相关参数：
  linearAccThreshold   = 0.15g
  sustainedLinearAlpha = 0.05  (EMA 系数, 对应约 4 秒时间常数)
```

#### 三重机制的联合作用

三个机制对 `accWeight` 的影响是**乘性叠加**的：

```
最终 accWeight = 基础权重 × (1/方向缩放) × (1/划桨缩放) × (1/持续缩放)
```

这意味着多种干扰同时存在时（例如船上划桨 + 波浪升沉），系统会自动组合各检测结果来决定对加速度计的信任程度。

### 5.2 冲击/碰撞

**问题**：突然的机械冲击会产生极大的瞬时加速度，导致 EKF 状态突变。

**处理**：检测加速度幅值变化率 `|a(t) - a(t-1)|`，超过 `impactAccThreshold` 时：
1. 瞬时将 `accWeight` 降到 `accWeightImpact`（默认 0.05，基本不信任加速度计）
2. 在 `impactRecoveryDuration` 时间内线性恢复权重

```
场景示例            impactAccThreshold    impactRecoveryDuration
精密机器人                0.3g                  0.2s
普通车辆                  0.5g                  0.5s
越野/工程机械              2.0g                  1.0s
```

### 5.3 升沉检测（Heave）

**问题**：船舶、浮标等水面平台需要估计垂直方向的周期性运动。

**处理**：
1. 将加速度旋转到地球坐标系，提取 Z 轴分量
2. 减去垂直加速度偏置（低通估计）
3. 对垂直加速度做低通滤波
4. 积分得到速度，高通滤波去除漂移
5. 再次积分得到位移，高通滤波去除漂移

```
升沉截止频率选择：
  远洋船舶    0.02 ~ 0.05 Hz  (长周期涌浪)
  近岸/内河   0.05 ~ 0.1 Hz
  小型浮标    0.1  ~ 0.3 Hz
  不需要升沉  设为 0（禁用高通滤波）
```

### 5.4 6 轴的根本限制

以下问题是 6 轴 IMU **无法解决**的：

| 限制 | 原因 | 应对方式 |
|------|------|---------|
| 航向漂移 | 无磁力计/GPS，偏航角会缓慢漂移 | 接受漂移，或外部航向源修正 |
| 恒定加速度 | 等效原理，匀加速与重力不可区分 | 所有检测都假设平移加速度是暂态的 |
| 自由落体 | 加速度计读数为零，完全失去重力参考 | 纯陀螺积分，短时间可接受 |

---

## 6. 调试与诊断

### 关键状态变量

| 变量 | 含义 | 正常范围 | 异常诊断 |
|------|------|---------|---------|
| `sys.motionState` | 运动状态 | STATIC 在静止时 | 一直 DYNAMIC → 降低门限或检查安装 |
| `sys.accWeight` | 加速度计权重 | 0.05 ~ 1.0 | 一直很低 → 传感器噪声太大 |
| `sys.accMagnitude` | 加速度幅值 | 静止时 ≈ 1.0 | 偏差大 → 需要加速度计标定 |
| `sys.gyroBias` | 陀螺零偏估计 | 逐渐收敛 | 不收敛 → 增大 processNoiseBias |
| `sys.isPaddling` | 划桨检测 | 振动时 true | 误触发 → 增大 paddlingVarThreshold |
| `sys.isLinearMotion` | 平移检测 | 加速时 true | 误触发 → 增大 linearAccThreshold |
| `sys.impactState` | 冲击状态 | 通常 NONE | 频繁触发 → 增大 impactAccThreshold |
| `sys.accDirectionError` | 方向偏差 | 静止时 ≈ 0 | 持续偏大 → 检查坐标系/安装 |
| `sys.P[0][0]` ~ `P[2][2]` | 姿态不确定性 | 逐渐减小 | 持续增大 → EKF 发散, 检查 Q/R |
| `sys.P[3][3]` ~ `P[5][5]` | 零偏不确定性 | 逐渐减小 | 持续增大 → 增大 processNoiseBias |

### 常见问题排查

**问题：姿态在静止时缓慢振荡**
```
原因：processNoiseAngle 太大或 measureNoiseAcc 太小
修复：减小 processNoiseAngle 或增大 measureNoiseAcc
验证：观察 P[0][0] 是否过大
```

**问题：快速旋转后姿态恢复慢**
```
原因：measureNoiseAcc 太大，加速度计修正太弱
修复：减小 measureNoiseAcc 或增大 processNoiseAngle
```

**问题：车辆加速时俯仰角偏移**
```
原因：平移加速度补偿不够
修复：减小 linearAccThreshold 或增大 sustainedLinearAlpha 的时间常数
验证：观察 sys.isLinearMotion 是否在加速时为 true
```

**问题：安装在振动平台上姿态漂移**
```
原因：划桨检测未触发或 R 放大不够
修复：减小 paddlingVarThreshold 或增大 paddlingRScale
验证：观察 sys.isPaddling 是否在振动时为 true
```

**问题：陀螺零偏估计值不合理**
```
正常范围：消费级 IMU 零偏通常 < 10°/s
如果 gyroBias 超过此范围，检查：
  1. 传感器数据单位是否正确（必须是 °/s 和 g）
  2. 传感器是否有机械问题
  3. processNoiseBias 是否设置过大
```

---

## 7. 平台移植

### STM32F407 (Cortex-M4F)

**完全兼容**。资源占用：

| 资源 | 占用 |
|------|------|
| RAM（TacticalSystem 结构体）| ~540 字节 |
| 栈空间（最深调用链）| ~1.0 KB |
| 每次更新 CPU 时间 @168MHz | ~12 µs |
| 500Hz 采样率 CPU 占用 | ~0.6% |
| Flash（代码段估计）| ~5 KB |

编译选项：
```
-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard -std=gnu++17 -O2
```

### 注意事项

1. **所有数学运算都是 `float`（单精度）**，完全匹配 Cortex-M4F 硬件 FPU
2. **无动态内存分配**，无 STL 依赖，无异常/RTTI
3. **无 `double`**：三角函数全部使用 `sinf`/`cosf`/`atan2f` 等 `f` 后缀版本
4. 确保链接脚本中栈大小 ≥ 2KB（大多数默认配置即可满足）

### 其他平台

| 平台 | 兼容性 | 备注 |
|------|--------|------|
| STM32F1 (Cortex-M3) | 可用，但无硬件 FPU | 软件浮点，性能下降约 10x |
| STM32H7 (Cortex-M7) | 完全兼容 | 双精度 FPU，但代码只用单精度 |
| ESP32 | 完全兼容 | 有单精度 FPU |
| Arduino AVR | 不推荐 | 8位MCU，软件浮点太慢 |
| Linux/PC | 完全兼容 | 用于仿真测试 |

---

## 附录：参数速查表

### 按调优优先级排序

```
优先级 1（必须确认）：
  processNoiseAngle  = 1e-5    陀螺噪声 → 姿态过程噪声
  processNoiseBias   = 1e-7    陀螺稳定性 → 零偏过程噪声
  measureNoiseAcc    = 1e-2    加速度计噪声 → 观测噪声

优先级 2（按需调整）：
  impactAccThreshold = 0.5     冲击检测灵敏度 (g)
  linearAccThreshold = 0.15    平移加速度门限 (g)
  noiseTau           = 1.0     噪声估计时间常数 (s)
  heaveCutoffFreq    = 0.1     升沉高通截止 (Hz)

优先级 3（通常不动）：
  accNoiseScale / gyroNoiseScale / biasNoiseScale / accErrorScale
  accStaticThreshold / gyroStaticThreshold
  accDirectionThreshold / accDirectionScale
  paddlingVarThreshold / paddlingRScale
  sustainedLinearAlpha
```

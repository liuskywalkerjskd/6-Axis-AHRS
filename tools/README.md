# IMU EKF 参数自动整定工具

通过串口接收IMU原始数据，离线优化自适应EKF参数。

## 功能特性

| 功能 | 说明 |
|------|------|
| 串口数据采集 | 从嵌入式设备实时接收IMU数据 |
| 数据录制回放 | HDF5/CSV格式存储，支持离线分析 |
| EKF离线仿真 | Python实现的EKF，用于参数评估 |
| 参数优化 | 差分进化/网格搜索算法自动寻优 |
| 可视化分析 | 姿态误差曲线、运动状态检测可视化 |

## 安装依赖

```bash
pip install numpy scipy h5py matplotlib serial
```

## 串口通信协议

嵌入式设备需按以下格式输出数据 (43字节/帧):

| 字段 | 偏移 | 长度 | 说明 |
|------|------|------|------|
| Header | 0 | 2 | 0xAA 0x55 |
| Gyro | 2 | 12 | 3×float [deg/s] |
| Acc | 14 | 12 | 3×float [g] |
| Quaternion | 26 | 16 | 4×float [w,x,y,z] |
| Checksum | 42 | 1 | 校验和 |

### C代码示例

```c
typedef struct {
    float gyro[3];      // deg/s
    float acc[3];      // g
    float quaternion[4]; // w,x,y,z
} IMUFrame;

void send_imu_frame(IMUFrame* frame) {
    uint8_t buffer[43];
    buffer[0] = 0xAA; buffer[1] = 0x55;

    memcpy(&buffer[2], frame->gyro, 12);
    memcpy(&buffer[14], frame->acc, 12);
    memcpy(&buffer[26], frame->quaternion, 16);

    // 校验和
    uint8_t sum = 0;
    for(int i = 0; i < 42; i++) sum += buffer[i];
    buffer[42] = sum;

    serial_write(buffer, 43);
}
```

## 使用方法

### 1. 采集数据

```bash
python tools/auto_tune_ekf.py collect -p /dev/ttyUSB0 -d 30 -o data.hdf5
```

参数说明:
- `-p, --port`: 串口名称 (默认: /dev/ttyUSB0)
- `-b, --baudrate`: 波特率 (默认: 115200)
- `-d, --duration`: 采集时长秒 (默认: 30.0)
- `-o, --output`: 输出文件 (默认: imu_data.hdf5)
- `-f, --format`: 输出格式 hdf5/csv (默认: hdf5)

### 2. 优化参数

```bash
# 使用测试数据 (开发调试用)
python tools/auto_tune_ekf.py optimize --test -o params.json

# 使用录制的数据
python tools/auto_tune_ekf.py optimize -i data.hdf5 -o params.json
```

参数说明:
- `-i, --input`: 输入数据文件
- `--test`: 使用内置测试数据
- `-o, --output`: 输出参数文件 (默认: ekf_params.json)
- `-m, --method`: 优化方法 de/grid (默认: de)
- `--iter`: 迭代次数 (默认: 30)

### 3. 参数分析

```bash
python tools/auto_tune_ekf.py analyze -i data.hdf5 -p params.json
```

生成分析图:
- 姿态估计误差曲线
- 运动状态检测结果

## 数据采集建议

| 场景 | 时长 | 动作描述 |
|------|------|----------|
| 静态标定 | 10s | 完全静止，放置平稳 |
| 动态测试 | 20s | 各种姿态变化、旋转 |
| 撞击测试 | 10s | 敲击IMU产生撞击 |
| 线性运动 | 10s | 前后推拉(平动) |

建议总采集时长: 30-60秒，覆盖所有典型工况。

## 优化算法

### 差分进化 (Differential Evolution)

全局优化算法，适合高维参数空间:
- 自动探索最优解
- 对初始值不敏感
- 鲁棒性强

### 网格搜索 (Grid Search)

枚举式搜索，适合参数维度较低时:
- 简单可靠
- 可并行化
- 适合小规模参数

## 参数说明

优化器会自动搜索以下参数:

| 参数 | 范围 | 说明 |
|------|------|------|
| processNoiseAngle | 1e-7 ~ 1e-3 | 角度过程噪声 |
| processNoiseBias | 1e-10 ~ 1e-5 | 零偏过程噪声 |
| measureNoiseAcc | 1e-4 ~ 1e-1 | 加速度计观测噪声 |
| accStaticThreshold | 0.01 ~ 0.05 | 静止检测加速度阈值 |
| gyroStaticThreshold | 0.1 ~ 2.0 | 静止检测陀螺仪阈值 |
| impactAccThreshold | 0.2 ~ 1.0 | 撞击检测加速度阈值 |
| impactGyroThreshold | 50 ~ 200 | 撞击检测陀螺仪阈值 |

## 输出格式

优化后的参数保存为JSON:

```json
{
  "processNoiseAngle": 1.0e-5,
  "processNoiseBias": 1.0e-7,
  "measureNoiseAcc": 0.01,
  "accStaticThreshold": 0.02,
  "gyroStaticThreshold": 0.5,
  "impactAccThreshold": 0.5,
  "impactGyroThreshold": 100.0,
  "impactRecoveryDuration": 0.5,
  "linearAccThreshold": 0.15
}
```

## 常见问题

### 1. 串口无法打开
- 检查串口权限: `sudo chmod 666 /dev/ttyUSB0`
- 确认波特率匹配

### 2. 优化时间过长
- 减少迭代次数: `--iter 10`
- 使用网格搜索: `-m grid`
- 减少数据量

### 3. 优化效果不佳
- 增加数据采集时长和工况覆盖
- 检查参考姿态数据质量
- 尝试不同优化算法

## 相关文档

- [EKF算法详解](../README.md#2-扩展卡尔曼滤波ekf)
- [C版本EKF实现](../EKF_Version/Tactical_Fusion.cpp)

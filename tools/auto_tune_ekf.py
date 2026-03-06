#!/usr/bin/env python3
"""
IMU EKF参数自动整定工具
通过串口接收IMU原始数据，离线优化EKF参数

支持功能:
1. 串口数据采集
2. 数据录制和回放
3. EKF离线仿真
4. 参数优化(差分进化/网格搜索)
5. 参数敏感性分析

使用示例:
    # 采集数据
    python auto_tune_ekf.py collect -p /dev/ttyUSB0 -d 30

    # 使用测试数据优化参数
    python auto_tune_ekf.py optimize --test -o params.json

    # 使用录制的数据优化
    python auto_tune_ekf.py optimize -i data recording.hdf5 -o params.json

    # 参数敏感性分析
    python auto_tune_ekf.py analyze -i data.hdf5 -p params.json
"""

import argparse
import serial
import struct
import time
import json
import os
import sys
import numpy as np
from dataclasses import dataclass, field
from typing import List, Tuple, Optional, Dict
from collections import deque
import threading
import h5py

# 尝试导入优化库
try:
    from scipy.optimize import differential_evolution, minimize
    from scipy.spatial.transform import Rotation
    HAS_SCIPY = True
except ImportError:
    HAS_SCIPY = False
    print("Warning: scipy not installed. Optimization will use grid search.")

try:
    import matplotlib.pyplot as plt
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False


# ==================== 数据结构 ====================

@dataclass
class IMUFrame:
    """单帧IMU数据结构"""
    timestamp: float = 0.0
    gyro: Tuple[float, float, float] = (0.0, 0.0, 0.0)  # deg/s
    acc: Tuple[float, float, float] = (0.0, 0.0, 1.0)   # g
    quaternion: Tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)  # w,x,y,z


@dataclass
class EKfParams:
    """EKF参数结构"""
    # 过程噪声
    processNoiseAngle: float = 1e-5
    processNoiseBias: float = 1e-7

    # 观测噪声
    measureNoiseAcc: float = 1e-2
    accNoiseScale: float = 2.0
    gyroNoiseScale: float = 1.0
    biasNoiseScale: float = 0.1
    accErrorScale: float = 2.0

    # 运动检测阈值
    accStaticThreshold: float = 0.02      # g
    gyroStaticThreshold: float = 0.5       # deg/s
    accStableThreshold: float = 0.15       # g
    gyroStableThreshold: float = 50.0      # deg/s

    # 静止方差阈值
    accVarStatic: float = 0.0004
    gyroVarStatic: float = 0.25

    # 噪声统计时间常数
    noiseTau: float = 1.0

    # 撞击检测参数
    impactAccThreshold: float = 0.5        # g
    impactGyroThreshold: float = 100.0     # deg/s
    impactRecoveryDuration: float = 0.5    # s
    accWeightNormal: float = 1.0
    accWeightImpact: float = 0.05

    # 线性运动检测
    linearAccThreshold: float = 0.15       # g

    def to_list(self) -> List[float]:
        return [
            self.processNoiseAngle, self.processNoiseBias,
            self.measureNoiseAcc, self.accNoiseScale, self.gyroNoiseScale,
            self.biasNoiseScale, self.accErrorScale,
            self.accStaticThreshold, self.gyroStaticThreshold,
            self.accStableThreshold, self.gyroStableThreshold,
            self.accVarStatic, self.gyroVarStatic,
            self.noiseTau,
            self.impactAccThreshold, self.impactGyroThreshold,
            self.impactRecoveryDuration, self.accWeightNormal, self.accWeightImpact,
            self.linearAccThreshold
        ]

    @staticmethod
    def from_list(values: List[float]) -> 'EKfParams':
        params = EKfParams()
        params.processNoiseAngle = values[0]
        params.processNoiseBias = values[1]
        params.measureNoiseAcc = values[2]
        params.accNoiseScale = values[3]
        params.gyroNoiseScale = values[4]
        params.biasNoiseScale = values[5]
        params.accErrorScale = values[6]
        params.accStaticThreshold = values[7]
        params.gyroStaticThreshold = values[8]
        params.accStableThreshold = values[9]
        params.gyroStableThreshold = values[10]
        params.accVarStatic = values[11]
        params.gyroVarStatic = values[12]
        params.noiseTau = values[13]
        params.impactAccThreshold = values[14]
        params.impactGyroThreshold = values[15]
        params.impactRecoveryDuration = values[16]
        params.accWeightNormal = values[17]
        params.accWeightImpact = values[18]
        params.linearAccThreshold = values[19]
        return params

    def to_dict(self) -> Dict:
        return {
            'processNoiseAngle': self.processNoiseAngle,
            'processNoiseBias': self.processNoiseBias,
            'measureNoiseAcc': self.measureNoiseAcc,
            'accNoiseScale': self.accNoiseScale,
            'gyroNoiseScale': self.gyroNoiseScale,
            'biasNoiseScale': self.biasNoiseScale,
            'accErrorScale': self.accErrorScale,
            'accStaticThreshold': self.accStaticThreshold,
            'gyroStaticThreshold': self.gyroStaticThreshold,
            'accStableThreshold': self.accStableThreshold,
            'gyroStableThreshold': self.gyroStableThreshold,
            'accVarStatic': self.accVarStatic,
            'gyroVarStatic': self.gyroVarStatic,
            'noiseTau': self.noiseTau,
            'impactAccThreshold': self.impactAccThreshold,
            'impactGyroThreshold': self.impactGyroThreshold,
            'impactRecoveryDuration': self.impactRecoveryDuration,
            'accWeightNormal': self.accWeightNormal,
            'accWeightImpact': self.accWeightImpact,
            'linearAccThreshold': self.linearAccThreshold,
        }


# ==================== 串口通信 ====================

class IMUSerialReader:
    """
    串口IMU数据读取器

    默认协议格式 (43字节):
    - Header: 0xAA 0x55 (2字节)
    - Gyro: 3 x float (12字节) [deg/s]
    - Acc: 3 x float (12字节) [g]
    - Quaternion: 4 x float (16字节) [w,x,y,z]
    - Checksum: 1字节
    """

    PROTOCOL_HEADER = [0xAA, 0x55]

    def __init__(self, port: str, baudrate: int = 115200,
                 protocol: str = 'default', frame_size: int = 43):
        self.port = port
        self.baudrate = baudrate
        self.protocol = protocol
        self.frame_size = frame_size
        self.serial = None
        self.running = False
        self.data_buffer: List[IMUFrame] = []
        self.lock = threading.Lock()
        self.bytes_buffer = bytes()

    def start(self) -> bool:
        """启动串口读取线程"""
        try:
            self.serial = serial.Serial(
                self.port, self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=1.0
            )
            self.running = True
            self.thread = threading.Thread(target=self._read_loop)
            self.thread.daemon = True
            self.thread.start()
            print(f"串口 {self.port} 已打开")
            return True
        except Exception as e:
            print(f"串口打开失败: {e}")
            return False

    def stop(self):
        """停止读取"""
        self.running = False
        if self.serial and self.serial.is_open:
            self.serial.close()
        print("串口已关闭")

    def _read_loop(self):
        """串口读取循环"""
        while self.running:
            try:
                if self.serial.in_waiting > 0:
                    self.bytes_buffer += self.serial.read(self.serial.in_waiting)

                    # 查找帧头
                    while len(self.bytes_buffer) >= self.frame_size:
                        # 尝试找帧头
                        header_idx = -1
                        for i in range(len(self.bytes_buffer) - 1):
                            if (self.bytes_buffer[i] == self.PROTOCOL_HEADER[0] and
                                self.bytes_buffer[i+1] == self.PROTOCOL_HEADER[1]):
                                header_idx = i
                                break

                        if header_idx == -1:
                            # 没有找到帧头，保留最后几个字节
                            self.bytes_buffer = self.bytes_buffer[-2:]
                            break

                        # 跳过无效字节
                        if header_idx > 0:
                            self.bytes_buffer = self.bytes_buffer[header_idx:]

                        if len(self.bytes_buffer) >= self.frame_size:
                            frame_data = self.bytes_buffer[:self.frame_size]
                            self.bytes_buffer = self.bytes_buffer[self.frame_size:]

                            # 解析帧
                            frame = self._parse_frame(frame_data)
                            if frame:
                                with self.lock:
                                    self.data_buffer.append(frame)
            except Exception as e:
                print(f"读取错误: {e}")
                time.sleep(0.01)

    def _parse_frame(self, data: bytes) -> Optional[IMUFrame]:
        """解析单帧数据"""
        try:
            # 验证校验和
            checksum = sum(data[:self.frame_size-1]) & 0xFF
            if checksum != data[self.frame_size-1]:
                return None

            gyro = struct.unpack('fff', data[2:14])
            acc = struct.unpack('fff', data[14:26])
            quat = struct.unpack('ffff', data[26:42])

            return IMUFrame(
                timestamp=time.time(),
                gyro=gyro,
                acc=acc,
                quaternion=quat
            )
        except:
            return None

    def get_data(self, max_count: int = 1000) -> List[IMUFrame]:
        """获取缓冲区数据"""
        with self.lock:
            if len(self.data_buffer) <= max_count:
                data = self.data_buffer.copy()
                self.data_buffer.clear()
            else:
                data = self.data_buffer[-max_count:]
                self.data_buffer = self.data_buffer[-max_count:]
        return data


# ==================== 数据录制 ====================

class IMURecorder:
    """IMU数据录制器"""

    def __init__(self, output_path: str):
        self.output_path = output_path
        self.frames: List[IMUFrame] = []

    def add_frame(self, frame: IMUFrame):
        self.frames.append(frame)

    def add_frames(self, frames: List[IMUFrame]):
        self.frames.extend(frames)

    def save(self, format: str = 'hdf5'):
        """保存数据"""
        if format == 'hdf5':
            self._save_hdf5()
        elif format == 'csv':
            self._save_csv()
        else:
            raise ValueError(f"Unknown format: {format}")

    def _save_hdf5(self):
        """保存为HDF5格式"""
        n = len(self.frames)
        if n == 0:
            print("没有数据可保存")
            return

        with h5py.File(self.output_path, 'w') as f:
            f.create_dataset('timestamp', data=[fr.timestamp for fr in self.frames])
            f.create_dataset('gyro', data=[fr.gyro for fr in self.frames])
            f.create_dataset('acc', data=[fr.acc for fr in self.frames])
            f.create_dataset('quaternion', data=[fr.quaternion for fr in self.frames])

        print(f"已保存 {n} 帧到 {self.output_path}")

    def _save_csv(self):
        """保存为CSV格式"""
        import csv
        with open(self.output_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['timestamp', 'gyro_x', 'gyro_y', 'gyro_z',
                             'acc_x', 'acc_y', 'acc_z', 'quat_w', 'quat_x', 'quat_y', 'quat_z'])
            for fr in self.frames:
                writer.writerow([fr.timestamp] + list(fr.gyro) + list(fr.acc) + list(fr.quaternion))
        print(f"已保存 {len(self.frames)} 帧到 {self.output_path}")

    @staticmethod
    def load(path: str) -> List[IMUFrame]:
        """加载数据"""
        if path.endswith('.hdf5') or path.endswith('.h5'):
            return IMURecorder._load_hdf5(path)
        elif path.endswith('.csv'):
            return IMURecorder._load_csv(path)
        else:
            raise ValueError(f"Unknown file format: {path}")

    @staticmethod
    def _load_hdf5(path: str) -> List[IMUFrame]:
        frames = []
        with h5py.File(path, 'r') as f:
            timestamps = f['timestamp'][:]
            gyros = f['gyro'][:]
            accs = f['acc'][:]
            quats = f['quaternion'][:]

            for i in range(len(timestamps)):
                frames.append(IMUFrame(
                    timestamp=timestamps[i],
                    gyro=tuple(gyros[i]),
                    acc=tuple(accs[i]),
                    quaternion=tuple(quats[i])
                ))
        print(f"已加载 {len(frames)} 帧")
        return frames

    @staticmethod
    def _load_csv(path: str) -> List[IMUFrame]:
        import csv
        frames = []
        with open(path, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                frames.append(IMUFrame(
                    timestamp=float(row['timestamp']),
                    gyro=(float(row['gyro_x']), float(row['gyro_y']), float(row['gyro_z'])),
                    acc=(float(row['acc_x']), float(row['acc_y']), float(row['acc_z'])),
                    quaternion=(float(row['quat_w']), float(row['quat_x']),
                               float(row['quat_y']), float(row['quat_z']))
                ))
        print(f"已加载 {len(frames)} 帧")
        return frames


# ==================== EKF仿真器 ====================

class PythonEKF:
    """
    Python实现的EKF仿真器

    用于离线评估不同参数下的EKF性能
    """

    def __init__(self, params: EKfParams, sample_rate: float = 100.0):
        self.params = params
        self.sample_rate = sample_rate
        self.dt = 1.0 / sample_rate

        # 状态: 四元数 + 陀螺仪零偏
        self.quaternion = np.array([1.0, 0.0, 0.0, 0.0])  # w,x,y,z
        self.gyro_bias = np.array([0.0, 0.0, 0.0])

        # 协方差矩阵
        self.P = np.diag([1e-3, 1e-3, 1e-3, 1e-2, 1e-2, 1e-2])

        # 噪声统计
        self.acc_mean = np.array([0.0, 0.0, 1.0])
        self.gyro_mean = np.array([0.0, 0.0, 0.0])
        self.acc_var = np.array([1e-4, 1e-4, 1e-4])
        self.gyro_var = np.array([0.01, 0.01, 0.01])
        self.acc_mag_mean = 1.0
        self.gyro_mag_mean = 0.0
        self.acc_mag_var = 1e-4
        self.gyro_mag_var = 0.01

        # 运动状态
        self.motion_state = 'STATIC'  # STATIC, STABLE, DYNAMIC
        self.acc_magnitude = 1.0

        # 撞击状态
        self.impact_state = 'NONE'  # NONE, DETECTED, RECOVERING
        self.impact_timer = 0.0
        self.acc_weight = 1.0
        self.prev_acc_mag = 1.0
        self.prev_gyro_mag = 0.0

    def quaternion_multiply(self, q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
        """四元数乘法"""
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        return np.array([
            w1*w2 - x1*x2 - y1*y2 - z1*z2,
            w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2
        ])

    def quaternion_conjugate(self, q: np.ndarray) -> np.ndarray:
        """四元数共轭"""
        return np.array([q[0], -q[1], -q[2], -q[3]])

    def quaternion_normalize(self, q: np.ndarray) -> np.ndarray:
        """四元数归一化"""
        norm = np.linalg.norm(q)
        if norm < 1e-10:
            return np.array([1.0, 0.0, 0.0, 0.0])
        return q / norm

    def update_noise_stats(self, acc: np.ndarray, gyro: np.ndarray):
        """更新噪声统计"""
        p = self.params
        alpha = self.dt / (p.noiseTau + self.dt) if p.noiseTau > 0 else 1.0

        # 更新均值和方差
        for i in range(3):
            delta = acc[i] - self.acc_mean[i]
            self.acc_mean[i] += alpha * delta
            self.acc_var[i] += alpha * (delta * delta - self.acc_var[i])

            delta = gyro[i] - self.gyro_mean[i]
            self.gyro_mean[i] += alpha * delta
            self.gyro_var[i] += alpha * (delta * delta - self.gyro_var[i])

        acc_mag = np.linalg.norm(acc)
        gyro_mag = np.linalg.norm(gyro)

        delta = acc_mag - self.acc_mag_mean
        self.acc_mag_mean += alpha * delta
        self.acc_mag_var += alpha * (delta * delta - self.acc_mag_var)

        delta = gyro_mag - self.gyro_mag_mean
        self.gyro_mag_mean += alpha * delta
        self.gyro_mag_var += alpha * (delta * delta - self.gyro_mag_var)

    def analyze_motion(self, acc: np.ndarray, gyro: np.ndarray) -> str:
        """分析运动状态"""
        p = self.params
        acc_mag = np.linalg.norm(acc)
        gyro_mag = np.linalg.norm(gyro)
        acc_err = abs(acc_mag - 1.0)

        acc_var_mag = np.sum(self.acc_var)
        gyro_var_mag = np.sum(self.gyro_var)

        if (acc_err < p.accStaticThreshold and
            gyro_mag < p.gyroStaticThreshold and
            self.acc_mag_var < p.accVarStatic and
            self.gyro_mag_var < p.gyroVarStatic):
            return 'STATIC'

        if (acc_err < p.accStableThreshold and
            gyro_mag < p.gyroStableThreshold):
            return 'STABLE'

        return 'DYNAMIC'

    def compute_adaptive_noise(self, acc_error: float) -> Tuple[float, float, float]:
        """计算自适应噪声"""
        p = self.params

        acc_noise = np.sqrt(max(np.sum(self.acc_var), 0.0))
        gyro_noise = np.sqrt(max(np.sum(self.gyro_var), 0.0))

        if acc_noise <= 0:
            acc_noise = 0.01
        if gyro_noise <= 0:
            gyro_noise = 0.05

        gyro_noise_rad = np.deg2rad(gyro_noise)
        gyro_noise_var = gyro_noise_rad ** 2
        acc_noise_var = acc_noise ** 2

        q_angle = p.processNoiseAngle + p.gyroNoiseScale * gyro_noise_var
        q_bias = p.processNoiseBias + p.biasNoiseScale * gyro_noise_var
        r_acc = p.measureNoiseAcc + p.accNoiseScale * acc_noise_var + p.accErrorScale * acc_error ** 2

        if self.motion_state == 'STABLE':
            r_acc *= 3.0
        elif self.motion_state == 'DYNAMIC':
            r_acc *= 12.0

        # 限制范围
        q_angle = np.clip(q_angle, 1e-9, 1.0)
        q_bias = np.clip(q_bias, 1e-12, 1.0)
        r_acc = np.clip(r_acc, 1e-6, 1.0)

        return q_angle, q_bias, r_acc

    def gravity_body(self, q: np.ndarray) -> np.ndarray:
        """计算本体坐标系下的重力向量"""
        w, x, y, z = q

        half_gravity = np.array([
            x*z - w*y,
            y*z + w*x,
            w*w - 0.5 + z*z
        ])

        return 2.0 * half_gravity

    def update(self, gyro: np.ndarray, acc: np.ndarray) -> np.ndarray:
        """
        EKF更新一步

        Args:
            gyro: 陀螺仪数据 [deg/s]
            acc: 加速度计数据 [g]

        Returns:
            估计的四元数
        """
        p = self.params
        dt = self.dt

        # 更新噪声统计
        self.update_noise_stats(acc, gyro)

        # 分析运动状态
        self.motion_state = self.analyze_motion(acc, gyro)
        self.acc_magnitude = np.linalg.norm(acc)

        # ===== 撞击检测 =====
        acc_mag = self.acc_magnitude
        gyro_mag = np.linalg.norm(gyro)
        acc_delta = abs(acc_mag - self.prev_acc_mag)
        gyro_delta = abs(gyro_mag - self.prev_gyro_mag)

        self.prev_acc_mag = acc_mag
        self.prev_gyro_mag = gyro_mag

        if self.impact_state == 'NONE':
            if (acc_delta > p.impactAccThreshold or
                gyro_delta > p.impactGyroThreshold * dt):
                self.impact_state = 'DETECTED'
                self.impact_timer = 0.0

        if self.impact_state == 'DETECTED':
            self.impact_state = 'RECOVERING'
            self.impact_timer = 0.0
            self.acc_weight = p.accWeightImpact
        elif self.impact_state == 'RECOVERING':
            self.impact_timer += dt
            recovery_progress = self.impact_timer / p.impactRecoveryDuration
            self.acc_weight = p.accWeightImpact + (p.accWeightNormal - p.accWeightImpact) * recovery_progress

            if self.impact_timer >= p.impactRecoveryDuration:
                self.impact_state = 'NONE'
                self.acc_weight = p.accWeightNormal
        else:
            # 正常状态
            if self.motion_state == 'STATIC':
                self.acc_weight = p.accWeightNormal
            elif self.motion_state == 'STABLE':
                self.acc_weight = p.accWeightNormal * 0.5
            else:
                self.acc_weight = p.accWeightNormal * 0.2

        # ===== EKF预测 =====
        gyro_corrected = gyro - self.gyro_bias
        omega_rad = np.deg2rad(gyro_corrected)
        delta_theta = omega_rad * 0.5 * dt

        delta_q = np.array([
            1.0,
            delta_theta[0],
            delta_theta[1],
            delta_theta[2]
        ])

        self.quaternion = self.quaternion_multiply(self.quaternion, delta_q)
        self.quaternion = self.quaternion_normalize(self.quaternion)

        # 协方差预测 (简化版)
        acc_error = abs(self.acc_magnitude - 1.0)
        q_angle, q_bias, _ = self.compute_adaptive_noise(acc_error)

        # 撞击时增加过程噪声
        impact_q_scale = 1.0 / max(self.acc_weight, 0.01)
        q_angle *= impact_q_scale

        for i in range(3):
            self.P[i, i] += q_angle * dt
        for i in range(3, 6):
            self.P[i, i] += q_bias * dt

        # ===== EKF更新 =====
        acc_norm = acc / (np.linalg.norm(acc) + 1e-10)
        gravity = self.gravity_body(self.quaternion)
        if np.linalg.norm(gravity) > 0:
            gravity = gravity / np.linalg.norm(gravity)

        error = np.cross(acc_norm, gravity)

        _, _, r_acc = self.compute_adaptive_noise(acc_error)

        # 卡尔曼增益 (简化)
        S = self.P[:3, :3] + r_acc * np.eye(3)
        try:
            S_inv = np.linalg.inv(S)
        except:
            S_inv = np.eye(3)

        K1 = self.P[:3, :3] @ S_inv
        K2 = self.P[3:6, :3] @ S_inv

        # 状态更新
        delta_theta_kalman = K1 @ error * 0.5
        delta_q_kalman = np.array([1.0, delta_theta_kalman[0], delta_theta_kalman[1], delta_theta_kalman[2]])
        self.quaternion = self.quaternion_multiply(self.quaternion, delta_q_kalman)
        self.quaternion = self.quaternion_normalize(self.quaternion)

        # 零偏更新
        delta_bias = K2 @ error
        self.gyro_bias += np.rad2deg(delta_bias)

        # 协方差更新 (简化)
        I_KH = np.eye(6)
        I_KH[:3, :3] -= K1
        I_KH[3:6, :3] -= K2
        self.P = I_KH @ self.P @ I_KH.T + r_acc * (K1 @ K1.T + K2 @ K2.T)

        # 静止时ZUPT
        if self.motion_state == 'STATIC':
            alpha = min(p.noiseTau * 0.001, 1.0)
            self.gyro_bias += (gyro - self.gyro_bias) * alpha

        return self.quaternion.copy()


# ==================== 参数优化 ====================

class ParameterOptimizer:
    """EKF参数优化器"""

    # 参数边界定义
    PARAM_BOUNDS = [
        (1e-7, 1e-3),   # processNoiseAngle
        (1e-10, 1e-5),  # processNoiseBias
        (1e-4, 1e-1),   # measureNoiseAcc
        (0.5, 5.0),     # accNoiseScale
        (0.1, 3.0),     # gyroNoiseScale
        (0.01, 0.5),    # biasNoiseScale
        (0.5, 5.0),     # accErrorScale
        (0.01, 0.05),   # accStaticThreshold
        (0.1, 2.0),     # gyroStaticThreshold
        (0.05, 0.3),    # accStableThreshold
        (20, 100),      # gyroStableThreshold
        (0.0001, 0.001),# accVarStatic
        (0.1, 1.0),     # gyroVarStatic
        (0.1, 5.0),     # noiseTau
        (0.2, 1.0),     # impactAccThreshold
        (50, 200),      # impactGyroThreshold
        (0.2, 1.0),     # impactRecoveryDuration
        (0.5, 1.0),     # accWeightNormal
        (0.01, 0.2),    # accWeightImpact
        (0.1, 0.3),     # linearAccThreshold
    ]

    PARAM_NAMES = [
        'processNoiseAngle', 'processNoiseBias', 'measureNoiseAcc',
        'accNoiseScale', 'gyroNoiseScale', 'biasNoiseScale', 'accErrorScale',
        'accStaticThreshold', 'gyroStaticThreshold', 'accStableThreshold',
        'gyroStableThreshold', 'accVarStatic', 'gyroVarStatic', 'noiseTau',
        'impactAccThreshold', 'impactGyroThreshold', 'impactRecoveryDuration',
        'accWeightNormal', 'accWeightImpact', 'linearAccThreshold'
    ]

    def __init__(self, imu_data: List[IMUFrame], sample_rate: float = 100.0):
        self.imu_data = imu_data
        self.sample_rate = sample_rate

        # 使用嵌入式设备传来的姿态作为参考
        self.reference_quats = np.array([d.quaternion for d in imu_data])

    def evaluate(self, params: List[float]) -> float:
        """
        评估一组参数的性能

        Returns:
            姿态误差得分 (越小越好)
        """
        try:
            ekf_params = EKfParams.from_list(params)
            ekf = PythonEKF(ekf_params, self.sample_rate)

            errors = []
            for frame in self.imu_data:
                gyro_np = np.array(frame.gyro)
                acc_np = np.array(frame.acc)

                est_quat = ekf.update(gyro_np, acc_np)

                # 计算与参考姿态的误差
                error = self._quaternion_error(est_quat, np.array(frame.quaternion))
                errors.append(error)

            # 综合得分
            errors = np.array(errors)
            mean_error = np.mean(errors)
            std_error = np.std(errors)
            max_error = np.max(errors)
            p95_error = np.percentile(errors, 95)

            # 偏向稳定且误差小的参数
            score = mean_error + 0.2 * std_error + 0.1 * max_error + 0.05 * p95_error

            return score

        except Exception as e:
            print(f"评估错误: {e}")
            return 1e6  # 返回大值表示失败

    def _quaternion_error(self, q1: np.ndarray, q2: np.ndarray) -> float:
        """计算四元数角度误差 (度)"""
        q1 = q1 / np.linalg.norm(q1)
        q2 = q2 / np.linalg.norm(q2)

        dot = abs(np.dot(q1, q2))
        dot = min(dot, 1.0)
        error_rad = 2 * np.arccos(dot)
        return np.rad2deg(error_rad)

    def optimize(self, method: str = 'differential_evolution',
                  max_iter: int = 30) -> Tuple[EKfParams, float]:
        """运行参数优化"""

        if method == 'differential_evolution':
            if not HAS_SCIPY:
                print("scipy未安装，使用网格搜索")
                return self._grid_search()

            result = differential_evolution(
                self.evaluate,
                self.PARAM_BOUNDS,
                maxiter=max_iter,
                seed=42,
                polish=True,
                workers=1,
                disp=True
            )

            optimal_params = EKfParams.from_list(result.x)
            return optimal_params, result.fun

        elif method == 'grid_search':
            return self._grid_search()

        else:
            raise ValueError(f"Unknown method: {method}")

    def _grid_search(self) -> Tuple[EKfParams, float]:
        """简化网格搜索 (仅用于演示)"""
        print("运行简化网格搜索...")

        # 简化的参数搜索
        best_score = float('inf')
        best_params = None

        # 搜索几个关键参数
        for q_angle in [1e-5, 1e-4, 1e-3]:
            for r_acc in [1e-3, 1e-2, 1e-1]:
                for acc_thresh in [0.02, 0.05, 0.1]:
                    params = EKfParams()
                    params.processNoiseAngle = q_angle
                    params.measureNoiseAcc = r_acc
                    params.accStaticThreshold = acc_thresh

                    score = self.evaluate(params.to_list())

                    if score < best_score:
                        best_score = score
                        best_params = params

        return best_params, best_score


# ==================== 工具函数 ====================

def generate_test_data(duration: float = 30.0, sample_rate: float = 100.0) -> List[IMUFrame]:
    """生成测试数据"""
    np.random.seed(42)
    n = int(duration * sample_rate)

    data = []
    t = 0.0

    for i in range(n):
        # 静态阶段 (0-5s)
        if i < 5 * sample_rate:
            acc = (np.random.randn() * 0.005, np.random.randn() * 0.005, 1.0 + np.random.randn() * 0.005)
            gyro = (np.random.randn() * 0.1, np.random.randn() * 0.1, np.random.randn() * 0.1)
            quat = (1.0, 0.0, 0.0, 0.0)

        # 动态阶段 (5-15s)
        elif i < 15 * sample_rate:
            t = (i - 5 * sample_rate) / sample_rate
            # 缓慢旋转
            angle = 0.1 * t
            c, s = np.cos(angle), np.sin(angle)
            quat = (c, s * 0.1, s * 0.05, 0.0)

            acc = (np.random.randn() * 0.02, np.random.randn() * 0.02, 1.0 + np.random.randn() * 0.02)
            gyro = (10 * np.sin(t * 0.5), 5 * np.cos(t * 0.3), 3 * np.sin(t * 0.7))
            gyro = tuple(gyro + np.random.randn(3) * 0.5)

        # 撞击阶段 (15-17s)
        elif i < 17 * sample_rate:
            # 突然的加速度变化
            acc = (np.random.randn() * 0.3, np.random.randn() * 0.3, 1.5 + np.random.randn() * 0.3)
            gyro = (np.random.randn() * 50, np.random.randn() * 50, np.random.randn() * 50)
            quat = (0.9, 0.2, 0.2, 0.3)  # 撞击后姿态突变

        # 恢复阶段 (17-25s)
        elif i < 25 * sample_rate:
            t = (i - 17 * sample_rate) / sample_rate
            # 逐渐收敛到静止
            alpha = min(t / 3.0, 1.0)
            acc = (np.random.randn() * 0.02 * alpha, np.random.randn() * 0.02 * alpha,
                   1.0 + np.random.randn() * 0.02 * alpha)
            gyro = (np.random.randn() * 0.5 * alpha, np.random.randn() * 0.5 * alpha,
                   np.random.randn() * 0.5 * alpha)
            quat = (1.0, 0.0, 0.0, 0.0)

        # 线性运动阶段 (25-30s)
        else:
            t = (i - 25 * sample_rate) / sample_rate
            # 线性加速导致acc偏离1g
            linear_acc = 0.3 * np.sin(t * 2)
            acc = (linear_acc, 0.0, 1.0 + linear_acc)
            gyro = (np.random.randn() * 0.5, np.random.randn() * 0.5, np.random.randn() * 0.5)
            quat = (1.0, 0.0, 0.0, 0.0)

        data.append(IMUFrame(
            timestamp=i / sample_rate,
            gyro=gyro,
            acc=acc,
            quaternion=quat
        ))

    return data


def collect_data_cli(args):
    """命令行采集数据"""
    print(f"开始采集数据，时长 {args.duration} 秒...")

    reader = IMUSerialReader(args.port, args.baudrate)
    if not reader.start():
        return

    recorder = IMURecorder(args.output)

    start_time = time.time()
    try:
        while time.time() - start_time < args.duration:
            frames = reader.get_data()
            if frames:
                recorder.add_frames(frames)
                print(f"\r已采集 {len(recorder.frames)} 帧", end='', flush=True)
            time.sleep(0.01)
    except KeyboardInterrupt:
        print("\n用户中断")
    finally:
        reader.stop()

    print(f"\n采集完成，共 {len(recorder.frames)} 帧")

    if len(recorder.frames) > 0:
        recorder.save(args.format)
    else:
        print("没有数据可保存")


def optimize_cli(args):
    """命令行优化参数"""
    # 加载数据
    if args.test:
        print("使用测试数据")
        imu_data = generate_test_data(30.0)
    elif args.input:
        print(f"从 {args.input} 加载数据")
        imu_data = IMURecorder.load(args.input)
    else:
        print("错误: 需要指定 --input 或 --test")
        return

    if len(imu_data) < 100:
        print("数据量不足，需要至少100帧")
        return

    # 优化
    print(f"\n开始优化，使用 {len(imu_data)} 帧数据...")
    optimizer = ParameterOptimizer(imu_data)

    if args.method == 'de' and not HAS_SCIPY:
        args.method = 'grid'

    optimal_params, score = optimizer.optimize(method=args.method, max_iter=args.iter)

    print("\n" + "="*50)
    print("最优参数:")
    print("="*50)
    for name, value in optimal_params.to_dict().items():
        print(f"  {name}: {value}")

    print(f"\n评估得分: {score:.4f}°")

    # 保存
    with open(args.output, 'w') as f:
        json.dump(optimal_params.to_dict(), f, indent=2)
    print(f"\n参数已保存到: {args.output}")


def analyze_cli(args):
    """命令行分析参数敏感性"""
    if not HAS_MATPLOTLIB:
        print("需要安装matplotlib才能使用分析功能")
        return

    # 加载数据和参数
    imu_data = IMURecorder.load(args.input)

    with open(args.params, 'r') as f:
        params_dict = json.load(f)

    params = EKfParams()
    for k, v in params_dict.items():
        if hasattr(params, k):
            setattr(params, k, v)

    # 运行EKF
    print("运行EKF仿真...")
    ekf = PythonEKF(params)

    quat_errors = []
    motion_states = []

    for frame in imu_data:
        gyro_np = np.array(frame.gyro)
        acc_np = np.array(frame.acc)
        est_quat = ekf.update(gyro_np, acc_np)

        error = ekf._quaternion_error(est_quat, np.array(frame.quaternion))
        quat_errors.append(error)
        motion_states.append(ekf.motion_state)

    # 绘图
    fig, axes = plt.subplots(2, 1, figsize=(12, 8))

    # 姿态误差
    axes[0].plot(quat_errors)
    axes[0].set_xlabel('Frame')
    axes[0].set_ylabel('Error (deg)')
    axes[0].set_title('Attitude Estimation Error')
    axes[0].grid(True)

    # 运动状态
    state_map = {'STATIC': 0, 'STABLE': 1, 'DYNAMIC': 2}
    states = [state_map.get(s, 1) for s in motion_states]
    axes[1].plot(states)
    axes[1].set_xlabel('Frame')
    axes[1].set_ylabel('Motion State')
    axes[1].set_title('Motion State Detection')
    axes[1].set_yticks([0, 1, 2])
    axes[1].set_yticklabels(['STATIC', 'STABLE', 'DYNAMIC'])
    axes[1].grid(True)

    plt.tight_layout()
    plt.savefig('analysis.png', dpi=150)
    print("分析图已保存到 analysis.png")


# ==================== 主程序 ====================

def main():
    parser = argparse.ArgumentParser(
        description='IMU EKF参数自动整定工具',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
使用示例:
  # 采集数据
  python auto_tune_ekf.py collect -p /dev/ttyUSB0 -d 30 -o data.hdf5

  # 使用测试数据优化
  python auto_tune_ekf.py optimize --test -o params.json

  # 使用录制的数据优化
  python auto_tune_ekf.py optimize -i data.hdf5 -o params.json

  # 参数分析
  python auto_tune_ekf.py analyze -i data.hdf5 -p params.json
        """
    )

    subparsers = parser.add_subparsers(dest='command', help='子命令')

    # collect 子命令
    collect_parser = subparsers.add_parser('collect', help='采集IMU数据')
    collect_parser.add_argument('-p', '--port', default='/dev/ttyUSB0', help='串口名称')
    collect_parser.add_argument('-b', '--baudrate', type=int, default=115200, help='波特率')
    collect_parser.add_argument('-d', '--duration', type=float, default=30.0, help='采集时长(秒)')
    collect_parser.add_argument('-o', '--output', default='imu_data.hdf5', help='输出文件')
    collect_parser.add_argument('-f', '--format', default='hdf5', choices=['hdf5', 'csv'], help='输出格式')

    # optimize 子命令
    optimize_parser = subparsers.add_parser('optimize', help='优化EKF参数')
    optimize_parser.add_argument('-i', '--input', help='输入数据文件')
    optimize_parser.add_argument('--test', action='store_true', help='使用测试数据')
    optimize_parser.add_argument('-o', '--output', default='ekf_params.json', help='输出参数文件')
    optimize_parser.add_argument('-m', '--method', default='de', choices=['de', 'grid'], help='优化方法')
    optimize_parser.add_argument('--iter', type=int, default=30, help='迭代次数')

    # analyze 子命令
    analyze_parser = subparsers.add_parser('analyze', help='分析参数性能')
    analyze_parser.add_argument('-i', '--input', required=True, help='输入数据文件')
    analyze_parser.add_argument('-p', '--params', required=True, help='参数文件')

    args = parser.parse_args()

    if args.command == 'collect':
        collect_data_cli(args)
    elif args.command == 'optimize':
        optimize_cli(args)
    elif args.command == 'analyze':
        analyze_cli(args)
    else:
        parser.print_help()


if __name__ == '__main__':
    main()

# IMU EKF Auto-Tuning Tool

Receives IMU raw data via serial port and performs offline optimization of adaptive EKF parameters.

## Features

| Feature | Description |
|---------|-------------|
| Serial Data Collection | Real-time IMU data reception from embedded devices |
| Data Recording & Playback | HDF5/CSV format storage, supports offline analysis |
| EKF Offline Simulation | Python-based EKF implementation for parameter evaluation |
| Parameter Optimization | Differential Evolution / Grid Search algorithms for auto-tuning |
| Visualization Analysis | Attitude error curves, motion state detection visualization |

## Installation

```bash
pip install numpy scipy h5py matplotlib serial
```

## Serial Communication Protocol

Embedded device must output data in the following format (43 bytes/frame):

| Field | Offset | Length | Description |
|-------|--------|--------|-------------|
| Header | 0 | 2 | 0xAA 0x55 |
| Gyro | 2 | 12 | 3×float [deg/s] |
| Acc | 14 | 12 | 3×float [g] |
| Quaternion | 26 | 16 | 4×float [w,x,y,z] |
| Checksum | 42 | 1 | Checksum |

### C Code Example

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

    // Checksum
    uint8_t sum = 0;
    for(int i = 0; i < 42; i++) sum += buffer[i];
    buffer[42] = sum;

    serial_write(buffer, 43);
}
```

## Usage

### 1. Data Collection

```bash
python tools/auto_tune_ekf.py collect -p /dev/ttyUSB0 -d 30 -o data.hdf5
```

Parameters:
- `-p, --port`: Serial port name (default: /dev/ttyUSB0)
- `-b, --baudrate`: Baud rate (default: 115200)
- `-d, --duration`: Collection duration in seconds (default: 30.0)
- `-o, --output`: Output file (default: imu_data.hdf5)
- `-f, --format`: Output format hdf5/csv (default: hdf5)

### 2. Parameter Optimization

```bash
# Use test data (for development/debugging)
python tools/auto_tune_ekf.py optimize --test -o params.json

# Use recorded data
python tools/auto_tune_ekf.py optimize -i data.hdf5 -o params.json
```

Parameters:
- `-i, --input`: Input data file
- `--test`: Use built-in test data
- `-o, --output`: Output parameter file (default: ekf_params.json)
- `-m, --method`: Optimization method de/grid (default: de)
- `--iter`: Number of iterations (default: 30)

### 3. Parameter Analysis

```bash
python tools/auto_tune_ekf.py analyze -i data.hdf5 -p params.json
```

Generates analysis plots:
- Attitude estimation error curves
- Motion state detection results

## Data Collection Recommendations

| Scenario | Duration | Motion Description |
|----------|----------|---------------------|
| Static Calibration | 10s | Completely still, placed stably |
| Dynamic Testing | 20s | Various attitude changes, rotations |
| Impact Testing | 10s | Impact on IMU by tapping |
| Linear Motion | 10s | Push/pull (translation) |

Recommended total collection duration: 30-60 seconds, covering all typical operating conditions.

## Optimization Algorithms

### Differential Evolution (DE)

Global optimization algorithm, suitable for high-dimensional parameter spaces:
- Automatic exploration of optimal solutions
- Insensitive to initial values
- Strong robustness

### Grid Search

Enumeration-based search, suitable for lower dimensional parameters:
- Simple and reliable
- Can be parallelized
- Suitable for small-scale parameters

## Parameter Description

The optimizer searches for the following parameters:

| Parameter | Range | Description |
|-----------|-------|-------------|
| processNoiseAngle | 1e-7 ~ 1e-3 | Angle process noise |
| processNoiseBias | 1e-10 ~ 1e-5 | Bias process noise |
| measureNoiseAcc | 1e-4 ~ 1e-1 | Accelerometer measurement noise |
| accStaticThreshold | 0.01 ~ 0.05 | Static detection acceleration threshold |
| gyroStaticThreshold | 0.1 ~ 2.0 | Static detection gyro threshold |
| impactAccThreshold | 0.2 ~ 1.0 | Impact detection acceleration threshold |
| impactGyroThreshold | 50 ~ 200 | Impact detection gyro threshold |

## Output Format

Optimized parameters are saved as JSON:

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

## FAQ

### 1. Cannot Open Serial Port
- Check serial permissions: `sudo chmod 666 /dev/ttyUSB0`
- Confirm baud rate matches

### 2. Optimization Takes Too Long
- Reduce iterations: `--iter 10`
- Use grid search: `-m grid`
- Reduce data volume

### 3. Poor Optimization Results
- Increase data collection duration and coverage
- Check reference attitude data quality
- Try different optimization algorithms

## Related Documentation

- [EKF Algorithm Details](../README.md#2-extended-kalman-filter-ekf)
- [C Implementation of EKF](../EKF_Version/Tactical_Fusion.cpp)

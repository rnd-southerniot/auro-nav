# Project Memory

## Origin
- Firmware forked from auro-classic Phase 1 (Feb 22, 2026)
- auro-classic repo remains untouched as Phase 1 baseline

## Stable Decisions
- Gate-based development: stop on FAIL, transcript evidence required
- `HARDWARE_MAP.md` is hardware source of truth
- `protocol/ndjson_v1.md` is wire contract
- PicoSDK is the production firmware track (MicroPython frozen in auro-classic)

## Validated Hardware/Firmware Facts
- Board: Cytron Maker Pi RP2040
- Encoder pins: left GP2/GP3, right GP4/GP5
- Motor pins: GP8-GP11
- IMU: ICM-20948 on I2C1 GP6/GP7, confirmed with i2c_scan
- UART bridge: RP2040 GP0(TX)/GP1(RX) ↔ Pi5 GPIO14(TX)/GPIO15(RX), 115200 baud

## Validated Build Facts
- PicoSDK build: local vendored toolchain + SDK under third_party/
- Build output: firmware_sdk/build/airborne_classic_fw.uf2
- Flash: picotool reboot -u -f, copy UF2 to RPI-RP2

## Encoder Calibration
- Left counts/rev: 582 (trials: 576, 586, 585)
- Right counts/rev: 583 (trials: 575, 591, 584)
- Sign: ENC_L_SIGN=+1, ENC_R_SIGN=-1

## PID Controller
- Defaults: Kp=2.0, Ki=8.0, Kd=0.05
- Integral clamp: ±1.0
- Control loop: 50 Hz (20ms)
- Runtime tunable via set_pid command

## IMU
- ICM-20948, gyro-only, 500 dps, 65.5 LSB/dps
- Auto-calibrate at boot: 250 samples (~0.5s)
- cal_imu command: 500 samples (~1s)
- Yaw integration: 50 Hz, 0.5 dps dead-zone, wraps [-180, 180]

## ROS2 Stack (Pi 5)
- OS: Ubuntu Server 24.04, ROS2 Jazzy
- LiDAR: RPLidar C1 via sllidar_ros2 driver
- SLAM: slam_toolbox async mode
- Nav2: AMCL + DWB controller + NavFn planner
- Bridge: auro_bridge_node (UART, /cmd_vel ↔ set_rpm, tele ↔ /odom+TF)
- Visualization: Foxglove on Mac

## Current Priorities
1. Validate PID closed-loop on bench (Gates F2-0 through F2-3)
2. Wire UART to Pi 5 and validate bridge (Gate F2-4)
3. Build map with mobile SLAM
4. Validate Nav2 autonomous navigation end-to-end

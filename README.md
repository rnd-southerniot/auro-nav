# auro-nav — Autonomous Navigation for Airborne Classic

Closed-loop PID motor control, ICM-20948 IMU, and ROS2 Nav2 autonomous navigation stack for the Airborne Classic XRP-style robot car.

**Upstream**: Forked firmware from [auro-classic](https://github.com/rnd-southerniot/auro-classic) Phase 1 (open-loop motors, PIO encoders, NDJSON protocol). This repo extends it with PID, IMU, and full Nav2 integration — auro-classic remains untouched as the validated Phase 1 baseline.

## Architecture

```
RPLidar C1 → Pi 5 (ROS2 Jazzy: SLAM / Nav2) ↔ UART ↔ RP2040 (PID + IMU + Motors) → Foxglove (Mac)
```

## Current Status

- **Firmware v2**: PID closed-loop speed control (Kp=0.003, Ki=0.002, FF=0.004), ICM-20948 gyro Z yaw tracking, runtime PID tuning via `set_pid`
- **ROS2 bridge**: `auro_bridge_node` translates `/cmd_vel` ↔ `set_rpm` and `tele` ↔ `/odom` + TF — verified on Pi 5 at 19.6 Hz
- **Nav2**: Full navigation stack with AMCL, DWB controller, NavFn planner, costmaps tuned for RPLidar C1
- **Web teleop**: `web_teleop_node` — browser-based driving UI at `http://<pi5-ip>:8080` (WASD/touch/D-pad, no SSH needed)
- **Gates passed**: F2-0 through F2-5 (re-verified on v2 firmware), F2-6a (SLAM mapping PASS)
- **Next**: F2-6b Nav2 autonomous navigation test

## Repository Layout

```
firmware_sdk/          RP2040 PicoSDK firmware (PID + IMU + NDJSON)
  src/main.c           Main firmware (v2)
  src/quadrature_sample.pio   PIO encoder sampler (from auro-classic)
  CMakeLists.txt       Build config (+hardware_i2c)

ros2_ws/src/auro_nav/  ROS2 Jazzy package (runs on Pi 5)
  auro_nav/            Python package
    auro_bridge_node.py   UART ↔ ROS2 bridge
    web_teleop_node.py    Browser-based teleop (http://pi5:8080)
  config/              YAML configs
    nav2_params.yaml      Nav2 full config
    slam_params.yaml      slam_toolbox config
  launch/              Launch files
    auro_nav2_launch.py   Full Nav2 navigation
    mobile_slam_launch.py SLAM with mobile base
  maps/                Saved maps (.pgm + .yaml)
  udev/                udev rules for Pi 5

protocol/              NDJSON serial protocol spec
docs/                  Phase status and notes
tools/                 Host utilities
```

## Quick Start: Firmware

```bash
# One-time: vendor SDK + toolchain (see auro-classic README for details)
export PICO_SDK_PATH="$PWD/third_party/pico-sdk"
export PICO_TOOLCHAIN_PATH="$PWD/third_party/arm-gnu-toolchain-.../bin"

cmake -S firmware_sdk -B firmware_sdk/build -G Ninja
cmake --build firmware_sdk/build -j

# Flash: picotool reboot -u -f, then copy .uf2 to RPI-RP2
```

## Quick Start: ROS2 (Pi 5)

```bash
# Build workspace
cd ros2_ws
colcon build --packages-select auro_nav
source install/setup.bash

# Build a map (drive with web teleop)
ros2 launch auro_nav mobile_slam_launch.py
ros2 run auro_nav web_teleop_node  # open http://<pi5-ip>:8080

# Autonomous navigation (with saved map)
ros2 launch auro_nav auro_nav2_launch.py map:=/path/to/my_room.yaml
```

## Hardware

- **Controller**: Cytron Maker Pi RP2040
- **IMU**: ICM-20948 on I2C0 (GP16/GP17), addr 0x69
- **Motors**: Dual H-bridge on GP8-GP11, 20kHz PWM (clamp at 0.45 — reversal above ~0.50)
- **Encoders**: Quadrature via PIO (GP2/3 left, GP4/5 right)
- **LiDAR**: Slamtec RPLidar C1 (USB to Pi 5, /dev/rplidar)
- **Compute**: Raspberry Pi 5, Ubuntu Server 24.04, ROS2 Jazzy
- **Transport**: USB CDC (/dev/ttyACM0) — UART (GP0/GP1 ↔ Pi5 GPIO14/15) available as alternate

See `HARDWARE_MAP.md` for full pin assignments and kinematics.

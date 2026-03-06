# auro-nav — Repo Scaffold & Integration Prompt

## What This Is

You are creating and pushing a NEW standalone repository called `auro-nav` under
`rnd-southerniot`. This repo contains the RP2040 firmware (PID + IMU) and the ROS2
bridge node that connects the Airborne Classic robot car to the Pi 5 navigation stack.

## Relationship to Other Repos

| Repo | Role | Status |
|------|------|--------|
| `rnd-southerniot/auro-classic` | Phase 1 firmware: open-loop motors, PIO encoders, NDJSON protocol | FROZEN — do not modify |
| `rnd-southerniot/pi5-lidar` | Pi 5 ROS2 Jazzy stack: RPLidar C1, slam_toolbox, ROS2 workspace (`lidar_slam_project` package) | EXISTING — already deployed on Pi 5 |
| `rnd-southerniot/auro-nav` | **THIS REPO** — PID firmware v2, IMU, UART bridge, Nav2 launch files | NEW — you are creating this |

The `auro-nav` ROS2 package (`auro_nav`) will be built alongside `lidar_slam_project`
in the same ROS2 workspace on the Pi 5. The launch files in `auro-nav` reference
`sllidar_ros2` (the RPLidar C1 driver already cloned in `pi5-lidar`'s workspace).

## Pre-condition

All source files are already present in this directory tree. Every file has been
reviewed against `auro-classic/HARDWARE_MAP.md`, the NDJSON protocol spec, and the
Pi 5 ROS2 Jazzy environment. Your job is to:

1. Create the GitHub repo and push
2. Validate firmware builds (if toolchain is available)
3. Validate the ROS2 package integrates with the existing Pi 5 workspace
4. Update docs as specified

## Step 1: Create repo and push

```bash
cd /path/to/auro-nav
git init
git add .
git commit -m "feat: initial scaffold — PID firmware v2 + ROS2 Nav2 bridge

Firmware (RP2040 PicoSDK):
- Closed-loop PID speed control (50Hz, Kp=2.0/Ki=8.0/Kd=0.05)
- ICM-20948 IMU gyro Z yaw tracking with hardware calibration
- Runtime PID tuning via set_pid command
- Backward-compatible with auro-classic NDJSON protocol v1

ROS2 package 'auro_nav' (Pi 5):
- auro_bridge_node: UART bridge (cmd_vel <-> set_rpm, tele <-> odom+TF)
- Nav2 config: AMCL + DWB + NavFn, tuned for RPLidar C1 (12m range)
- Launch files: mobile SLAM mapping and full Nav2 autonomous navigation
- Designed to coexist with lidar_slam_project package from pi5-lidar repo

Firmware forked from rnd-southerniot/auro-classic Phase 1.
Integrates with rnd-southerniot/pi5-lidar for SLAM and LiDAR driver."

gh repo create rnd-southerniot/auro-nav --private --source=. --push
```

## Step 2: Validate firmware build

```bash
# Only if third_party/pico-sdk and ARM toolchain are vendored locally:
export PICO_SDK_PATH="$PWD/third_party/pico-sdk"
export PICO_TOOLCHAIN_PATH="$PWD/third_party/arm-gnu-toolchain-15.2.rel1-darwin-arm64-arm-none-eabi/bin"

cmake -S firmware_sdk -B firmware_sdk/build -G Ninja
cmake --build firmware_sdk/build -j

# PASS criteria: zero errors, airborne_classic_fw.uf2 produced
# If toolchain not vendored yet, skip and note in output.
```

## Step 3: Deploy ROS2 package to Pi 5 workspace

The Pi 5 already has a ROS2 workspace at `~/ros2_ws/` from the `pi5-lidar` project.
The `auro_nav` package must be cloned into the same workspace alongside the existing
`lidar_slam_project` and `sllidar_ros2` packages.

```bash
# On Pi 5 (via SSH):
cd ~/ros2_ws/src

# Clone auro-nav and symlink the ROS2 package into the workspace
git clone https://github.com/rnd-southerniot/auro-nav.git
ln -s auro-nav/ros2_ws/src/auro_nav auro_nav

# Verify workspace structure before building:
# ~/ros2_ws/src/
# ├── sllidar_ros2/            (from pi5-lidar, already exists)
# ├── lidar_slam_project/      (from pi5-lidar, already exists)
# ├── auro-nav/                (this repo, just cloned)
# └── auro_nav -> auro-nav/ros2_ws/src/auro_nav  (symlink)

# Install Python dependency
pip install pyserial --break-system-packages

# Build
cd ~/ros2_ws
colcon build --packages-select auro_nav
source install/setup.bash

# PASS criteria:
ros2 pkg executables auro_nav
# Expected output: auro_nav auro_bridge_node

ros2 launch auro_nav --list
# Expected: auro_nav2_launch.py, mobile_slam_launch.py
```

## Step 4: Install udev rules on Pi 5

```bash
# RPLidar C1 (may already exist from pi5-lidar setup)
sudo cp ~/ros2_ws/src/auro_nav/udev/99-rplidar.rules /etc/udev/rules.d/

# UART symlink for RP2040
sudo cp ~/ros2_ws/src/auro_nav/udev/99-auro.rules /etc/udev/rules.d/

sudo udevadm control --reload-rules && sudo udevadm trigger

# PASS criteria:
ls -la /dev/rplidar   # symlink to ttyUSBx
ls -la /dev/auro      # symlink to ttyAMA0
```

## Step 5: Enable UART on Pi 5

```bash
# Add to /boot/firmware/config.txt:
echo "enable_uart=1" | sudo tee -a /boot/firmware/config.txt

# Remove serial console from cmdline.txt:
# Edit /boot/firmware/cmdline.txt and remove 'console=serial0,115200'
sudo sed -i 's/console=serial0,115200 //' /boot/firmware/cmdline.txt

sudo reboot

# After reboot, verify:
ls -la /dev/ttyAMA0
# PASS criteria: device exists
```

## Step 6: Validate end-to-end (gate-based)

Execute IN ORDER. Stop on FAIL.

### Gate F2-0: Firmware PID build
- Rebuild firmware with USB CDC stdio (bench testing)
- Flash UF2 to RP2040
- PASS: boot telemetry shows `pid:true`

### Gate F2-1: PID closed-loop
```
{"type":"cmd","id":"1","ts":0,"cmd":"arm"}
{"type":"cmd","id":"2","ts":0,"cmd":"set_rpm","left":60,"right":60}
```
- PASS: `rpm_l` and `rpm_r` converge to ~60 within 2 seconds
- Then: `stop`, `disarm`

### Gate F2-2: IMU calibration
```
{"type":"cmd","id":"5","ts":0,"cmd":"cal_imu"}
```
- Robot MUST be stationary
- PASS: ack returns `ok:true` with `bias_z` value < 2.0 dps
- Rotate robot by hand → `yaw` field changes smoothly in telemetry

### Gate F2-3: UART switch
- In `firmware_sdk/CMakeLists.txt`, change to UART stdio:
  ```
  pico_enable_stdio_uart(airborne_classic_fw 1)
  pico_enable_stdio_usb(airborne_classic_fw 0)
  ```
- Rebuild, reflash
- Wire: Pi5 GPIO14(TX) → RP2040 GP1(RX), Pi5 GPIO15(RX) → RP2040 GP0(TX), GND↔GND
- PASS: `minicom -D /dev/ttyAMA0 -b 115200` shows boot telemetry

### Gate F2-4: ROS2 bridge
```bash
ros2 run auro_nav auro_bridge_node --ros-args -p serial_port:=/dev/ttyAMA0
```
- PASS: `ros2 topic list` shows `/odom`, `/cmd_vel`
- PASS: `ros2 topic echo /odom` shows updating pose
- PASS: `ros2 run tf2_tools view_frames` shows `odom → base_link`

### Gate F2-5: Mobile SLAM
```bash
ros2 launch auro_nav mobile_slam_launch.py
```
- Drive with `ros2 run teleop_twist_keyboard teleop_twist_keyboard`
- PASS: map builds in Foxglove
- Save map: `ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/src/auro_nav/maps/my_room --ros-args -p map_subscribe_transient_local:=true`

### Gate F2-6: Nav2 autonomous navigation
```bash
ros2 launch auro_nav auro_nav2_launch.py map:=/path/to/my_room.yaml
```
- PASS: all Nav2 lifecycle nodes reach Active state
- PASS: send goal from Foxglove `/goal_pose` → robot navigates autonomously

## Key Cross-Repo Dependencies

The `auro_nav` launch files reference these packages that live in `pi5-lidar`'s workspace:

| Package | Source | Used By |
|---------|--------|---------|
| `sllidar_ros2` | `~/ros2_ws/src/sllidar_ros2/` (cloned in pi5-lidar setup) | `auro_nav2_launch.py`, `mobile_slam_launch.py` — includes C1 launch |
| `nav2_bringup` | `apt: ros-jazzy-nav2-bringup` | `auro_nav2_launch.py` — includes bringup_launch.py |
| `slam_toolbox` | `apt: ros-jazzy-slam-toolbox` | `mobile_slam_launch.py` — async_slam_toolbox_node |

If any of these are missing on the Pi 5, install them:
```bash
sudo apt install -y ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox
# sllidar_ros2 should already be in ~/ros2_ws/src/ from pi5-lidar setup
```

## Constraints

- Do NOT modify `auro-classic` repo — it is frozen at Phase 1
- Do NOT modify `pi5-lidar` repo — it has the working SLAM stack
- Do NOT modify `quadrature_sample.pio` — validated and locked
- All HARDWARE_MAP.md pin assignments are LOCKED
- Firmware is backward-compatible with NDJSON protocol v1
- Test each gate before proceeding. Stop on FAIL.
- Use conventional commits: `feat:`, `fix:`, `docs:`, `chore:` scoped by layer

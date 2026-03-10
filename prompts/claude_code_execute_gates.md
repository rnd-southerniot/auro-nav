# auro-nav — Gate Execution Prompt (F2-0 through F2-6)

## Context

The `auro-nav` repo is pushed and live at https://github.com/rnd-southerniot/auro-nav.
Step 1 (scaffold + push) is PASS. You are now executing the hardware validation gates.

Related repos (DO NOT modify):
- `rnd-southerniot/auro-classic` — Phase 1 baseline (frozen)
- `rnd-southerniot/pi5-lidar` — Pi 5 SLAM stack (ROS2 package: `lidar_slam_project`)

Hardware source of truth: `HARDWARE_MAP.md` in the auro-nav repo.
Serial protocol: `protocol/ndjson_v1.md` in the auro-nav repo.

## Rules

1. Execute gates IN ORDER. Do not skip ahead.
2. Stop on FAIL. Report what failed, what you observed, and propose a fix.
3. Every gate must produce transcript evidence (command + output).
4. Commit passing gate evidence to `docs/GATE_LOG.md` after each PASS.
5. Use conventional commits scoped by layer: `docs(gates):`, `fix(firmware):`, etc.

---

## Gate F2-0: Firmware build

```bash
cd /path/to/auro-nav

# Vendor SDK if not already present
if [ ! -d "third_party/pico-sdk" ]; then
  mkdir -p third_party
  git clone --depth 1 https://github.com/raspberrypi/pico-sdk.git third_party/pico-sdk
  git -C third_party/pico-sdk submodule update --init --recursive
fi

# Vendor ARM toolchain if not already present (macOS ARM example)
if [ ! -d "third_party/arm-gnu-toolchain-15.2.rel1-darwin-arm64-arm-none-eabi" ]; then
  curl -fL -o /tmp/arm-gnu-toolchain.tar.xz \
    https://developer.arm.com/-/media/Files/downloads/gnu/15.2.rel1/binrel/arm-gnu-toolchain-15.2.rel1-darwin-arm64-arm-none-eabi.tar.xz
  tar -xf /tmp/arm-gnu-toolchain.tar.xz -C third_party
fi

export PICO_SDK_PATH="$PWD/third_party/pico-sdk"
export PICO_TOOLCHAIN_PATH="$PWD/third_party/arm-gnu-toolchain-15.2.rel1-darwin-arm64-arm-none-eabi/bin"

# Clean build
rm -rf firmware_sdk/build
cmake -S firmware_sdk -B firmware_sdk/build -G Ninja
cmake --build firmware_sdk/build -j
```

**PASS criteria:**
- Zero build errors
- File exists: `firmware_sdk/build/airborne_classic_fw.uf2`
- Record: `ls -la firmware_sdk/build/airborne_classic_fw.uf2`

**On FAIL:** Read the compiler error. Most likely causes:
- Missing `hardware/i2c.h` → check `target_link_libraries` includes `hardware_i2c`
- PIO header not found → check `pico_generate_pio_header` path in CMakeLists.txt

---

## Gate F2-1: Flash and PID validation (USB CDC)

Ensure CMakeLists.txt has USB CDC mode (should be default):
```
pico_enable_stdio_uart(airborne_classic_fw 0)
pico_enable_stdio_usb(airborne_classic_fw 1)
```
If changed, rebuild first.

```bash
# Force BOOTSEL and flash
picotool reboot -u -f
# Wait 2 seconds for RPI-RP2 to mount, then:
cp firmware_sdk/build/airborne_classic_fw.uf2 /Volumes/RPI-RP2/
# (on Linux: cp ... /media/$USER/RPI-RP2/)
```

Wait for reboot (~2 seconds). Open serial monitor:
```bash
# macOS:
screen /dev/cu.usbmodem* 115200
# or: minicom -D /dev/cu.usbmodem* -b 115200
```

**Test sequence** — type each line, observe response:

```json
{"type":"cmd","id":"1","ts":0,"cmd":"ping"}
```
Expected: `{"type":"ack","id":"1","ok":true}`

```json
{"type":"cmd","id":"2","ts":0,"cmd":"version"}
```
Expected: ack with `"fw":"sdk-v2-pid"`

```json
{"type":"cmd","id":"3","ts":0,"cmd":"arm"}
```
Expected: ack ok

```json
{"type":"cmd","id":"4","ts":0,"cmd":"set_rpm","left":60,"right":60}
```
Watch telemetry for 3-5 seconds. Expected:
- `pid:true` in telemetry
- `rpm_l` and `rpm_r` converge toward 60 within 2 seconds
- `pwm_l` and `pwm_r` are non-zero and adjusting (PID working)

```json
{"type":"cmd","id":"5","ts":0,"cmd":"stop"}
{"type":"cmd","id":"6","ts":0,"cmd":"disarm"}
```

**PASS criteria:**
- ping/version ack OK
- Telemetry shows `pid:true`
- RPM converges to target within 2 seconds
- Motors stop on `stop` command
- Record 5+ telemetry lines as evidence

**On FAIL — RPM does not converge:**
- If RPM oscillates wildly: Kp too high. Try:
  ```json
  {"type":"cmd","id":"7","ts":0,"cmd":"set_pid","kp":1.0,"ki":5.0,"kd":0.0}
  ```
  Then re-test set_rpm.
- If RPM stays at 0: check motor wiring, check `armed` is true in telemetry
- If RPM overshoots then settles: that's OK, PID is working, just needs tuning

---

## Gate F2-2: IMU calibration

Robot MUST be stationary and on a flat surface.

```json
{"type":"cmd","id":"10","ts":0,"cmd":"cal_imu"}
```

Wait ~1 second for calibration to complete.

Expected: `{"type":"ack","id":"10","ok":true,"data":{"bias_z":-0.xxxx,"samples":500}}`

**Verify yaw tracking:**
- Slowly rotate robot 90° clockwise by hand
- Watch `yaw` field in telemetry — should change by approximately -90
- Rotate back — should return toward 0

**PASS criteria:**
- cal_imu ack returns ok:true with bias_z value (should be < 2.0 dps absolute)
- yaw changes smoothly on rotation
- yaw is stable (drift < 1 degree) when stationary over 10 seconds
- Record: cal_imu ack + 3 tele lines at rest + 3 tele lines after rotation

**On FAIL — `no_imu` error:**
- IMU not detected. Check I2C wiring: SDA=GP6, SCL=GP7
- Verify with MicroPython i2c_scan if needed (from auro-classic firmware)
- Check AD0 strap on breakout board (determines 0x68 vs 0x69)

---

## Gate F2-3: Runtime PID tuning

```json
{"type":"cmd","id":"20","ts":0,"cmd":"set_pid","kp":3.0,"ki":10.0,"kd":0.1}
```

Expected: `{"type":"ack","id":"20","ok":true,"data":{"kp":3.000,"ki":10.000,"kd":0.100}}`

Then re-test motor response:
```json
{"type":"cmd","id":"21","ts":0,"cmd":"arm"}
{"type":"cmd","id":"22","ts":0,"cmd":"set_rpm","left":80,"right":80}
```
Watch for convergence, then stop and disarm.

**PASS criteria:**
- set_pid ack echoes new gains correctly
- Motors respond with new PID gains
- Record ack + 3 tele lines showing convergence

**After tuning:** Record the best PID gains in `MEMORY.md`:
```markdown
## Tuned PID Gains
- Kp=X.X, Ki=X.X, Kd=X.X (tuned [date])
- Test: set_rpm 80/80, convergence within Xs, steady-state error < X RPM
```

---

## Gate F2-4: UART switch and Pi 5 wiring

### 4a: Rebuild firmware for UART

Edit `firmware_sdk/CMakeLists.txt`:
```cmake
# Change FROM:
pico_enable_stdio_uart(airborne_classic_fw 0)
pico_enable_stdio_usb(airborne_classic_fw 1)

# Change TO:
pico_enable_stdio_uart(airborne_classic_fw 1)
pico_enable_stdio_usb(airborne_classic_fw 0)
```

Rebuild and flash:
```bash
cmake --build firmware_sdk/build -j
picotool reboot -u -f
cp firmware_sdk/build/airborne_classic_fw.uf2 /Volumes/RPI-RP2/
```

### 4b: Wire UART to Pi 5

Physical connections (3 wires):
```
Pi 5 GPIO14 (TXD, pin 8)  ───→  RP2040 GP1 (UART0 RX)
Pi 5 GPIO15 (RXD, pin 10) ←───  RP2040 GP0 (UART0 TX)
Pi 5 GND    (pin 6)       ←──→  RP2040 GND
```

**CRITICAL:** TX crosses to RX. Pi transmits on GPIO14, RP2040 receives on GP1.

### 4c: Enable UART on Pi 5

```bash
# SSH into Pi 5
ssh youruser@pi5-ros.local

# Enable UART
echo "enable_uart=1" | sudo tee -a /boot/firmware/config.txt

# Remove serial console (so our bridge can use the port)
sudo sed -i 's/console=serial0,115200 //' /boot/firmware/cmdline.txt

sudo reboot
```

### 4d: Verify UART

After reboot and with RP2040 powered + wired:
```bash
minicom -D /dev/ttyAMA0 -b 115200
```

Expected: NDJSON telemetry streaming. Type ping command to test bidirectional:
```json
{"type":"cmd","id":"1","ts":0,"cmd":"ping"}
```

**PASS criteria:**
- Boot telemetry visible in minicom
- ping ack returns ok:true
- Record 3 tele lines + ping ack as evidence

**On FAIL — no output in minicom:**
- Check TX/RX crossover wiring (most common mistake)
- Verify `enable_uart=1` in config.txt
- Verify `console=serial0,115200` removed from cmdline.txt
- Check RP2040 is powered and firmware was rebuilt with UART stdio

Commit CMakeLists change:
```bash
git add firmware_sdk/CMakeLists.txt
git commit -m "feat(firmware): switch stdio to UART for Pi 5 bridge"
git push
```

---

## Gate F2-5: ROS2 bridge on Pi 5

### 5a: Deploy auro_nav package

```bash
# On Pi 5:
cd ~/ros2_ws/src

# Clone if not already present
if [ ! -d "auro-nav" ]; then
  git clone https://github.com/rnd-southerniot/auro-nav.git
fi

# Create symlink so colcon finds the package
ln -sf auro-nav/ros2_ws/src/auro_nav auro_nav

# Install dependency
pip install pyserial --break-system-packages

# Build
cd ~/ros2_ws
colcon build --packages-select auro_nav
source install/setup.bash
```

### 5b: Install udev rules

```bash
sudo cp ~/ros2_ws/src/auro_nav/udev/99-rplidar.rules /etc/udev/rules.d/
sudo cp ~/ros2_ws/src/auro_nav/udev/99-auro.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger

# Verify:
ls -la /dev/rplidar /dev/auro
```

### 5c: Test bridge node standalone

```bash
source ~/ros2_ws/install/setup.bash

ros2 run auro_nav auro_bridge_node --ros-args \
  -p serial_port:=/dev/ttyAMA0 \
  -p baud_rate:=115200
```

In another terminal:
```bash
# Check topics
ros2 topic list
# Expected: /odom, /cmd_vel (among others)

# Check odom is publishing
ros2 topic hz /odom
# Expected: ~20 Hz

# Check TF
ros2 run tf2_tools view_frames
# Expected: odom -> base_link

# Test motor control
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.1}, angular: {z: 0.0}}"
# Expected: robot moves forward briefly, then watchdog stops it
```

**PASS criteria:**
- /odom publishing at ~20 Hz
- TF tree shows odom → base_link
- cmd_vel drives the motors
- Watchdog stops robot after 0.5s of silence
- Record: topic list, hz output, view_frames output

---

## Gate F2-6a: Mobile SLAM (map building)

```bash
source ~/ros2_ws/install/setup.bash

# Launch SLAM with mobile base
ros2 launch auro_nav mobile_slam_launch.py
```

In another terminal, drive with keyboard:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

On your Mac, open Foxglove and connect to the Pi 5. Add panels for:
- /map (Map)
- /scan (LaserScan)
- /tf (TF)

Drive the robot slowly around the room. The map should build progressively.

When the map looks complete, save it:
```bash
ros2 run nav2_map_server map_saver_cli \
  -f ~/ros2_ws/src/auro-nav/ros2_ws/src/auro_nav/maps/my_room \
  --ros-args -p map_subscribe_transient_local:=true
```

**PASS criteria:**
- Map builds in Foxglove as robot moves
- Walls and obstacles are clearly visible
- map_saver produces my_room.pgm + my_room.yaml
- Record: screenshot from Foxglove (or describe map quality)

Commit the map:
```bash
cd ~/ros2_ws/src/auro-nav
git add ros2_ws/src/auro_nav/maps/
git commit -m "feat(maps): first room map from mobile SLAM"
git push
```

---

## Gate F2-6b: Nav2 autonomous navigation

```bash
source ~/ros2_ws/install/setup.bash

# Launch Nav2 with saved map
ros2 launch auro_nav auro_nav2_launch.py \
  map:=$(ros2 pkg prefix auro_nav)/share/auro_nav/maps/my_room.yaml
```

Wait for all Nav2 lifecycle nodes to reach Active state. Check:
```bash
ros2 lifecycle list /controller_server
ros2 lifecycle list /planner_server
ros2 lifecycle list /amcl
# All should show: active [1]
```

### Set initial pose (if not auto-set):
```bash
ros2 topic pub --once /initialpose \
  geometry_msgs/PoseWithCovarianceStamped \
  "{header: {frame_id: 'map'}, pose: {pose: {position: {x: 0.0, y: 0.0}, orientation: {w: 1.0}}}}"
```

Drive the robot briefly with teleop to help AMCL converge (particle cloud tightens).

### Send navigation goal:
From Foxglove: use the Publish panel on `/goal_pose` topic.
Or from terminal:
```bash
ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}}}"
```

**PASS criteria:**
- All lifecycle nodes Active
- AMCL particle cloud converges
- Robot plans a path (visible in /plan topic)
- Robot navigates to goal autonomously
- Robot avoids obstacles encountered en route
- Record: lifecycle states, describe navigation behavior

**On FAIL:**
- Lifecycle nodes stuck → check map loaded, TF tree complete
- No path found → check costmap inflation_radius vs robot_radius
- Robot oscillates → reduce max_vel_x, increase xy_goal_tolerance
- AMCL diverges → drive with teleop first, increase alpha1-4

---

## Post-Gate: Update docs and commit

After all gates PASS, update and commit:

### docs/GATE_LOG.md
```markdown
# Gate Log

## F2-0: Firmware build — PASS
- Date: [date]
- UF2 size: [size]
- Build: zero errors

## F2-1: PID validation — PASS
- Date: [date]
- Target: 60 RPM, converged in [X]s
- Steady-state error: [X] RPM
- PID gains: Kp=[X] Ki=[X] Kd=[X]

## F2-2: IMU calibration — PASS
- Date: [date]
- Bias Z: [X] dps
- Yaw drift at rest: [X] deg/10s

## F2-3: PID tuning — PASS
- Final gains: Kp=[X] Ki=[X] Kd=[X]

## F2-4: UART bridge — PASS
- Date: [date]
- Bidirectional NDJSON confirmed

## F2-5: ROS2 bridge — PASS
- /odom rate: [X] Hz
- TF: odom → base_link confirmed

## F2-6a: Mobile SLAM — PASS
- Map: [filename]
- Quality: [brief description]

## F2-6b: Nav2 navigation — PASS
- Goal reached: yes/no
- Obstacle avoidance: yes/no
```

### MEMORY.md
Add tuned PID gains and any calibration values discovered during testing.

```bash
git add docs/ MEMORY.md
git commit -m "docs(gates): F2-0 through F2-6 PASS evidence"
git push
```

---

## Summary: What Success Looks Like

When all gates pass, you have:
- RP2040 running closed-loop PID at 50 Hz with IMU yaw tracking
- UART serial link from RP2040 to Pi 5
- `auro_bridge_node` translating ROS2 ↔ NDJSON at 20 Hz
- RPLidar C1 providing 360° scans at 10 Hz
- slam_toolbox building maps from LiDAR + encoder odometry
- Nav2 autonomously navigating to goals with AMCL localization + DWB control
- Three repos cleanly separated: firmware baseline (auro-classic), navigation firmware (auro-nav), LiDAR stack (pi5-lidar)

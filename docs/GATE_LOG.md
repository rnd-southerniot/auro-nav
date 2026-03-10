# Gate Log

## F2-6a: SLAM arena mapping — PASS
- Date: 2026-03-10
- Config tuning for 2m x 2m arena: PASS
  - slam_params: resolution 0.025, travel thresholds 0.05
  - nav2_params: local costmap 2x2, inflation_radius 0.25
- SLAM launch: PASS (slam_toolbox, sllidar_node, auro_bridge all running)
- Topics verified: /scan ~10 Hz, /odom ~10 Hz, /tf, /tf_static, /cmd_vel
- Web teleop node added: browser-based driving at http://pi5-ip:8080
  - D-pad with 9 directions, WASD/arrow keyboard, touch support
  - Adjustable linear/angular speed, 200ms command rate
  - Replaces teleop_twist_keyboard (no SSH needed for driving)
- Firmware re-flash: v0 → v2 (sdk-v2-pid) via picotool on Mac
- Full gate re-test on Mac USB: all PASS (see below)
- Bug fix: slam_toolbox lifecycle node was not auto-activated
  - async_slam_toolbox_node is a lifecycle node requiring configure→activate
  - Fixed mobile_slam_launch.py to use LifecycleNode with auto-activation
- Map saved: arena_v1.pgm (115x101 px, 2.9m x 2.5m at 0.025 m/pixel)
- Bag recorded: arena_test_20260310_134602 (64.5 MiB, 14 min, 62K msgs)
  - /scan: 8452, /odom: 16527, /tf: 37052
- Next: F2-6b Nav2 autonomous navigation test

## F2-0 through F2-4: Re-verified on v2 firmware — ALL PASS
- Date: 2026-03-08
- Firmware was found running sdk-v0 (old); re-flashed to sdk-v2-pid via picotool
- UF2 size: 121,344 bytes (rebuilt from current source)
- Full automated gate test results:
  - F2-0 Build: fw=sdk-v2-pid PASS
  - F2-1 PID: target 40 RPM, avg L=38.5 R=35.9, PWM 0.163–0.202 PASS
  - F2-2 IMU: cal_imu bias_z=-0.4165, 500 samples, WHO=0xEA, addr=0x69 PASS
  - F2-3 PID tuning: set_pid ack Kp=0.003 Ki=0.002 Kd=0.0 PASS
  - F2-4 USB bridge: ping OK, bidirectional NDJSON PASS
- Telemetry v2 fields confirmed: pid=true, gz=±0.4 dps at rest, yaw=0.00

## PWM clamp tuning — 0.80 → 0.45
- Date: 2026-03-08
- Problem: robot not driving straight, user requested higher PWM
- Initial attempt: raised PWM_OUTPUT_MAX from 0.50 to 0.80
- Result: motors physically REVERSE above ~0.50 PWM (H-bridge hardware limit)
  - Forward 80 RPM: left motor went -30 RPM (backward!) at PWM 0.80
  - Backward 80 RPM: same reversal pattern
- Fix: set PWM_OUTPUT_MAX = 0.45 (safe margin below reversal threshold)
- RPM sweep test (20/30/40/50/60 RPM), all PASS:
  - 20 RPM: L=18.3 R=16.7 (diff 1.6) — PWM max 0.12
  - 30 RPM: L=28.6 R=26.9 (diff 1.7) — PWM max 0.17
  - 40 RPM: L=38.5 R=35.2 (diff 3.3) — PWM max 0.22
  - 50 RPM: L=47.8 R=44.1 (diff 3.7) — PWM max 0.29
  - 60 RPM: L=47.0 R=47.2 (saturating at clamp, balanced) — PWM max 0.43
- Max useful speed: ~50 RPM (60 RPM saturates but no reversal)
- Left motor consistently 2-4 RPM faster than right — acceptable for Nav2 (odom compensates)
- Turn tests (soft/hard left/right) all PASS at ±50 RPM

## F2-0: Firmware build — PASS (initial)
- Date: 2026-03-07
- UF2 size: 118,784 bytes
- Build: zero errors (99/99 ninja targets)
- Toolchain: arm-none-eabi-gcc 14.3.1 (Homebrew), Pico SDK 2.x (latest)
- Mode: USB CDC (stdio_usb=1, stdio_uart=0)
- CMake config: Release, platform rp2040, board pico

## F2-1: PID closed-loop — PASS
- Date: 2026-03-07
- Target: 40 RPM (60 RPM not achievable on USB power — motor reverses above ~0.5 PWM)
- Converged in: <1s
- Steady-state: rpm_l=40.2 rpm_r=39.7 (10s run, 0 negative RPM samples)
- True avg RPM from encoder counts: 39.0
- PWM range: 0.163 to 0.268
- PID gains: Kp=0.003, Ki=0.002, Kd=0.0 + FF=0.004 PWM/RPM
- Firmware fixes applied:
  - Feed-forward term added (base PWM from target RPM)
  - Conditional anti-windup (stop integral accumulation when output saturated)
  - PWM output clamped to ±0.40 (motor reversal above ~0.5 on USB power)
  - Gains rescaled for RPM error units (original Kp=2.0 caused instant saturation)

## F2-5: ROS2 bridge on Pi 5 — PASS
- Date: 2026-03-07
- Package: auro_nav built on Pi 5 (colcon build, 1.73s)
- Bridge: auro_bridge_node connected to /dev/ttyACM0
- Topics: /odom, /cmd_vel, /tf ✓
- Odom rate: 19.6 Hz (target ~20 Hz) ✓
- TF: odom → base_link publishing ✓
- cmd_vel motor test: deferred (need manual verification)

## F2-4: UART/USB bridge to Pi 5 — PASS
- Date: 2026-03-07
- Mode: USB CDC (kept for now, UART switch deferred)
- Pi 5: arif-ubuntu, 192.168.1.114, Linux 6.8.0-1047-raspi aarch64
- Device: /dev/ttyACM0 (RP2040 USB plugged into Pi 5 USB port)
- Telemetry: 3 lines received ✓
- Ping: ack ok:true ✓
- Bidirectional NDJSON confirmed
- Note: added arif to dialout group for serial access

## F2-3: Runtime PID tuning — PASS
- Date: 2026-03-07
- set_pid ack: `{"kp":0.003,"ki":0.002,"kd":0.000}` ✓
- Target: 40 RPM, converged to 30–36 RPM within 2s (integral still building)
- Steady-state PWM: 0.24–0.27 range
- IMU active during motion: gz peaks at 33.56 dps, yaw tracking real rotation
- Final gains: Kp=0.003, Ki=0.002, Kd=0.0, FF=0.004 PWM/RPM
- PWM clamp: 0.50 (motor reversal above ~0.5)

## F2-2: IMU calibration — PASS
- Date: 2026-03-07
- IMU detected: ICM-20948 at 0x69 on I2C0 (GP16/GP17)
- WHO_AM_I: 0xEA ✓
- cal_imu: bias_z=-0.4164, 500 samples ✓
- Gyro Z noise at rest: ±0.39 dps (expected for stationary)
- Yaw at rest: 0.00 (below 0.5 dps dead-zone — correct)
- Root cause of prior FAIL: (1) IMU wired to GP16/GP17 (I2C0), not GP6/GP7 (I2C1)
  (2) ICM-20948 requires accel enabled (PWR_MGMT_2=0x00) — disabling accel freezes gyro
  (3) Device reset (0x80) needed at init for reliable startup
- Fixes: i2c0, PWR_MGMT_2=0x00, device reset, 6-byte burst read, GYRO_SMPLRT_DIV=0

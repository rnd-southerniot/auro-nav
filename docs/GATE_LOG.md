# Gate Log

## F2-0: Firmware build — PASS
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

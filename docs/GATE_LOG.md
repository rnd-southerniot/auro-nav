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

## F2-2: IMU calibration — FAIL (hardware not connected)
- Date: 2026-03-07
- Error: `no_imu` — ICM-20948 not detected on I2C1 (GP6/GP7)
- Yaw: stuck at 0.00 (no IMU data)
- Action: verify I2C wiring (SDA=GP6, SCL=GP7, 3.3V, GND)
- Note: ROS2 bridge can operate without IMU using encoder-only odometry

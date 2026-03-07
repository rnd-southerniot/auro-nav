# NDJSON Serial Protocol v1 (Airborne Classic)

Transport: USB CDC serial
Encoding: newline-delimited JSON (one JSON object per line)

## Commands (Hub → Controller)

All commands include:
- `type: "cmd"`
- `id`: string UUID
- `ts`: epoch milliseconds
- `cmd`: command name

### Arm/disarm
```json
{"type":"cmd","id":"uuid","ts":0,"cmd":"arm"}
{"type":"cmd","id":"uuid","ts":0,"cmd":"disarm"}
```

### Motor control
Open loop PWM (normalized -1..1):
```json
{"type":"cmd","id":"uuid","ts":0,"cmd":"set_pwm","left":0.2,"right":0.2}
```

RPM command:
```json
{"type":"cmd","id":"uuid","ts":0,"cmd":"set_rpm","left":120,"right":120}
```
Firmware v2 uses closed-loop PID with feed-forward to achieve target RPM.

Emergency stop:
```json
{"type":"cmd","id":"uuid","ts":0,"cmd":"stop"}
```

### Calibration
```json
{"type":"cmd","id":"uuid","ts":0,"cmd":"cal_imu"}
{"type":"cmd","id":"uuid","ts":0,"cmd":"cal_encoders"}
```

### Introspection
```json
{"type":"cmd","id":"uuid","ts":0,"cmd":"ping"}
{"type":"cmd","id":"uuid","ts":0,"cmd":"i2c_scan"}
{"type":"cmd","id":"uuid","ts":0,"cmd":"version"}
```

### Logging
Default is **NDJSON only**. Human logs can be enabled for debugging.

```json
{"type":"cmd","id":"uuid","ts":0,"cmd":"set_log","on":true}
{"type":"cmd","id":"uuid","ts":0,"cmd":"set_log","on":false}
```

Human log lines are prefixed with `# ` so parsers can ignore them.

## Telemetry (Controller → Hub)

Telemetry at 10–50 Hz depending mode.

```json
{
  "type":"tele",
  "ts":0,
  "mode":"DISARMED|ARMED|TELEOP|AUTO|FAULT",
  "armed":false,
  "pwm_l":0.0,
  "pwm_r":0.0,
  "enc_l":0,
  "enc_r":0,
  "rpm_l":0.0,
  "rpm_r":0.0,
  "yaw":0.0,
  "fault":null
}
```

## Acks (Controller → Hub)

```json
{"type":"ack","id":"uuid","ok":true}
```

Error:
```json
{"type":"ack","id":"uuid","ok":false,"error":{"code":"bad_cmd","message":"..."}}
```

## Implementation Status (March 7, 2026)

Protocol v1 is the contract; firmware tracks it in staged subsets.

- PicoSDK (`firmware_sdk/src/main.c`) implements:
  - `ping`, `version`, `arm`, `disarm`, `stop`, `set_pwm`, `set_rpm` (closed-loop PID), `cal_encoders`, `cal_imu` (500-sample gyro bias), `set_pid`, `imu_diag`

## v2 Additions (auro-nav)

### PID tuning command
```json
{"type":"cmd","id":"uuid","ts":0,"cmd":"set_pid","kp":0.003,"ki":0.002,"kd":0.0}
```
Ack returns: `{"ok":true,"data":{"kp":0.003,"ki":0.002,"kd":0.000}}`
Resets PID integral and previous error on both wheels.
Default gains: Kp=0.003, Ki=0.002, Kd=0.0, feed-forward=0.004 PWM/RPM.

### IMU diagnostic command
```json
{"type":"cmd","id":"uuid","ts":0,"cmd":"imu_diag"}
```
Ack returns register readback and two raw gyro samples 20ms apart for debugging.

### Updated `cal_imu` (was noop, now real)
Robot must be stationary. Collects 500 gyro Z samples over ~1 second.
```json
{"type":"cmd","id":"uuid","ts":0,"cmd":"cal_imu"}
```
Ack returns: `{"ok":true,"data":{"bias_z":-0.234,"samples":500}}`

### Updated telemetry fields
```json
{
  "type":"tele",
  "ts":0,
  "mode":"DISARMED|ARMED",
  "armed":false,
  "pwm_l":0.0,
  "pwm_r":0.0,
  "enc_l":0,
  "enc_r":0,
  "rpm_l":0.0,
  "rpm_r":0.0,
  "yaw":0.0,
  "pid":true,
  "gz":0.0,
  "fault":null
}
```
- `pid` (boolean): true when closed-loop PID active, false when open-loop (set_pwm mode)
- `yaw` (float): integrated gyro Z heading in degrees [-180, 180], 0.0 if IMU not calibrated
- `gz` (float): raw gyro Z in dps (bias-compensated), useful for IMU debugging

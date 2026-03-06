# auro-nav — HARDWARE_MAP (source of truth)

## Chassis / Kinematics (from auro-classic CAD)

- Wheel diameter: **54.875 mm** (treat as 55 mm)
- Wheel radius: **27.4375 mm**
- Track width (wheel-center to wheel-center): **155.0 mm**
- Wheelbase (drive axle center → caster contact center): **62 mm**

## Controller: Cytron Maker Pi RP2040

### Motor driver pins (LOCKED)
- M1A = GP8  (left forward)
- M1B = GP9  (left backward)
- M2A = GP10 (right forward)
- M2B = GP11 (right backward)

Truth table: 00=brake, 10=forward, 01=backward, 11=coast
PWM frequency: 20 kHz, wrap=6249

### Onboard peripherals
- WS2812 RGB: GP18
- Buttons: GP20, GP21
- Buzzer: GP22
- Servo signals: GP12–GP15

## Encoders (quadrature, PIO-based)

### Pin assignment (LOCKED)
- Left:  ENC_L_A = GP2, ENC_L_B = GP3
- Right: ENC_R_A = GP4, ENC_R_B = GP5

### Calibration (from auro-classic, Feb 22 2026)
- Left counts/rev: **582** (trials: 576, 586, 585)
- Right counts/rev: **583** (trials: 575, 591, 584)

### Sign convention
- ENC_L_SIGN = +1, ENC_R_SIGN = -1 (forward = positive both channels)

## IMU: ICM-20948

### I2C bus (LOCKED)
- I2C0: SDA=GP16, SCL=GP17, 400kHz
- Address: 0x68 (AD0=GND) or 0x69 (AD0=VCC), auto-detected

### Configuration
- Gyro + accel enabled (accel must stay enabled — disabling it freezes gyro output)
- Device reset (0x80) at init required for reliable startup
- Full scale: 500 dps (sensitivity: 65.5 LSB/dps)
- DLPF enabled, GYRO_SMPLRT_DIV=0 (ODR=1125 Hz)
- Calibration: 500-sample bias at boot

## UART Bridge: RP2040 ↔ Pi 5

### Pin assignment (LOCKED)
- RP2040 GP0 (UART0 TX) → Pi 5 GPIO15 (RXD)
- RP2040 GP1 (UART0 RX) ← Pi 5 GPIO14 (TXD)
- GND ↔ GND

### Settings
- Baud: 115200
- Pi 5 device: /dev/ttyAMA0 (symlink: /dev/auro)

### Firmware stdio mode
- UART mode (production): `pico_enable_stdio_uart=1, pico_enable_stdio_usb=0`
- USB CDC mode (bench dev): `pico_enable_stdio_uart=0, pico_enable_stdio_usb=1`

## RPLidar C1 (on Pi 5)

- USB to Pi 5 (blue USB 3.0 port)
- Adapter: C1M1 (CP2102)
- Device: /dev/rplidar (udev symlink)
- Baud: 460800 (handled by sllidar_ros2 C1 launch)

## PID Controller Defaults

- Kp=2.0, Ki=8.0, Kd=0.05
- Integral clamp: ±1.0
- Control rate: 50 Hz (20ms period)
- Output: normalized PWM [-1, 1]

## Power
- TBD (battery type/voltage)

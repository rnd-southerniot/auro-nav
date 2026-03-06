# Repository Guidelines

## Project Structure
- `firmware_sdk/`: PicoSDK firmware (PID + IMU + NDJSON). Production track.
- `ros2_ws/src/auro_nav/`: ROS2 Jazzy package for Pi 5 (bridge, Nav2 config, launch files).
- `protocol/`: NDJSON wire contract and JSON schema.
- `tools/`: host utilities.
- `docs/`: phase status and verification reports.
- `MEMORY.md`: persistent operational context for future sessions.
- `HARDWARE_MAP.md`: hardware source of truth (pins, kinematics, calibration).

## Build Commands
- Firmware: `cmake -S firmware_sdk -B firmware_sdk/build -G Ninja && cmake --build firmware_sdk/build -j`
- Flash: `picotool reboot -u -f` then copy UF2 to RPI-RP2
- ROS2: `cd ros2_ws && colcon build --packages-select auro_nav && source install/setup.bash`
- SLAM launch: `ros2 launch auro_nav mobile_slam_launch.py`
- Nav2 launch: `ros2 launch auro_nav auro_nav2_launch.py`

## Coding Style
- C (PicoSDK): explicit fixed-width types, small static helpers, deterministic loops
- Python (ROS2): type hints, snake_case, rclpy patterns
- Bash: `set -euo pipefail`
- Conventional commits: `feat:`, `fix:`, `docs:`, `chore:` scoped by layer

## Testing
- Gate-based: stop on FAIL, include transcript evidence
- Firmware: boot tele, ping/version ack, PID convergence, IMU calibration, encoder direction
- Bridge: /odom publishing, TF tree (odom→base_link→laser), cmd_vel→motor response
- Nav2: lifecycle nodes Active, AMCL convergence, goal navigation

## Constraints
- `quadrature_sample.pio` is validated and LOCKED
- HARDWARE_MAP.md pin assignments are LOCKED
- Protocol changes must update both `ndjson_v1.md` and `ndjson_v1.schema.json`

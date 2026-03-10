# Nav2 Desk Test — Small Arena Mapping & Navigation

## Context

I am in the `auro-nav` repo. The firmware is flashed and working (gates F2-0 through F2-5 PASS).
The Pi 5 is powered via AC (USB-C wall charger with extension cord — no battery constraints).
The RP2040 motor pack is on its own battery.
The RPLidar C1 is plugged into Pi 5 USB.
UART is wired: Pi5 GPIO14(TX)→RP2040 GP1(RX), Pi5 GPIO15(RX)→RP2040 GP0(TX), GND↔GND.

I have built a small ~2m × 2m test arena using cardboard boxes as walls with 1-2 obstacles in the center.

The ROS2 workspace is at `~/ros2_ws/` on the Pi 5. The `auro_nav` package is built and sourced.
The `sllidar_ros2` package is also in the workspace (from pi5-lidar setup).

## What I need you to do

### Phase A: Tune configs for small arena

1. SSH into the Pi 5
2. Edit `~/ros2_ws/src/auro-nav/ros2_ws/src/auro_nav/config/slam_params.yaml`:
   - Set `resolution: 0.025` (2.5cm/pixel for small space detail)
   - Set `minimum_travel_distance: 0.05`
   - Set `minimum_travel_heading: 0.05`
   - Keep all other params unchanged
3. Edit `~/ros2_ws/src/auro-nav/ros2_ws/src/auro_nav/config/nav2_params.yaml`:
   - In `local_costmap` section: set `width: 2` and `height: 2`
   - Set `inflation_radius: 0.25` (in both global and local costmap)
   - Keep all other params unchanged
4. Rebuild: `cd ~/ros2_ws && colcon build --packages-select auro_nav && source install/setup.bash`

### Phase B: Map the arena (SLAM)

5. Start foxglove bridge:
   ```
   ros2 launch foxglove_bridge foxglove_bridge_launch.xml
   ```
6. In a new terminal, start SLAM:
   ```
   source ~/ros2_ws/install/setup.bash
   ros2 launch auro_nav mobile_slam_launch.py
   ```
7. Wait for all nodes to come up. Verify:
   ```
   ros2 topic list | grep -E "scan|odom|map|tf"
   ros2 topic hz /scan    # expect ~10 Hz
   ros2 topic hz /odom    # expect ~20 Hz
   ```
8. In a new terminal, start teleop:
   ```
   source ~/ros2_ws/install/setup.bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   ```
9. Tell me the teleop is ready — I will manually drive the robot around the arena.
   Remind me to:
   - Press `x` 3-4 times to reduce linear speed to ~0.1 m/s
   - Press `c` 2-3 times to reduce angular speed to ~0.3 rad/s
   - Drive slowly along all 4 walls
   - Return to starting position for loop closure
   - Then tell you when mapping is done

### Phase C: Save map and record bag

10. When I say mapping is done, start recording a bag:
    ```
    ros2 bag record /scan /odom /tf /tf_static /cmd_vel -o ~/rosbags/arena_test_$(date +%Y%m%d_%H%M%S)
    ```
11. Save the map:
    ```
    ros2 run nav2_map_server map_saver_cli \
      -f ~/ros2_ws/src/auro-nav/ros2_ws/src/auro_nav/maps/arena_v1 \
      --ros-args -p map_subscribe_transient_local:=true
    ```
12. Verify map files exist:
    ```
    ls -la ~/ros2_ws/src/auro-nav/ros2_ws/src/auro_nav/maps/arena_v1.*
    ```
13. Stop the bag recording (Ctrl+C) and verify:
    ```
    ros2 bag info ~/rosbags/arena_test_*/
    ```

### Phase D: Nav2 autonomous navigation test

14. Stop SLAM (Ctrl+C on the slam launch terminal)
15. Launch Nav2 with the saved map:
    ```
    source ~/ros2_ws/install/setup.bash
    ros2 launch auro_nav auro_nav2_launch.py \
      map:=$(ros2 pkg prefix auro_nav)/share/auro_nav/maps/arena_v1.yaml
    ```
16. Verify all lifecycle nodes are active:
    ```
    ros2 lifecycle list /amcl
    ros2 lifecycle list /controller_server
    ros2 lifecycle list /planner_server
    ```
17. Help AMCL converge — run teleop briefly:
    ```
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
    ```
    Tell me to drive forward/back for 5 seconds then stop.
18. Send a test navigation goal:
    ```
    ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped \
      "{header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 0.5, z: 0.0}, orientation: {w: 1.0}}}"
    ```
19. Monitor the robot:
    ```
    ros2 topic echo /cmd_vel
    ```
    Report whether the robot is receiving velocity commands and moving toward the goal.

### Phase E: Commit results

20. Stop all nodes (Ctrl+C everywhere)
21. Commit map and config changes:
    ```
    cd ~/ros2_ws/src/auro-nav
    git add ros2_ws/src/auro_nav/maps/ ros2_ws/src/auro_nav/config/
    git commit -m "feat(maps): arena test map + small-arena config tuning

    - slam_params: resolution 0.025, travel thresholds 0.05 for 2m arena
    - nav2_params: local costmap 2x2, inflation 0.25 for tight space
    - arena_v1 map from first successful SLAM run"
    git push
    ```
22. Update `docs/GATE_LOG.md` with F2-6a and F2-6b results, commit and push.

## Rules

- Execute phases in order A → B → C → D → E
- Stop on FAIL at any phase — report what failed and wait for my input
- Phases B and D require my manual input (driving the robot) — pause and wait for me
- Every verification command must show output before proceeding
- Do not modify firmware files — only config YAML and maps

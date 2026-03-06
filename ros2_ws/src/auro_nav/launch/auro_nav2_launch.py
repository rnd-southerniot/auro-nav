"""
Airborne Classic + Nav2 full navigation launch

Starts: RPLidar C1, auro_bridge (UART to RP2040), static TF, Nav2 stack

Usage:
  ros2 launch auro_nav auro_nav2_launch.py
  ros2 launch auro_nav auro_nav2_launch.py map:=/path/to/map.yaml
"""
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory('auro_nav')
    sllidar = get_package_share_directory('sllidar_ros2')
    nav2_bringup = get_package_share_directory('nav2_bringup')

    nav2_params = os.path.join(pkg, 'config', 'nav2_params.yaml')
    map_file = LaunchConfiguration('map')

    return LaunchDescription([
        DeclareLaunchArgument('map',
            default_value=os.path.join(pkg, 'maps', 'my_room.yaml'),
            description='Full path to map yaml'),

        # ── RPLidar C1 driver ──
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(sllidar, 'launch', 'sllidar_c1_launch.py')),
            launch_arguments={'serial_port': '/dev/rplidar'}.items(),
        ),

        # ── Airborne Classic bridge (UART to RP2040) ──
        # Publishes /odom and TF (odom->base_link)
        # Subscribes /cmd_vel and sends set_rpm to RP2040
        Node(
            package='auro_nav',
            executable='auro_bridge_node',
            name='auro_bridge',
            parameters=[{
                'serial_port': '/dev/ttyAMA0',
                'baud_rate': 115200,
                'odom_frame': 'odom',
                'base_frame': 'base_link',
                'max_linear_vel': 0.3,
                'max_angular_vel': 2.0,
                'cmd_vel_timeout': 0.5,
            }],
            output='screen',
        ),

        # ── Static TF: base_link -> laser ──
        # ADJUST these offsets to match your LiDAR mount on the chassis!
        # Measured from base_link (center of wheel axle at ground)
        # to LiDAR center. Z = height above ground.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0.08', '0', '0', '0', 'base_link', 'laser'],
            name='static_tf_base_laser',
        ),

        # ── Nav2 full navigation stack ──
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup, 'launch', 'bringup_launch.py')),
            launch_arguments={
                'map': map_file,
                'params_file': nav2_params,
                'use_sim_time': 'false',
                'autostart': 'true',
            }.items(),
        ),
    ])

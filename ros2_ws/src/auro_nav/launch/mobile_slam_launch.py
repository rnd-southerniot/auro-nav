"""
Mobile SLAM launch — drive robot with teleop, build map.
Uses auro_bridge for motor control + odometry.
"""
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory('auro_nav')
    sllidar = get_package_share_directory('sllidar_ros2')
    slam_cfg = os.path.join(pkg, 'config', 'slam_params.yaml')

    return LaunchDescription([
        # RPLidar C1
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(sllidar, 'launch', 'sllidar_c1_launch.py')),
            launch_arguments={'serial_port': '/dev/rplidar'}.items(),
        ),

        # Airborne Classic bridge (UART)
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

        # Static TF: base_link -> laser (adjust Z for your mount!)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0.08', '0', '0', '0', 'base_link', 'laser'],
            name='static_tf_base_laser',
        ),

        # SLAM Toolbox (mapping mode)
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[slam_cfg],
            output='screen',
        ),
    ])

"""Capa base de percepción y odometría del AMR (simulación o robot real).

    - ekf_filter_node : odom → base_footprint (encoders + IMU)
    - scan_merger     : /lidar_front/scan + /lidar_rear/scan → /scan (360°)
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('amr_navigation')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            parameters=[PathJoinSubstitution([pkg, 'config', 'ekf.yaml']),
                        {'use_sim_time': use_sim_time}],
            remappings=[('odometry/filtered', '/odom')],
            output='screen',
        ),
        Node(
            package='amr_navigation',
            executable='scan_merger.py',
            name='scan_merger',
            # Sin tiempo simulado: reutiliza el sello de los escaneos y así no
            # procesa /clock (1 kHz en Gazebo), que en Python cuesta mucha CPU.
            parameters=[{'use_sim_time': False}],
            output='screen',
        ),
    ])

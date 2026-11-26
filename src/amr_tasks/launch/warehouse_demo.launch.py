"""Demo completa: simulación + localización/navegación + gestor de tareas.

    ros2 launch amr_tasks warehouse_demo.launch.py
    ros2 launch amr_tasks warehouse_demo.launch.py tasks_file:=/ruta/tareas.yaml

Seguimiento:
    ros2 topic echo /amr/task_status
Añadir tareas en caliente:
    ros2 topic pub --once /amr/task_request std_msgs/String "data: 'area_2'"
    ros2 topic pub --once /amr/task_request std_msgs/String "data: '2x area_1,area_3'"
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    tasks_pkg = FindPackageShare('amr_tasks')
    sim = LaunchConfiguration('sim')

    navigation = GroupAction(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare('amr_navigation'), 'launch', 'navigation.launch.py'])),
                launch_arguments={'sim': sim, 'headless': LaunchConfiguration('headless'),
                                  'rviz': LaunchConfiguration('rviz')}.items(),
            ),
        ],
        condition=IfCondition(LaunchConfiguration('with_navigation')),
    )
    manager = Node(
        package='amr_tasks',
        executable='task_manager.py',
        name='task_manager',
        output='screen',
        parameters=[{
            'use_sim_time': sim,
            'stations_file': LaunchConfiguration('stations_file'),
            'tasks_file': LaunchConfiguration('tasks_file'),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('sim', default_value='true'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('with_navigation', default_value='true',
                              description='false: solo el gestor (Nav2 ya en marcha)'),
        DeclareLaunchArgument('stations_file', default_value=PathJoinSubstitution(
            [FindPackageShare('amr_gazebo'), 'config', 'warehouse_stations.yaml'])),
        DeclareLaunchArgument('tasks_file', default_value=PathJoinSubstitution(
            [tasks_pkg, 'config', 'tasks.yaml'])),
        navigation,
        manager,
    ])

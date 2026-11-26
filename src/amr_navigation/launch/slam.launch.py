"""SLAM: construcción del mapa del depósito con slam_toolbox (online async).

Uso (simulación):
    ros2 launch amr_navigation slam.launch.py
    # conducir: ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true
    #   o recorrido automático: ros2 run amr_navigation mapping_route.py
    # guardar:  ros2 run amr_navigation save_map.sh  [nombre]

Argumentos: sim (true), headless (false), rviz (true), x, y, yaw (spawn)
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    nav_pkg = FindPackageShare('amr_navigation')
    sim = LaunchConfiguration('sim')
    use_sim_time = LaunchConfiguration('sim')

    # GroupAction (scoped): los argumentos pasados a sim.launch.py (rviz:=false...)
    # no deben sobrescribir los de este launch.
    simulation = GroupAction(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare('amr_gazebo'), 'launch', 'sim.launch.py'])),
                launch_arguments={'headless': LaunchConfiguration('headless'), 'rviz': 'false',
                                  'x': LaunchConfiguration('x'), 'y': LaunchConfiguration('y'),
                                  'yaw': LaunchConfiguration('yaw')}.items(),
            ),
        ],
        condition=IfCondition(sim),
    )
    base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([nav_pkg, 'launch', 'base.launch.py'])),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )
    slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[PathJoinSubstitution([nav_pkg, 'config', 'slam_toolbox.yaml']),
                    {'use_sim_time': use_sim_time, 'use_lifecycle_manager': True}],
        output='screen',
    )
    # En Jazzy slam_toolbox es un nodo lifecycle: lo configura/activa el gestor
    slam_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_slam',
        parameters=[{'use_sim_time': use_sim_time, 'autostart': True,
                     'node_names': ['slam_toolbox']}],
        output='screen',
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', PathJoinSubstitution([nav_pkg, 'rviz', 'slam.rviz'])],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('sim', default_value='true'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        simulation,
        base,
        slam,
        slam_lifecycle,
        rviz,
    ])

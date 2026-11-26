"""Navegación autónoma: mapa guardado + AMCL + Nav2 (MPPI omnidireccional).

La posición del AMR en el mapa la estima AMCL en todo momento (TF map → odom).

Uso (simulación):
    ros2 launch amr_navigation navigation.launch.py
    ros2 launch amr_navigation navigation.launch.py map:=/ruta/mapa.yaml x:=.. y:=.. yaw:=..

Argumentos:
    sim (true)  headless (false)  rviz (true)
    map         mapa para AMCL (yaml de map_server), por defecto maps/warehouse.yaml
    keepout_map mapa con zonas prohibidas para los costmaps (maps/warehouse_keepout.yaml)
    x, y, yaw   pose inicial (spawn en Gazebo y pose inicial de AMCL)
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    nav_pkg = FindPackageShare('amr_navigation')
    params = PathJoinSubstitution([nav_pkg, 'config', 'nav2_params.yaml'])
    sim = LaunchConfiguration('sim')
    use_sim_time = LaunchConfiguration('sim')
    x, y, yaw = LaunchConfiguration('x'), LaunchConfiguration('y'), LaunchConfiguration('yaw')

    # GroupAction (scoped): los argumentos pasados a sim.launch.py (rviz:=false...)
    # no deben sobrescribir los de este launch.
    simulation = GroupAction(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare('amr_gazebo'), 'launch', 'sim.launch.py'])),
                launch_arguments={'headless': LaunchConfiguration('headless'), 'rviz': 'false',
                                  'x': x, 'y': y, 'yaw': yaw}.items(),
            ),
        ],
        condition=IfCondition(sim),
    )
    base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([nav_pkg, 'launch', 'base.launch.py'])),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    common = [params, {'use_sim_time': use_sim_time}]

    # --- Localización: mapa + AMCL
    localization = [
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             parameters=common + [{'yaml_filename': LaunchConfiguration('map')}], output='screen'),
        # Mapa con zonas prohibidas para los costmaps (AMCL usa el mapa SLAM real)
        Node(package='nav2_map_server', executable='map_server', name='keepout_map_server',
             parameters=[{'use_sim_time': use_sim_time,
                          'yaml_filename': LaunchConfiguration('keepout_map')}],
             remappings=[('map', 'map_keepout')], output='screen'),
        Node(package='nav2_amcl', executable='amcl', name='amcl', output='screen',
             parameters=common + [{
                 'set_initial_pose': True,
                 'initial_pose.x': ParameterValue(x, value_type=float),
                 'initial_pose.y': ParameterValue(y, value_type=float),
                 'initial_pose.yaw': ParameterValue(yaw, value_type=float),
             }]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_localization', output='screen',
             parameters=[{'use_sim_time': use_sim_time, 'autostart': True,
                          'node_names': ['map_server', 'keepout_map_server', 'amcl']}]),
    ]

    # --- Navegación (cadena de velocidad: controller/behaviors → cmd_vel_nav
    #     → velocity_smoother → /cmd_vel → mecanum_drive_controller)
    navigation = [
        Node(package='nav2_controller', executable='controller_server', output='screen',
             parameters=common, remappings=[('cmd_vel', 'cmd_vel_nav')]),
        Node(package='nav2_planner', executable='planner_server', name='planner_server',
             parameters=common, output='screen'),
        Node(package='nav2_behaviors', executable='behavior_server', name='behavior_server',
             parameters=common, remappings=[('cmd_vel', 'cmd_vel_nav')], output='screen'),
        Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator',
             parameters=common, output='screen'),
        Node(package='nav2_velocity_smoother', executable='velocity_smoother',
             name='velocity_smoother', parameters=common, output='screen',
             remappings=[('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', output='screen',
             parameters=[{'use_sim_time': use_sim_time, 'autostart': True,
                          'node_names': ['controller_server', 'planner_server', 'behavior_server',
                                         'bt_navigator', 'velocity_smoother']}]),
    ]

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', PathJoinSubstitution([nav_pkg, 'rviz', 'navigation.rviz'])],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('sim', default_value='true'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('map', default_value=PathJoinSubstitution(
            [nav_pkg, 'maps', 'warehouse.yaml'])),
        DeclareLaunchArgument('keepout_map', default_value=PathJoinSubstitution(
            [nav_pkg, 'maps', 'warehouse_keepout.yaml'])),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        simulation,
        base,
        *localization,
        *navigation,
        rviz,
    ])

"""Simulación del AMR en Gazebo Harmonic (solo robot + mundo).

Para SLAM / navegación / tareas usar los launch de amr_navigation y amr_tasks,
que incluyen este y añaden EKF, fusión de LiDAR, mapa y Nav2.

Lanza: Gazebo + spawn del robot + robot_state_publisher + puente ROS-GZ
       + controladores (joint_state_broadcaster, mecanum_drive_controller, lift_controller)
       + RViz (opcional).

Uso:
    ros2 launch amr_gazebo sim.launch.py
    ros2 launch amr_gazebo sim.launch.py headless:=true rviz:=false
    ros2 launch amr_gazebo sim.launch.py x:=-14.0 y:=-6.6 yaw:=-1.5708   # frente a la estación 1

Mover el robot (TwistStamped en /cmd_vel):
    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true
Elevar la plataforma 1.0 m (a 0.05 m/s):
    ros2 run amr_gazebo lift_cmd.py 1.0
"""
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription, RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (Command, LaunchConfiguration, PathJoinSubstitution,
                                  PythonExpression)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    desc_pkg = FindPackageShare('amr_description')
    gz_pkg = FindPackageShare('amr_gazebo')

    world = LaunchConfiguration('world')
    headless = LaunchConfiguration('headless')
    rviz = LaunchConfiguration('rviz')
    x, y, yaw = LaunchConfiguration('x'), LaunchConfiguration('y'), LaunchConfiguration('yaw')

    controllers_file = PathJoinSubstitution([gz_pkg, 'config', 'amr_controllers.yaml'])

    robot_description = ParameterValue(
        Command([
            'xacro ', PathJoinSubstitution([desc_pkg, 'urdf', 'amr.urdf.xacro']),
            ' use_sim:=true',
            ' controllers_file:=', controllers_file,
        ]),
        value_type=str,
    )

    gz_args = [world, ' -r -v 2', PythonExpression(["' -s' if '", headless, "' == 'true' else ''"])]

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])),
        launch_arguments={'gz_args': gz_args, 'on_exit_shutdown': 'true'}.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        output='screen',
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'amr',
                   '-x', x, '-y', y, '-z', '0.02', '-Y', yaw],
        output='screen',
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': PathJoinSubstitution([gz_pkg, 'config', 'gz_bridge.yaml']),
            'use_sim_time': True,
        }],
        output='screen',
    )

    def spawner(name, extra=None):
        args = [name, '--controller-manager', '/controller_manager']
        if extra:
            args += extra
        return Node(package='controller_manager', executable='spawner',
                    arguments=args, parameters=[{'use_sim_time': True}], output='screen')

    jsb = spawner('joint_state_broadcaster')
    drive = spawner('mecanum_drive_controller', [
        '--controller-ros-args',
        '-r /mecanum_drive_controller/reference:=/cmd_vel '
        '-r /mecanum_drive_controller/odometry:=/odom/wheel',
    ])
    lift = spawner('lift_controller')

    lift_relay = Node(
        package='amr_gazebo',
        executable='lift_stage_relay.py',
        parameters=[{'use_sim_time': False}],   # no necesita /clock
        output='screen',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', PathJoinSubstitution([desc_pkg, 'rviz', 'sim.rviz'])],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(rviz),
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=PathJoinSubstitution(
            [gz_pkg, 'worlds', 'warehouse.sdf'])),
        DeclareLaunchArgument('headless', default_value='false'),
        # RViz propio de la simulación (fixed frame odom: requiere el EKF de
        # amr_navigation). Normalmente se usa el RViz de slam/navigation.launch.py
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),

        # Rutas para model:// (mallas del depósito y del robot)
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', PathJoinSubstitution([gz_pkg, '..'])),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', PathJoinSubstitution([desc_pkg, '..'])),

        gazebo,
        robot_state_publisher,
        spawn,
        bridge,
        lift_relay,
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[jsb])),
        RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=[drive, lift])),
        rviz_node,
    ])

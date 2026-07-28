"""Gazebo Harmonic + robot + surrogate dynamics, with no navigation stack.

Use this on its own to check that the boat drives:

    ros2 launch roboboat_bringup sim.launch.py
    ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \\
        '{linear: {x: 1.0}, angular: {z: 0.2}}'
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    description_share = get_package_share_directory('roboboat_description')
    control_share = get_package_share_directory('roboboat_control')

    default_world = os.path.join(description_share, 'worlds', 'roboboat_course.sdf')
    default_bridge = os.path.join(description_share, 'config', 'gz_bridge.yaml')
    default_boat_params = os.path.join(control_share, 'config', 'boat_params.yaml')
    xacro_file = os.path.join(description_share, 'urdf', 'roboboat.urdf.xacro')

    args = [
        DeclareLaunchArgument('world', default_value=default_world,
                              description='World SDF to load.'),
        DeclareLaunchArgument('headless', default_value='false',
                              description='Run gz server only, with headless '
                                          'rendering for the GPU lidar.'),
        DeclareLaunchArgument('boat_params', default_value=default_boat_params),
        DeclareLaunchArgument('bridge_config', default_value=default_bridge),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('mode', default_value='dynamic',
                              description="'dynamic' (thrusters + 3-DOF model) "
                                          "or 'kinematic' (twist passthrough)."),
        DeclareLaunchArgument('markers', default_value='true',
                              description='Publish thrust-vector markers for RViz.'),
        DeclareLaunchArgument('chase_camera', default_value='false',
                              description='Mount a follow camera on the boat. '
                                          'Rendering it is expensive and Nav2 '
                                          'does not use it; for footage only.'),
    ]

    # gz sim. '-r' starts unpaused; verbosity 2 keeps the console readable.
    # Headless still renders (offscreen) because the GPU lidar needs it.
    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])),
        launch_arguments={
            'gz_args': [LaunchConfiguration('world'), ' -r -v 2'],
        }.items(),
        condition=UnlessCondition(LaunchConfiguration('headless')),
    )
    gz_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])),
        launch_arguments={
            'gz_args': [LaunchConfiguration('world'),
                        ' -r -v 2 -s --headless-rendering'],
        }.items(),
        condition=IfCondition(LaunchConfiguration('headless')),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            # value_type=str matters: without it launch_ros tries to infer a
            # type from the XML and the description can arrive mangled.
            'robot_description': ParameterValue(
                Command(['xacro ', xacro_file, ' chase_camera:=',
                         LaunchConfiguration('chase_camera')]), value_type=str),
        }],
    )

    # The model name MUST stay 'roboboat': the VelocityControl and
    # OdometryPublisher plugins in the xacro address topics under
    # /model/roboboat/, and so does gz_bridge.yaml.
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', 'roboboat',
            '-topic', 'robot_description',
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', LaunchConfiguration('z'),
            '-Y', LaunchConfiguration('yaw'),
        ],
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[{
            'config_file': LaunchConfiguration('bridge_config'),
            'use_sim_time': True,
        }],
    )

    dynamics = Node(
        package='roboboat_control',
        executable='surrogate_dynamics',
        name='surrogate_dynamics',
        output='screen',
        parameters=[
            LaunchConfiguration('boat_params'),
            {'use_sim_time': True, 'mode': LaunchConfiguration('mode')},
        ],
    )

    markers = Node(
        package='roboboat_control',
        executable='thruster_markers',
        name='thruster_markers',
        output='screen',
        parameters=[
            LaunchConfiguration('boat_params'),
            {'use_sim_time': True},
        ],
        condition=IfCondition(LaunchConfiguration('markers')),
    )

    return LaunchDescription(
        args + [gz, gz_headless, robot_state_publisher, spawn, bridge,
                dynamics, markers])

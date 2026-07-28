"""Everything: Gazebo, the boat, Nav2 with MPPI, and RViz.

    ros2 launch roboboat_bringup boat_nav.launch.py

Then send a goal from RViz's "2D Goal Pose" tool, or from the CLI:

    ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \\
      "{pose: {header: {frame_id: map}, pose: {position: {x: 20.0, y: 6.0},
        orientation: {w: 1.0}}}}"
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    bringup_share = get_package_share_directory('roboboat_bringup')
    description_share = get_package_share_directory('roboboat_description')
    launch_dir = os.path.join(bringup_share, 'launch')
    rviz_config = os.path.join(description_share, 'rviz', 'roboboat_nav2.rviz')

    args = [
        DeclareLaunchArgument('headless', default_value='false',
                              description='Skip the Gazebo GUI. RViz is still '
                                          'launched unless rviz:=false.'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('mode', default_value='dynamic'),
        DeclareLaunchArgument('use_velocity_smoother', default_value='false'),
        DeclareLaunchArgument('nav2_params', default_value=os.path.join(
            bringup_share, 'config', 'nav2_mppi.yaml')),
    ]

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'sim.launch.py')),
        launch_arguments={
            'headless': LaunchConfiguration('headless'),
            'mode': LaunchConfiguration('mode'),
        }.items(),
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'nav2.launch.py')),
        launch_arguments={
            'params_file': LaunchConfiguration('nav2_params'),
            'use_velocity_smoother': LaunchConfiguration('use_velocity_smoother'),
        }.items(),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription(args + [sim, nav2, rviz])

"""Nav2 for open water: no map server, no AMCL, both costmaps rolling.

Deliberately spelled out node by node instead of including
nav2_bringup/navigation_launch.py. That launch file also brings up
route_server, collision_monitor and docking_server; every one of those is a
lifecycle node, and a single failure to configure takes the whole stack down
with it. None of them are needed for this proof of concept, so they are not
here. Add them back explicitly when you have a reason to.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# Bond heartbeats are timed on the node clock, which here is sim time. When
# the sim clock stutters -- and under software rendering it does -- a server
# can miss the 4 s default while working perfectly well, at which point the
# lifecycle manager declares CRITICAL FAILURE and tears the whole stack down
# mid-goal. 0.0 disables the check (nav2_lifecycle_manager returns early from
# createBondTimer when this is <= 0). Set it back to 4.0 on the real vehicle,
# where a genuinely dead server is a safety concern and the clock is real.
BOND_TIMEOUT = 0.0

LIFECYCLE_NODES = [
    'controller_server',
    'smoother_server',
    'planner_server',
    'behavior_server',
    'bt_navigator',
    'waypoint_follower',
]


def generate_launch_description() -> LaunchDescription:
    bringup_share = get_package_share_directory('roboboat_bringup')
    default_params = os.path.join(bringup_share, 'config', 'nav2_mppi.yaml')
    default_bt = os.path.join(bringup_share, 'behavior_trees',
                              'navigate_to_pose_boat.xml')
    default_through_bt = os.path.join(bringup_share, 'behavior_trees',
                                      'navigate_through_poses_boat.xml')

    params_file = LaunchConfiguration('params_file')
    use_smoother = LaunchConfiguration('use_velocity_smoother')
    # A bare LaunchConfiguration reaches the node as the string 'true',
    # which lifecycle_manager rejects because it declares a bool.
    autostart = ParameterValue(LaunchConfiguration('autostart'), value_type=bool)

    args = [
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('bt_xml', default_value=default_bt),
        DeclareLaunchArgument('through_bt_xml', default_value=default_through_bt),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('use_velocity_smoother', default_value='false',
                              description='Insert nav2_velocity_smoother between '
                                          'the controller and the boat. Off by '
                                          'default: MPPI already outputs smooth '
                                          'commands and the extra stage adds '
                                          'latency its model does not know about.'),
        DeclareLaunchArgument('publish_map_odom_tf', default_value='true',
                              description='Publish a static identity map->odom. '
                                          'Turn off once you feed a real global '
                                          'estimate (GPS/EKF) into that link.'),
        DeclareLaunchArgument('log_level', default_value='info'),
    ]

    common = [params_file, {'use_sim_time': True}]
    log = ['--ros-args', '--log-level', LaunchConfiguration('log_level')]

    # Open water has no features to localise against and no prior map, so the
    # map frame is simply where the boat booted. Everything downstream still
    # works because both costmaps roll with the vehicle.
    map_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom',
        output='screen',
        arguments=['--frame-id', 'map', '--child-frame-id', 'odom'],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('publish_map_odom_tf')),
    )

    # Without the smoother the controller drives /cmd_vel directly. With it,
    # the controller and the recovery behaviors move to /cmd_vel_nav and the
    # smoother republishes onto /cmd_vel, which is what the boat listens to.
    direct = [('cmd_vel', '/cmd_vel')]
    via_smoother = [('cmd_vel', '/cmd_vel_nav')]

    def velocity_producer(package, executable, name):
        return [
            Node(package=package, executable=executable, name=name,
                 output='screen',
                 parameters=common, arguments=log, remappings=direct,
                 condition=UnlessCondition(use_smoother)),
            Node(package=package, executable=executable, name=name,
                 output='screen',
                 parameters=common, arguments=log, remappings=via_smoother,
                 condition=IfCondition(use_smoother)),
        ]

    nodes = []
    nodes += velocity_producer('nav2_controller', 'controller_server',
                               'controller_server')
    nodes += velocity_producer('nav2_behaviors', 'behavior_server',
                               'behavior_server')

    nodes.append(Node(
        package='nav2_smoother', executable='smoother_server',
        name='smoother_server', output='screen',
        parameters=common, arguments=log))

    nodes.append(Node(
        package='nav2_planner', executable='planner_server',
        name='planner_server', output='screen',
        parameters=common, arguments=log))

    nodes.append(Node(
        package='nav2_bt_navigator', executable='bt_navigator',
        name='bt_navigator', output='screen',
        parameters=common + [{
            'default_nav_to_pose_bt_xml': LaunchConfiguration('bt_xml'),
            'default_nav_through_poses_bt_xml':
                LaunchConfiguration('through_bt_xml'),
        }],
        arguments=log))

    nodes.append(Node(
        package='nav2_waypoint_follower', executable='waypoint_follower',
        name='waypoint_follower', output='screen',
        parameters=common, arguments=log))

    nodes.append(Node(
        package='nav2_velocity_smoother', executable='velocity_smoother',
        name='velocity_smoother', output='screen',
        parameters=common, arguments=log,
        remappings=[('cmd_vel', '/cmd_vel_nav'),
                    ('cmd_vel_smoothed', '/cmd_vel')],
        condition=IfCondition(use_smoother)))

    manager_nodes = LIFECYCLE_NODES + ['velocity_smoother']

    lifecycle = [
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', output='screen',
             parameters=[{'use_sim_time': True,
                          'autostart': autostart,
                          'bond_timeout': BOND_TIMEOUT,
                          'node_names': LIFECYCLE_NODES}],
             condition=UnlessCondition(use_smoother)),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', output='screen',
             parameters=[{'use_sim_time': True,
                          'autostart': autostart,
                          'bond_timeout': BOND_TIMEOUT,
                          'node_names': manager_nodes}],
             condition=IfCondition(use_smoother)),
    ]

    return LaunchDescription(args + [map_odom_tf] + nodes + lifecycle)

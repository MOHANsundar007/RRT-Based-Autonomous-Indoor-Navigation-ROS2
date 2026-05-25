#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_name = 'path_planning_pkg'
    pkg_dir  = get_package_share_directory(pkg_name)

    config_dir = os.path.join(pkg_dir, 'config')
    maps_dir   = os.path.join(pkg_dir, 'maps')
    rviz_dir   = os.path.join(pkg_dir, 'rviz')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    declare_world_arg = DeclareLaunchArgument(
        'world_file',
        default_value=os.path.join(pkg_dir, 'worlds', 'path_planning.world'),
        description='Full path to the Gazebo world file',
    )
    declare_map_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(maps_dir, 'map_1777558867.yaml'),
        description='Full path to map yaml file to load',
    )
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true',
    )

    world_file    = LaunchConfiguration('world_file')
    map_yaml_file = LaunchConfiguration('map')

    gzserver = ExecuteProcess(
        cmd=[
            'gzserver', '--verbose',
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
            world_file,
        ],
        output='screen',
    )
    gzclient = ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
    )

    urdf_path = os.path.join(pkg_dir, 'urdf', 'diffbot.urdf')
    with open(urdf_path, 'r') as f:
        robot_desc = f.read()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_desc,
        }],
    )

    spawn_entity = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-topic', 'robot_description',
                    '-entity', 'diffbot',
                    '-x', '-4.0', '-y', '-3.60', '-z', '0.1',
                    '-Y', '1.5708',   
                ],
                output='screen',
            )
        ],
    )

    map_server = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                parameters=[
                    {'yaml_filename': map_yaml_file},
                    {'use_sim_time': use_sim_time},
                ],
            )
        ],
    )
    amcl = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='nav2_amcl',
                executable='amcl',
                name='amcl',
                output='screen',
                parameters=[
                    os.path.join(config_dir, 'amcl.yaml'),
                    {'use_sim_time': use_sim_time},
                ],
                remappings=[('scan', '/scan')],
            )
        ],
    )

    lifecycle_manager_localization = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                output='screen',
                parameters=[
                    {'use_sim_time': use_sim_time},
                    {'autostart': True},
                    {'node_names': ['map_server', 'amcl']},
                    {'bond_timeout': 4.0},
                ],
            )
        ],
    )

    planner_server = TimerAction(
        period=12.0,
        actions=[
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                parameters=[
                    os.path.join(config_dir, 'nav2_rrt_planning.yaml'),
                    {'use_sim_time': use_sim_time},
                ],
            )
        ],
    )

    controller_server = TimerAction(
        period=12.0,
        actions=[
            Node(
                package='nav2_controller',
                executable='controller_server',
                name='controller_server',
                output='screen',
                parameters=[
                    os.path.join(config_dir, 'nav2_controller.yaml'),
                    {'use_sim_time': use_sim_time},
                ],
                remappings=[('cmd_vel', 'cmd_vel')],
            )
        ],
    )
    recoveries_server = TimerAction(
        period=12.0,
        actions=[
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='recoveries_server',
                output='screen',
                parameters=[
                    {'use_sim_time': use_sim_time},
                    {'local_costmap_topic': 'local_costmap/costmap'},
                    {'local_footprint_topic': 'local_costmap/published_footprint'},
                    {'cycle_frequency': 10.0},
                    {'behavior_plugins': ['spin', 'backup', 'wait']},
                    {'spin': {'plugin': 'nav2_behaviors/Spin'}},
                    {'backup': {'plugin': 'nav2_behaviors/BackUp'}},
                    {'wait': {'plugin': 'nav2_behaviors/Wait'}},
                ],
            )
        ],
    )

    bt_navigator = TimerAction(
        period=12.0,
        actions=[
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                parameters=[
                    os.path.join(config_dir, 'nav2_bt.yaml'),
                    {'use_sim_time': use_sim_time},
                ],
            )
        ],
    )

    lifecycle_manager_navigation = TimerAction(
        period=15.0,
        actions=[
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                parameters=[
                    {'use_sim_time': use_sim_time},
                    {'autostart': True},
                    {'node_names': [
                        'planner_server',
                        'controller_server',
                        'recoveries_server',
                        'bt_navigator',
                    ]},
                    {'bond_timeout': 4.0},
                ],
            )
        ],
    )

    rviz_node = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', os.path.join(rviz_dir, '/home/baby_x/robot_ws/src/path_planning_pkg/rviz/rviz_rrt_viz.rviz')],
                parameters=[{'use_sim_time': use_sim_time}],
            )
        ],
    )


    rrt_planner_node = TimerAction(
        period=18.0,
        actions=[
            Node(
                package='path_planning_pkg',
                executable='rrt_planner',
                name='rrt_planner_node',
                output='screen',
                parameters=[
                    {'use_sim_time': use_sim_time},
                    {'viz_batch_size': 50},
                    {'viz_tick_ms':    25},
                    {'smooth_path':    True},
                    {'smooth_points':  200},
                ],
            )
        ],
    )

    return LaunchDescription([
        declare_world_arg,
        declare_map_arg,
        declare_use_sim_time,
        gzserver,
        gzclient,
        robot_state_publisher,
        spawn_entity,
        map_server,
        amcl,
        lifecycle_manager_localization,
        planner_server,
        controller_server,
        recoveries_server,
        bt_navigator,
        lifecycle_manager_navigation,
        rviz_node,
        rrt_planner_node,
    ])
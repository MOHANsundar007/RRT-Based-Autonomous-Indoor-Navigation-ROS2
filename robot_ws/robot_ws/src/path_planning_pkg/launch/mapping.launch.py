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

    os.makedirs(maps_dir, exist_ok=True)

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_file   = LaunchConfiguration('world_file')

    declare_world_arg = DeclareLaunchArgument(
        'world_file',
        default_value=os.path.join(pkg_dir, 'worlds', 'path_planning.world'),
        description='Full path to the Gazebo world file',
    )
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true',
    )

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
                    '-x', '0.0', '-y', '0.0', '-z', '0.1'],
                output='screen',
            )
        ],
    )

    slam_toolbox = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='slam_toolbox',
                executable='async_slam_toolbox_node',
                name='slam_toolbox',
                output='screen',
                parameters=[
                    os.path.join(config_dir, 'slam_mapping.yaml'),
                    {'use_sim_time': use_sim_time},
                ],
                remappings=[('scan', '/scan')],
            )
        ],
    )

    teleop_keyboard = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop',
        output='screen',
        prefix='xterm -e',
    )

    rviz_node = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', os.path.join(rviz_dir, 'rviz_mapping.rviz')],
                parameters=[{'use_sim_time': use_sim_time}],
            )
        ],
    )

    return LaunchDescription([
        declare_world_arg,
        declare_use_sim_time,
        gzserver,
        gzclient,
        robot_state_publisher,
        spawn_entity,
        slam_toolbox,
        teleop_keyboard,
        rviz_node,
    ])
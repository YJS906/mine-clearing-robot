#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    camera_launch = LaunchConfiguration('camera_launch')
    start_camera = LaunchConfiguration('start_camera')
    start_detector = LaunchConfiguration('start_detector')
    start_grasp = LaunchConfiguration('start_grasp')
    start_moveit = LaunchConfiguration('start_moveit')
    start_hardware = LaunchConfiguration('start_hardware')
    port_name = LaunchConfiguration('port_name')

    camera_x = LaunchConfiguration('camera_x')
    camera_y = LaunchConfiguration('camera_y')
    camera_z = LaunchConfiguration('camera_z')
    camera_roll = LaunchConfiguration('camera_roll')
    camera_pitch = LaunchConfiguration('camera_pitch')
    camera_yaw = LaunchConfiguration('camera_yaw')
    base_frame = LaunchConfiguration('base_frame')
    camera_frame = LaunchConfiguration('camera_frame')
    execute_on_target = LaunchConfiguration('execute_on_target')
    auto_execute_cooldown_sec = LaunchConfiguration('auto_execute_cooldown_sec')

    pkg_share = get_package_share_directory('open_manipulator_vision_grasp')
    config_file = os.path.join(pkg_share, 'config', 'color_grasp.yaml')

    hardware_launch = os.path.join(
        get_package_share_directory('open_manipulator_x_bringup'),
        'launch',
        'hardware.launch.py',
    )
    moveit_launch = os.path.join(
        get_package_share_directory('open_manipulator_x_moveit_config'),
        'launch',
        'moveit_core.launch.py',
    )

    return LaunchDescription([
        DeclareLaunchArgument('camera_launch', default_value='dabai_dc1.launch.xml'),
        DeclareLaunchArgument('start_camera', default_value='true'),
        DeclareLaunchArgument('start_detector', default_value='true'),
        DeclareLaunchArgument('start_grasp', default_value='true'),
        DeclareLaunchArgument('start_moveit', default_value='true'),
        DeclareLaunchArgument('start_hardware', default_value='false'),
        DeclareLaunchArgument('port_name', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('base_frame', default_value='link1'),
        DeclareLaunchArgument('camera_frame', default_value='camera_link'),
        DeclareLaunchArgument('camera_x', default_value='0.0'),
        DeclareLaunchArgument('camera_y', default_value='-0.12'),
        DeclareLaunchArgument('camera_z', default_value='0.0'),
        DeclareLaunchArgument('camera_roll', default_value='0.0'),
        DeclareLaunchArgument('camera_pitch', default_value='0.0'),
        DeclareLaunchArgument('camera_yaw', default_value='-1.57079632679'),
        DeclareLaunchArgument('execute_on_target', default_value='false'),
        DeclareLaunchArgument('auto_execute_cooldown_sec', default_value='10.0'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(hardware_launch),
            launch_arguments={'port_name': port_name}.items(),
            condition=IfCondition(start_hardware),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(moveit_launch),
            condition=IfCondition(start_moveit),
        ),
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('astra_camera'),
                    'launch',
                    camera_launch,
                ])
            ),
            launch_arguments={
                'camera_name': 'camera',
                'enable_color': 'false',
                'enable_ir': 'false',
                'use_uvc_camera': 'false',
            }.items(),
            condition=IfCondition(start_camera),
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_to_arm_tf',
            arguments=[
                '--x', camera_x,
                '--y', camera_y,
                '--z', camera_z,
                '--roll', camera_roll,
                '--pitch', camera_pitch,
                '--yaw', camera_yaw,
                '--frame-id', base_frame,
                '--child-frame-id', camera_frame,
            ],
            condition=IfCondition(start_detector),
        ),
        Node(
            package='open_manipulator_vision_grasp',
            executable='detect_depth_object.py',
            name='detect_depth_object',
            output='screen',
            parameters=[config_file],
            condition=IfCondition(start_detector),
        ),
        Node(
            package='open_manipulator_vision_grasp',
            executable='color_grasp_moveit',
            name='color_grasp_moveit',
            output='screen',
            parameters=[
                config_file,
                {
                    'execute_on_target': execute_on_target,
                    'auto_execute_cooldown_sec': auto_execute_cooldown_sec,
                },
            ],
            condition=IfCondition(start_grasp),
        ),
    ])

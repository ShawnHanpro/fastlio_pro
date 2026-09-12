#!/usr/bin/env python3
"""
点云转 LaserScan 节点启动文件
功能：将 Livox MID360 3D 点云转换为 2D 激光扫描数据，供 Nav2 使用
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

OTHER_CPU_CORE = EnvironmentVariable('ROBOT_OTHER_CPU_CORE', default_value='7')

def affinity_prefix(cpu_core):
    return ['taskset -c ', cpu_core]

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')
    
    pointcloud_to_laserscan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        prefix=affinity_prefix(OTHER_CPU_CORE),
        remappings=[
            ('cloud_in', '/livox/lidar'),
            ('scan', '/scan')
        ],
        parameters=[{
            'use_sim_time': use_sim_time,
            'target_frame': 'base_link',
            'transform_tolerance': 0.2,
            'min_height': 0.15,
            'max_height': 0.35,
            'angle_min': -1.047,
            'angle_max': 2.6,
            'angle_increment': 0.00174533,
            'scan_time': 0.1,
            'range_min': 0.35,
            'range_max': 20.0,
            'use_inf': False,
            'inf_epsilon': 1.0,

            # ==============================
            # 终极修复：强制输入为 RELIABLE
            # 与 Livox 完全一致！
            # ==============================
            'queue_size': 100,
            'max_queue_size': 100,
            "pointcloud_qos_overrides": "RELIABILITY(RELIABLE)",
            "scan_qos_overrides": "RELIABILITY(BEST_EFFORT)"
        }],
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        pointcloud_to_laserscan,
    ])

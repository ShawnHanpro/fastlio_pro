#!/usr/bin/env python3
"""
激光扫描运动补偿节点启动文件
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 参数声明
    scan_topic = LaunchConfiguration('scan_topic')
    odom_topic = LaunchConfiguration('odom_topic')
    output_topic = LaunchConfiguration('output_topic')
    sync_tolerance = LaunchConfiguration('sync_tolerance')
    
    declare_scan_topic = DeclareLaunchArgument(
        'scan_topic',
        default_value='/scan',
        description='输入的激光扫描话题'
    )
    
    declare_odom_topic = DeclareLaunchArgument(
        'odom_topic',
        default_value='/odom',
        description='输入的里程计话题'
    )
    
    declare_output_topic = DeclareLaunchArgument(
        'output_topic',
        default_value='/scan_corrected',
        description='输出的去畸变激光扫描话题'
    )
    
    declare_sync_tolerance = DeclareLaunchArgument(
        'sync_tolerance',
        default_value='0.05',
        description='时间同步容差（秒）。对于10Hz扫描（100ms间隔），建议0.03-0.05秒'
    )
    
    # 运动补偿节点
    motion_compensation_node = Node(
        package='nav2',
        executable='scan_motion_compensation.py',
        name='scan_motion_compensation',
        output='screen',
        parameters=[{
            'scan_topic': scan_topic,
            'odom_topic': odom_topic,
            'output_topic': output_topic,
            'sync_tolerance': sync_tolerance,
            'queue_size': 10,
            'odom_buffer_size': 100,
        }]
    )
    
    return LaunchDescription([
        declare_scan_topic,
        declare_odom_topic,
        declare_output_topic,
        declare_sync_tolerance,
        motion_compensation_node,
    ])

#!/usr/bin/env python3
"""
点云转 LaserScan 节点启动文件
功能：将 Livox MID360 3D 点云转换为 2D 激光扫描数据，供 Nav2 使用

使用方法：
    ros2 launch nav2 pointcloud_to_scan.launch.py

话题订阅：
    - /livox/lidar (sensor_msgs/PointCloud2): 输入的 3D 点云数据

话题发布：
    - /scan (sensor_msgs/LaserScan): 输出的 2D 激光扫描数据（给 Nav2/AMCL 使用）
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
    # ================================
    # Launch 参数声明
    # ================================
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')
    
    # ================================
    # 点云转 LaserScan 节点
    # ================================
    # 将 3D 点云 "压扁" 成 2D 激光数据
    # 原理：提取一定高度范围内（0.05m ~ 0.6m）的障碍物点，投影到水平面
    pointcloud_to_laserscan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        prefix=affinity_prefix(OTHER_CPU_CORE),
        remappings=[
            ('cloud_in', '/livox/lidar'),  # 输入：Livox 雷达发布的 3D 点云
            ('scan', '/scan')              # 输出：2D 激光扫描（Nav2/AMCL 订阅）
        ],
        parameters=[{
            'use_sim_time': use_sim_time,
            
            # TF 配置
            'target_frame': 'base_link',      # 目标坐标系（输出数据的坐标系）
            # 'target_frame': 'livox_frame',      # 目标坐标系（输出数据的坐标系）
            'transform_tolerance': 0.2,       # TF 变换容差（秒）
            
            # 高度范围（相对于 base_link）
            'min_height': 0.15,               # 最小高度 5cm（过滤地面）
            'max_height': 0.2,                # 最大高度 60cm（过滤天花板/上方障碍物）
            
            # 扫描角度范围
            'angle_min': -1.047,            # -180° (全方位扫描)
            'angle_max': 2.6,             # +180° (全方位扫描)
            'angle_increment': 0.00174533,        # 角度分辨率 0.5° (360个点)
            
            # 扫描时间和距离范围
            'scan_time': 0.1,                 # 扫描周期 0.1s (10Hz)
            'range_min': 0.35,                 # 最小测距 0.1m
            'range_max': 20.0,                # 最大测距 20m（与 Livox MID360 量程匹配）
            
            # 无效值处理
            'use_inf': False,                  # 超出范围的点用 inf 表示
            'inf_epsilon': 1.0                # inf 判断阈值
        }],
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        pointcloud_to_laserscan,
    ])

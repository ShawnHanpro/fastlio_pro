#!/bin/bash

echo "正在停止所有 Nav2 导航进程..."

# 停止 Nav2 component container
pkill -9 -f "component_container.*nav2_container"

# 停止 launch 文件
pkill -9 -f "navigation2_gazebo.launch.py"

# 停止其他 Nav2 进程
pkill -9 -f "gazebo_odom_publisher"
pkill -9 -f "wheel_odom_publisher"
pkill -9 -f "pointcloud_to_laserscan"

# 等待进程终止
sleep 2

# 重启 ROS2 daemon 清除节点缓存
echo "清除 ROS2 节点缓存..."
ros2 daemon stop > /dev/null 2>&1
sleep 1
ros2 daemon start > /dev/null 2>&1
sleep 1

echo "✓ Nav2 后台已完全清除"

# 验证
echo "验证剩余节点："
ros2 node list 2>/dev/null | grep -E "nav|amcl|odom|planner|controller|behavior" || echo "  (无)"


#!/usr/bin/env python3
"""
激光扫描运动补偿节点
功能：
1. 接收 /scan 和 /odom 话题
2. 进行时间同步
3. 使用里程计数据去除激光扫描的运动畸变
4. 发布去畸变后的 /scan_corrected 话题

原理：
- 激光扫描是在一段时间内完成的（scan_time）
- 在这段时间内，机器人可能发生移动
- 将每个激光点根据其时间戳和机器人运动轨迹，投影到扫描开始时刻的坐标系
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import message_filters
import numpy as np
import math
from collections import deque
from rclpy.qos import QoSProfile, ReliabilityPolicy


class ScanMotionCompensation(Node):
    def __init__(self):
        super().__init__('scan_motion_compensation')
        
        # 参数声明
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('output_topic', '/scan_corrected')
        self.declare_parameter('queue_size', 10)
        # 时间同步容差：对于10Hz扫描（100ms间隔），建议30-50ms
        # 太小可能无法同步，太大可能同步到相邻消息
        self.declare_parameter('sync_tolerance', 0.05)  # 默认50ms（扫描间隔的50%）
        self.declare_parameter('odom_buffer_size', 100)  # 里程计缓冲区大小
        
        # 获取参数
        scan_topic = self.get_parameter('scan_topic').value
        odom_topic = self.get_parameter('odom_topic').value
        output_topic = self.get_parameter('output_topic').value
        queue_size = self.get_parameter('queue_size').value
        self.sync_tolerance = self.get_parameter('sync_tolerance').value
        odom_buffer_size = self.get_parameter('odom_buffer_size').value
        
        # 里程计数据缓冲区（用于插值）
        self.odom_buffer = deque(maxlen=odom_buffer_size)
        
        # 为 /scan 设置 BEST_EFFORT QoS（匹配发布者的 QoS 设置）
        scan_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=queue_size
        )
        
        # 订阅者
        # message_filters.Subscriber 使用 qos_profile 关键字参数
        self.scan_sub = message_filters.Subscriber(self, LaserScan, scan_topic, qos_profile=scan_qos)
        self.odom_sub = message_filters.Subscriber(self, Odometry, odom_topic)
        
        # 时间同步器
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.scan_sub, self.odom_sub],
            queue_size=queue_size,
            slop=self.sync_tolerance
        )
        self.ts.registerCallback(self.sync_callback)
        
        # 发布者
        self.scan_pub = self.create_publisher(LaserScan, output_topic, 10)
        
        # 单独订阅里程计（用于构建缓冲区）
        self.odom_sub_raw = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            10
        )
        
        self.get_logger().info(f'扫描运动补偿节点已启动')
        self.get_logger().info(f'  输入扫描: {scan_topic}')
        self.get_logger().info(f'  输入里程计: {odom_topic}')
        self.get_logger().info(f'  输出扫描: {output_topic}')
        self.get_logger().info(f'  同步容差: {self.sync_tolerance}秒 ({self.sync_tolerance*1000:.0f}ms)')
        self.get_logger().info(f'  提示: 对于10Hz扫描（100ms间隔），建议同步容差为30-50ms')
    
    def odom_callback(self, msg):
        """存储里程计数据到缓冲区"""
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        # 提取位置和姿态
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        
        # 四元数转欧拉角（只取yaw）
        yaw = self.quaternion_to_yaw(qx, qy, qz, qw)
        
        self.odom_buffer.append({
            'timestamp': timestamp,
            'x': x,
            'y': y,
            'yaw': yaw
        })
    
    def sync_callback(self, scan_msg, odom_msg):
        """时间同步回调函数"""
        try:
            corrected_scan = self.compensate_scan(scan_msg, odom_msg)
            if corrected_scan is not None:
                self.scan_pub.publish(corrected_scan)
        except Exception as e:
            self.get_logger().error(f'运动补偿失败: {str(e)}')
    
    def compensate_scan(self, scan_msg, odom_msg):
        """
        对激光扫描进行运动补偿
        
        参数:
            scan_msg: LaserScan 消息
            odom_msg: Odometry 消息（扫描开始时刻的里程计）
        
        返回:
            去畸变后的 LaserScan 消息
        """
        # 获取扫描开始时刻的时间戳
        scan_start_time = scan_msg.header.stamp.sec + scan_msg.header.stamp.nanosec * 1e-9
        
        # 获取扫描开始时刻的机器人位姿
        x0 = odom_msg.pose.pose.position.x
        y0 = odom_msg.pose.pose.position.y
        qx = odom_msg.pose.pose.orientation.x
        qy = odom_msg.pose.pose.orientation.y
        qz = odom_msg.pose.pose.orientation.z
        qw = odom_msg.pose.pose.orientation.w
        yaw0 = self.quaternion_to_yaw(qx, qy, qz, qw)
        
        # 创建输出消息
        corrected_scan = LaserScan()
        corrected_scan.header = scan_msg.header
        corrected_scan.header.frame_id = scan_msg.header.frame_id
        corrected_scan.angle_min = scan_msg.angle_min
        corrected_scan.angle_max = scan_msg.angle_max
        corrected_scan.angle_increment = scan_msg.angle_increment
        corrected_scan.time_increment = scan_msg.time_increment
        corrected_scan.scan_time = scan_msg.scan_time
        corrected_scan.range_min = scan_msg.range_min
        corrected_scan.range_max = scan_msg.range_max
        
        # 初始化范围数组
        corrected_scan.ranges = [float('inf')] * len(scan_msg.ranges)
        corrected_scan.intensities = list(scan_msg.intensities) if scan_msg.intensities else []
        
        # 处理每个激光点
        num_ranges = len(scan_msg.ranges)
        for i in range(num_ranges):
            range_val = scan_msg.ranges[i]
            
            # 跳过无效值
            if (math.isnan(range_val) or 
                math.isinf(range_val) or 
                range_val < scan_msg.range_min or 
                range_val > scan_msg.range_max):
                corrected_scan.ranges[i] = range_val
                continue
            
            # 计算该激光点的时间戳（相对于扫描开始）
            point_time_offset = i * scan_msg.time_increment
            point_time = scan_start_time + point_time_offset
            
            # 获取该时刻的机器人位姿（通过插值）
            pose = self.get_pose_at_time(point_time)
            if pose is None:
                # 如果无法获取位姿，使用原始值
                corrected_scan.ranges[i] = range_val
                continue
            
            x_t, y_t, yaw_t = pose
            
            # 计算位姿变化（从扫描开始时刻到位姿时刻）
            dx = x_t - x0
            dy = y_t - y0
            dyaw = yaw_t - yaw0
            
            # 计算该激光点的角度
            angle = scan_msg.angle_min + i * scan_msg.angle_increment
            
            # 原始点的坐标（在扫描时刻的坐标系中）
            point_x = range_val * math.cos(angle)
            point_y = range_val * math.sin(angle)
            
            # 将点从扫描时刻的坐标系变换到扫描开始时刻的坐标系
            # 1. 先平移到扫描开始时刻的坐标系原点
            # 2. 再旋转到扫描开始时刻的方向
            
            # 旋转矩阵（从yaw_t转到yaw0）
            cos_dyaw = math.cos(-dyaw)
            sin_dyaw = math.sin(-dyaw)
            
            # 变换后的点坐标（相对于扫描开始时刻的机器人位置）
            corrected_x = (point_x + dx) * cos_dyaw - (point_y + dy) * sin_dyaw
            corrected_y = (point_x + dx) * sin_dyaw + (point_y + dy) * cos_dyaw
            
            # 计算新的距离和角度
            corrected_range = math.sqrt(corrected_x**2 + corrected_y**2)
            corrected_angle = math.atan2(corrected_y, corrected_x)
            
            # 检查范围是否有效
            if (corrected_range >= scan_msg.range_min and 
                corrected_range <= scan_msg.range_max):
                corrected_scan.ranges[i] = corrected_range
            else:
                corrected_scan.ranges[i] = float('inf')
        
        return corrected_scan
    
    def get_pose_at_time(self, timestamp):
        """
        通过插值获取指定时刻的机器人位姿
        
        参数:
            timestamp: 目标时间戳（秒）
        
        返回:
            (x, y, yaw) 或 None
        """
        if len(self.odom_buffer) < 2:
            return None
        
        # 查找时间戳前后的两个里程计数据点
        before = None
        after = None
        
        for odom in self.odom_buffer:
            if odom['timestamp'] <= timestamp:
                before = odom
            elif odom['timestamp'] > timestamp:
                after = odom
                break
        
        # 如果时间戳在缓冲区范围外，返回None
        if before is None or after is None:
            return None
        
        # 线性插值
        t1 = before['timestamp']
        t2 = after['timestamp']
        
        if abs(t2 - t1) < 1e-6:
            # 时间差太小，直接返回before
            return (before['x'], before['y'], before['yaw'])
        
        alpha = (timestamp - t1) / (t2 - t1)
        
        # 角度插值需要考虑周期性
        yaw1 = before['yaw']
        yaw2 = after['yaw']
        
        # 处理角度差（考虑-π到π的周期性）
        dyaw = yaw2 - yaw1
        if dyaw > math.pi:
            dyaw -= 2 * math.pi
        elif dyaw < -math.pi:
            dyaw += 2 * math.pi
        
        yaw = yaw1 + alpha * dyaw
        
        # 归一化角度到[-π, π]
        while yaw > math.pi:
            yaw -= 2 * math.pi
        while yaw < -math.pi:
            yaw += 2 * math.pi
        
        x = before['x'] + alpha * (after['x'] - before['x'])
        y = before['y'] + alpha * (after['y'] - before['y'])
        
        return (x, y, yaw)
    
    def quaternion_to_yaw(self, qx, qy, qz, qw):
        """
        四元数转yaw角
        
        参数:
            qx, qy, qz, qw: 四元数分量
        
        返回:
            yaw角（弧度，范围[-π, π]）
        """
        # 使用标准公式
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw


def main(args=None):
    rclpy.init(args=args)
    node = ScanMotionCompensation()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

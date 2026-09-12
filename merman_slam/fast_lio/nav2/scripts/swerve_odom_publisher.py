#!/usr/bin/env python3
"""
基于真实轮子速度的里程计发布节点 (适用于Swerve舵轮机器人)
配置：四轮独立转向，四轮驱动，支持全向移动
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from control_msgs.msg import DynamicJointState
import tf2_ros
import math
import numpy as np

def quaternion_from_euler(roll, pitch, yaw):
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    return [qx, qy, qz, qw]

class SwerveOdomPublisher(Node):
    def __init__(self):
        super().__init__('swerve_odom_publisher')

        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('publish_rate', 50.0)
        
        # 几何参数
        self.declare_parameter('wheelbase', 0.5)      # 轴距 L
        self.declare_parameter('track_width', 0.4)    # 轮距 W
        self.declare_parameter('wheel_radius', 0.1)   # 轮半径 r

        self.base_frame = self.get_parameter('base_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.wheelbase = self.get_parameter('wheelbase').value
        self.track_width = self.get_parameter('track_width').value
        self.wheel_radius = self.get_parameter('wheel_radius').value

        # Swerve模块位置（相对于底盘中心）
        # REP-103坐标系: x前, y左, z上
        half_L = self.wheelbase / 2.0
        half_W = self.track_width / 2.0
        # 顺序: FL, FR, RL, RR
        self.module_positions = [
            ( half_L,  half_W),  # Front-Left
            ( half_L, -half_W),  # Front-Right
            (-half_L,  half_W),  # Rear-Left
            (-half_L, -half_W),  # Rear-Right
        ]

        # 里程计状态
        self.x, self.y, self.yaw = 0.0, 0.0, 0.0
        self.vx, self.vy, self.vth = 0.0, 0.0, 0.0
        self.last_wheel_time = self.get_clock().now()

        # 关节名称定义
        # 驱动轮: FL, FR, RL, RR
        self.drive_joint_names = [
            'car_wheel_fl_Link2_Joint',
            'car_wheel_fr_Link2_Joint',
            'car_wheel_rl_Link2_Joint',
            'car_wheel_rr_Link2_Joint',
        ]
        # 转向轮: FL, FR, RL, RR（Swerve模式下所有轮子都可转向）
        self.steer_joint_names = [
            'car_wheel_fl_Link1_Joint',
            'car_wheel_fr_Link1_Joint',
            'car_wheel_rl_Link1_Joint',
            'car_wheel_rr_Link1_Joint',
        ]

        # 状态存储
        self.wheel_velocities = [0.0] * 4 # rad/s
        self.steer_angles = [0.0] * 4 # rad
        
        # 速度滤波（减少噪声导致的漂移）
        self.vx_filtered = 0.0
        self.vy_filtered = 0.0
        self.vth_filtered = 0.0
        self.filter_alpha = 0.7  # 低通滤波系数

        # 发布器
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # 订阅
        self.joint_states_sub = self.create_subscription(
            DynamicJointState,
            '/dynamic_joint_states',
            self.joint_states_callback,
            10
        )

        # 定时器
        period = 1.0 / float(self.get_parameter('publish_rate').value)
        self.timer = self.create_timer(period, self._publish_odom)
        
        self.get_logger().info(f"Swerve Wheel Odom Publisher Started (Wheelbase: {self.wheelbase}, Track: {self.track_width})")
        self.get_logger().info("支持全向移动：前进/后退、横移、原地旋转、任意组合")

    def joint_states_callback(self, msg: DynamicJointState):
        current_time = self.get_clock().now()
        
        # 更新驱动轮速度 (Velocity)
        for i, name in enumerate(self.drive_joint_names):
            if name in msg.joint_names:
                idx = msg.joint_names.index(name)
                for k, interface in enumerate(msg.interface_values[idx].interface_names):
                    if 'velocity' in interface.lower():
                        self.wheel_velocities[i] = msg.interface_values[idx].values[k]
                        break

        # 更新转向角 (Position) - Swerve模式下所有4个轮子都可转向
        for i, name in enumerate(self.steer_joint_names):
            if name in msg.joint_names:
                idx = msg.joint_names.index(name)
                for k, interface in enumerate(msg.interface_values[idx].interface_names):
                    if 'position' in interface.lower():
                        self.steer_angles[i] = msg.interface_values[idx].values[k]
                        break
        
        # 计算底盘速度
        self._compute_swerve_kinematics()

        # 积分位置
        dt = (current_time - self.last_wheel_time).nanoseconds * 1e-9
        if 0.0 < dt < 1.0:
            self._integrate_odometry(dt)
        
        self.last_wheel_time = current_time

    def _compute_swerve_kinematics(self):
        """
        Swerve逆运动学计算（从轮子状态推算底盘速度）
        
        使用最小二乘法求解超定方程组：
        对于每个轮子i，有：
        v_ix = vx - wz * l_iy
        v_iy = vy + wz * l_ix
        
        其中：
        - (l_ix, l_iy) 是轮子相对底盘中心的位置
        - v_ix, v_iy 是轮子在机体坐标系下的速度分量
        - vx, vy, wz 是底盘速度（要求解的）
        """
        
        # 从轮子速度和转向角计算每个轮子的速度分量
        wheel_velocities_xy = []
        for i in range(4):
            # 轮子线速度
            v_wheel = self.wheel_velocities[i] * self.wheel_radius
            # 转向角
            theta = self.steer_angles[i]
            # 轮子速度在机体坐标系下的分量
            v_ix = v_wheel * math.cos(theta)
            v_iy = v_wheel * math.sin(theta)
            wheel_velocities_xy.append((v_ix, v_iy))
        
        # 构建方程组 A * [vx, vy, wz]^T = b
        # 对于每个轮子，我们有两个方程：
        # v_ix = vx - wz * l_iy
        # v_iy = vy + wz * l_ix
        
        A = []
        b = []
        
        for i in range(4):
            l_ix, l_iy = self.module_positions[i]
            v_ix, v_iy = wheel_velocities_xy[i]
            
            # 方程1: v_ix = vx - wz * l_iy
            # 重写为: 1*vx + 0*vy + (-l_iy)*wz = v_ix
            A.append([1.0, 0.0, -l_iy])
            b.append(v_ix)
            
            # 方程2: v_iy = vy + wz * l_ix
            # 重写为: 0*vx + 1*vy + l_ix*wz = v_iy
            A.append([0.0, 1.0, l_ix])
            b.append(v_iy)
        
        # 使用最小二乘法求解（8个方程，3个未知数）
        A = np.array(A)
        b = np.array(b)
        
        try:
            # 最小二乘解: x = (A^T * A)^(-1) * A^T * b
            solution = np.linalg.lstsq(A, b, rcond=None)[0]
            vx_raw, vy_raw, wz_raw = solution
            
            # 应用低通滤波，减少噪声
            self.vx_filtered = self.filter_alpha * vx_raw + (1 - self.filter_alpha) * self.vx_filtered
            self.vy_filtered = self.filter_alpha * vy_raw + (1 - self.filter_alpha) * self.vy_filtered
            self.vth_filtered = self.filter_alpha * wz_raw + (1 - self.filter_alpha) * self.vth_filtered
            
            self.vx = self.vx_filtered
            self.vy = self.vy_filtered
            self.vth = self.vth_filtered
            
        except np.linalg.LinAlgError:
            # 如果求解失败，保持上一次的值
            self.get_logger().warn('Swerve运动学求解失败，保持上次速度', throttle_duration_sec=5.0)

    def _integrate_odometry(self, dt):
        """
        里程计积分（支持全向移动）
        """
        # 异常值检测
        if dt > 0.1 or dt < 0:
            return
        
        # 速度合理性检查
        max_reasonable_vx = 2.0   # 最大合理线速度 2 m/s
        max_reasonable_vy = 2.0   # Swerve可以横向移动
        max_reasonable_vth = 2.0  # 最大合理角速度 2 rad/s
        
        vx_safe = max(-max_reasonable_vx, min(max_reasonable_vx, self.vx))
        vy_safe = max(-max_reasonable_vy, min(max_reasonable_vy, self.vy))
        vth_safe = max(-max_reasonable_vth, min(max_reasonable_vth, self.vth))
        
        # 转换到全局坐标系（考虑横向速度vy）
        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)
        
        # 在底盘坐标系下: dx_b = vx * dt, dy_b = vy * dt
        # 旋转到全局:
        # dx_g = dx_b * cos(yaw) - dy_b * sin(yaw)
        # dy_g = dx_b * sin(yaw) + dy_b * cos(yaw)
        
        dx_body = vx_safe * dt
        dy_body = vy_safe * dt
        
        delta_x = dx_body * cos_yaw - dy_body * sin_yaw
        delta_y = dx_body * sin_yaw + dy_body * cos_yaw
        delta_th = vth_safe * dt
        
        # 位移合理性检查
        max_delta = 0.5  # 单步最大位移 0.5m
        delta_dist = math.sqrt(delta_x**2 + delta_y**2)
        
        if delta_dist > max_delta:
            scale = max_delta / delta_dist
            delta_x *= scale
            delta_y *= scale
            self.get_logger().warn(
                f'里程计异常位移检测: {delta_dist:.3f}m，已限制',
                throttle_duration_sec=5.0
            )

        self.x += delta_x
        self.y += delta_y
        self.yaw += delta_th
        
        # 归一化
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

    def _publish_odom(self):
        """
        发布里程计消息和 TF 变换
        
        TF 结构说明：
        - odom → base_link：由本节点（基于轮子速度的里程计）连续发布
        - 作用：提供连续、平滑的局部位姿估计
        - 特点：支持全向移动，可以横向移动和原地旋转
        
        - map → odom：由 AMCL/SLAM 节点发布
        - 作用：全局定位修正，弥补里程计漂移
        """
        current_time = self.get_clock().now()
        
        # ========== 发布里程计消息 ==========
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        q = quaternion_from_euler(0, 0, self.yaw)
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        
        # 里程计协方差（Swerve底盘全向移动能力强，但也会有累积误差）
        pose_cov = [0.0] * 36
        pose_cov[0] = 0.05   # x 位置不确定性（比Ackermann更准确）
        pose_cov[7] = 0.05   # y 位置不确定性
        pose_cov[35] = 0.03  # yaw 朝向不确定性
        odom.pose.covariance = pose_cov
        
        # 速度协方差（包含横向速度）
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.angular.z = self.vth
        twist_cov = [0.0] * 36
        twist_cov[0] = 0.03  # vx 速度不确定性
        twist_cov[7] = 0.03  # vy 速度不确定性（Swerve支持横向移动）
        twist_cov[35] = 0.03 # 角速度不确定性
        odom.twist.covariance = twist_cov

        self.odom_pub.publish(odom)
        
        # ========== 发布 TF 变换：odom → base_link ==========
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation = odom.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = SwerveOdomPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()








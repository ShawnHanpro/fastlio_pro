#!/usr/bin/env python3
"""
Gazebo Ground Truth Odometry Publisher
从 Gazebo 获取真实位姿，彻底消除积分漂移

这是解决里程计漂移问题的根本方案：
- 直接从 Gazebo 获取机器人真实位姿
- 零积分误差，零漂移
- 非常适合仿真环境
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import GetModelState
from tf2_ros import TransformBroadcaster
import math

class GazeboGTOdomPublisher(Node):
    def __init__(self):
        super().__init__('gazebo_gt_odom_publisher')
        
        # 参数声明
        self.declare_parameter('model_name', 'evbhuman_robot')  # Gazebo 中的模型名称
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('publish_rate', 50.0)
        
        self.model_name = self.get_parameter('model_name').value
        self.base_frame = self.get_parameter('base_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        
        # 发布器
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # ✅ 优先使用 Gazebo 服务（更可靠，总是可用）
        self.model_state_client = self.create_client(
            GetModelState,
            '/gazebo/get_model_state'
        )
        
        # 备用：订阅话题（如果可用）
        self.model_states_sub = self.create_subscription(
            ModelStates,
            '/gazebo/model_states',
            self.model_states_callback,
            10
        )
        
        # 统计信息
        self.model_states_received = False
        self.model_states_count = 0
        self.use_service = True  # ✅ 默认使用服务（更可靠）
        
        # 状态存储
        self.last_pose = None
        self.last_twist = None
        self.last_time = None
        self.vx = 0.0
        self.vy = 0.0
        self.vth = 0.0
        
        # 服务调用状态
        self.pending_service_call = None
        self.service_call_count = 0
        
        # 初始位姿（用于计算相对位移）
        self.initial_x = None
        self.initial_y = None
        self.initial_yaw = None
        
        # 定时发布（同时检查服务响应）
        period = 1.0 / self.get_parameter('publish_rate').value
        self.timer = self.create_timer(period, self.publish_odom)
        
        # ✅ 定时通过服务获取模型状态（主要方式）
        # 使用更高的频率来确保及时获取数据
        service_period = 0.1  # 每 100ms 调用一次服务
        self.service_timer = self.create_timer(service_period, self.get_model_state_via_service)
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('🚀 Gazebo Ground Truth Odometry Publisher')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'   模型名称: {self.model_name}')
        self.get_logger().info(f'   base_frame: {self.base_frame}')
        self.get_logger().info(f'   odom_frame: {self.odom_frame}')
        self.get_logger().info(f'   ✅ 使用 Gazebo 真实位姿，零漂移！')
        self.get_logger().info('=' * 60)
    
    def model_states_callback(self, msg):
        """从 Gazebo 获取模型真实位姿和速度"""
        self.model_states_received = True
        self.model_states_count += 1
        
        try:
            # 查找模型索引
            if self.model_name not in msg.name:
                # 只在第一次警告，避免刷屏
                if not hasattr(self, '_model_not_found_warned'):
                    self.get_logger().error(
                        f'❌ 模型 "{self.model_name}" 未找到！'
                    )
                    self.get_logger().error(
                        f'   可用模型: {msg.name}'
                    )
                    self.get_logger().error(
                        f'   请检查 launch 文件中的 model_name 参数是否正确'
                    )
                    self._model_not_found_warned = True
                return
            
            # 清除警告标志（如果找到了模型）
            if hasattr(self, '_model_not_found_warned'):
                self.get_logger().info(f'✅ 已找到模型 "{self.model_name}"')
                delattr(self, '_model_not_found_warned')
            
            idx = msg.name.index(self.model_name)
            pose = msg.pose[idx]
            twist = msg.twist[idx]
            
            # 保存初始位姿（第一次）
            if self.initial_x is None:
                self.initial_x = pose.position.x
                self.initial_y = pose.position.y
                # 计算初始 yaw
                qx = pose.orientation.x
                qy = pose.orientation.y
                qz = pose.orientation.z
                qw = pose.orientation.w
                self.initial_yaw = math.atan2(
                    2.0 * (qw * qz + qx * qy),
                    1.0 - 2.0 * (qy * qy + qz * qz)
                )
                self.get_logger().info(
                    f'📍 初始位姿已设置: x={self.initial_x:.3f}, y={self.initial_y:.3f}, '
                    f'yaw={math.degrees(self.initial_yaw):.1f}°'
                )
                self.get_logger().info(
                    f'✅ 开始从 Gazebo 获取真实位姿，零漂移模式已激活！'
                )
            
            self.last_pose = pose
            self.last_twist = twist
            self.last_time = self.get_clock().now()
            
            # 计算速度（在 base_link 坐标系下）
            # 注意：Gazebo 返回的速度是在 world 坐标系下的，需要转换
            # 简化处理：直接使用线性速度的 x 分量作为前进速度
            self.vx = twist.linear.x
            self.vy = twist.linear.y
            self.vth = twist.angular.z
            
        except Exception as e:
            self.get_logger().error(f'获取模型状态失败: {e}')
    
    def publish_odom(self):
        """发布里程计和 TF"""
        # 检查是否有未完成的服务调用，处理它
        if self.pending_service_call is not None and self.pending_service_call.done():
            try:
                response = self.pending_service_call.result()
                self._handle_service_response(response)
            except Exception as e:
                pass
            finally:
                self.pending_service_call = None
        
        # ✅ 如果还没收到模型状态，发布一个默认的 odom frame（原点）
        # 这样可以避免 Nav2 因为缺少 odom frame 而报错
        if self.last_pose is None or self.last_time is None:
            # 发布一个默认的 odom -> base_link（原点，无旋转）
            self._publish_default_odom()
            return
        
        if self.initial_x is None:
            # 如果没有初始位姿，使用当前位置作为初始位姿
            self.initial_x = self.last_pose.position.x
            self.initial_y = self.last_pose.position.y
            qx = self.last_pose.orientation.x
            qy = self.last_pose.orientation.y
            qz = self.last_pose.orientation.z
            qw = self.last_pose.orientation.w
            self.initial_yaw = math.atan2(
                2.0 * (qw * qz + qx * qy),
                1.0 - 2.0 * (qy * qy + qz * qz)
            )
            self.get_logger().info(
                f'📍 初始位姿已设置: x={self.initial_x:.3f}, y={self.initial_y:.3f}, '
                f'yaw={math.degrees(self.initial_yaw):.1f}°'
            )
        
        current_time = self.get_clock().now()
        
        # 提取位置和姿态
        pose = self.last_pose
        x_world = pose.position.x
        y_world = pose.position.y
        z_world = pose.position.z
        qx = pose.orientation.x
        qy = pose.orientation.y
        qz = pose.orientation.z
        qw = pose.orientation.w
        
        # 计算相对于初始位置的位移（odom frame 的起点）
        x_odom = x_world - self.initial_x
        y_odom = y_world - self.initial_y
        
        # 计算 yaw 角（相对于初始朝向）
        yaw_world = math.atan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy * qy + qz * qz)
        )
        yaw_odom = yaw_world - self.initial_yaw
        
        # 归一化角度到 [-π, π]
        while yaw_odom > math.pi:
            yaw_odom -= 2.0 * math.pi
        while yaw_odom < -math.pi:
            yaw_odom += 2.0 * math.pi
        
        # 转换为四元数
        qz_odom = math.sin(yaw_odom * 0.5)
        qw_odom = math.cos(yaw_odom * 0.5)
        
        # ========================================
        # 发布 TF: odom -> base_link
        # ========================================
        tf = TransformStamped()
        tf.header.stamp = current_time.to_msg()
        tf.header.frame_id = self.odom_frame
        tf.child_frame_id = self.base_frame
        tf.transform.translation.x = x_odom
        tf.transform.translation.y = y_odom
        tf.transform.translation.z = 0.0  # 2D 导航，z=0
        tf.transform.rotation.x = 0.0
        tf.transform.rotation.y = 0.0
        tf.transform.rotation.z = qz_odom
        tf.transform.rotation.w = qw_odom
        self.tf_broadcaster.sendTransform(tf)
        
        # ========================================
        # 发布 Odometry 消息
        # ========================================
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        
        # 位置（在 odom frame 下）
        odom.pose.pose.position.x = x_odom
        odom.pose.pose.position.y = y_odom
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = qz_odom
        odom.pose.pose.orientation.w = qw_odom
        
        # ========================================
        # ✅ 协方差：Ground Truth 非常准确
        # ========================================
        # 对角协方差矩阵，索引：
        # [0] x,  [7] y,  [14] z,  [21] roll,  [28] pitch,  [35] yaw
        pose_cov = [0.0] * 36
        pose_cov[0] = 0.001   # x: 非常小（Ground Truth 准确）
        pose_cov[7] = 0.001   # y: 非常小
        pose_cov[35] = 0.001  # yaw: 非常小
        odom.pose.covariance = pose_cov
        
        # 速度（在 base_link 坐标系下）
        # 将 world 坐标系下的速度转换到 base_link
        # 简化：假设速度已经是在 base_link 下（对于 2D 导航通常是合理的）
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.angular.z = self.vth
        
        # 速度协方差
        twist_cov = [0.0] * 36
        twist_cov[0] = 0.01   # vx
        twist_cov[7] = 0.01   # vy
        twist_cov[35] = 0.01  # vth
        odom.twist.covariance = twist_cov
        
        self.odom_pub.publish(odom)
    
    def _publish_default_odom(self):
        """发布默认的 odom frame（当还没有收到模型状态时）"""
        # 检查是否收到了任何模型状态消息
        if not self.model_states_received:
            # 只在第一次警告，避免刷屏
            if not hasattr(self, '_default_odom_warned'):
                self.get_logger().warn(
                    '⚠️  尚未收到 Gazebo 模型状态消息'
                )
                self.get_logger().warn(
                    '   请检查：'
                )
                self.get_logger().warn(
                    '   1. Gazebo 是否已启动'
                )
                self.get_logger().warn(
                    '   2. 模型是否已加载到 Gazebo'
                )
                self.get_logger().warn(
                    '   3. /gazebo/model_states 话题是否在发布'
                )
                self._default_odom_warned = True
        else:
            # 收到了消息但模型名称不匹配
            if not hasattr(self, '_model_mismatch_warned'):
                self.get_logger().warn(
                    f'⚠️  已收到 {self.model_states_count} 条模型状态消息，但未找到模型 "{self.model_name}"'
                )
                self.get_logger().warn(
                    '   请检查模型名称是否匹配'
                )
                self._model_mismatch_warned = True
        
        current_time = self.get_clock().now()
        
        # 发布 TF: odom -> base_link（原点）
        tf = TransformStamped()
        tf.header.stamp = current_time.to_msg()
        tf.header.frame_id = self.odom_frame
        tf.child_frame_id = self.base_frame
        tf.transform.translation.x = 0.0
        tf.transform.translation.y = 0.0
        tf.transform.translation.z = 0.0
        tf.transform.rotation.x = 0.0
        tf.transform.rotation.y = 0.0
        tf.transform.rotation.z = 0.0
        tf.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(tf)
        
        # 发布默认 Odometry 消息
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        
        odom.pose.pose.position.x = 0.0
        odom.pose.pose.position.y = 0.0
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.w = 1.0
        
        # 设置较大的协方差，表示不确定性
        pose_cov = [0.0] * 36
        pose_cov[0] = 1.0
        pose_cov[7] = 1.0
        pose_cov[35] = 0.5
        odom.pose.covariance = pose_cov
        
        odom.twist.twist.linear.x = 0.0
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = 0.0
        
        twist_cov = [0.0] * 36
        odom.twist.covariance = twist_cov
        
        self.odom_pub.publish(odom)
    
    def get_model_state_via_service(self):
        """通过服务获取模型状态（主要方式）"""
        # 如果有未完成的调用，检查是否完成
        if self.pending_service_call is not None:
            if self.pending_service_call.done():
                try:
                    response = self.pending_service_call.result()
                    self._handle_service_response(response)
                except Exception as e:
                    if not hasattr(self, '_service_result_error_warned'):
                        self.get_logger().warn(f'服务响应异常: {e}')
                        self._service_result_error_warned = True
                finally:
                    self.pending_service_call = None
            else:
                # 还在等待响应，下次再检查
                return
        
        # 等待服务可用
        if not self.model_state_client.wait_for_service(timeout_sec=0.01):
            if not hasattr(self, '_service_unavailable_warned'):
                self.get_logger().warn(
                    '⚠️  /gazebo/get_model_state 服务不可用，等待 Gazebo 启动...'
                )
                self._service_unavailable_warned = True
            return
        
        # 清除警告标志
        if hasattr(self, '_service_unavailable_warned'):
            delattr(self, '_service_unavailable_warned')
            self.get_logger().info('✅ Gazebo 服务已可用')
        
        try:
            request = GetModelState.Request()
            request.model_name = self.model_name
            request.relative_entity_name = ''  # world frame
            
            # 发起异步服务调用
            self.pending_service_call = self.model_state_client.call_async(request)
            self.service_call_count += 1
            
        except Exception as e:
            if not hasattr(self, '_service_error_warned'):
                self.get_logger().warn(f'发起服务调用失败: {e}')
                self._service_error_warned = True
    
    def _handle_service_response(self, response):
        """处理服务响应"""
        if response and response.success:
            # 更新状态
            self.last_pose = response.pose
            self.last_twist = response.twist
            self.last_time = self.get_clock().now()
            
            # 计算速度
            self.vx = response.twist.linear.x
            self.vy = response.twist.linear.y
            self.vth = response.twist.angular.z
            
            # ✅ 调试日志：每次更新都记录（限制频率）
            if not hasattr(self, '_last_log_time'):
                self._last_log_time = 0
            
            current_time_ns = self.get_clock().now().nanoseconds
            if current_time_ns - self._last_log_time > 5e9:  # 每5秒记录一次
                self.get_logger().debug(
                    f'📍 位姿更新: x={response.pose.position.x:.3f}, y={response.pose.position.y:.3f}, '
                    f'vx={self.vx:.3f}, ω={self.vth:.3f}'
                )
                self._last_log_time = current_time_ns
                
                # 设置初始位姿（第一次）
                if self.initial_x is None:
                    self.initial_x = response.pose.position.x
                    self.initial_y = response.pose.position.y
                    qx = response.pose.orientation.x
                    qy = response.pose.orientation.y
                    qz = response.pose.orientation.z
                    qw = response.pose.orientation.w
                    self.initial_yaw = math.atan2(
                        2.0 * (qw * qz + qx * qy),
                        1.0 - 2.0 * (qy * qy + qz * qz)
                    )
            # 更新状态
            self.last_pose = response.pose
            self.last_twist = response.twist
            self.last_time = self.get_clock().now()
            
            # 计算速度
            self.vx = response.twist.linear.x
            self.vy = response.twist.linear.y
            self.vth = response.twist.angular.z
            
            # 设置初始位姿（第一次）
            if self.initial_x is None:
                self.initial_x = response.pose.position.x
                self.initial_y = response.pose.position.y
                qx = response.pose.orientation.x
                qy = response.pose.orientation.y
                qz = response.pose.orientation.z
                qw = response.pose.orientation.w
                self.initial_yaw = math.atan2(
                    2.0 * (qw * qz + qx * qy),
                    1.0 - 2.0 * (qy * qy + qz * qz)
                )
                if not hasattr(self, '_service_success_logged'):
                    self.get_logger().info(
                        f'✅ 通过服务获取模型状态成功！'
                    )
                    self.get_logger().info(
                        f'📍 初始位姿: x={self.initial_x:.3f}, y={self.initial_y:.3f}, '
                        f'yaw={math.degrees(self.initial_yaw):.1f}°'
                    )
                    self._service_success_logged = True
                self.model_states_received = True
                
                # 清除之前的错误警告
                if hasattr(self, '_service_failed_warned'):
                    delattr(self, '_service_failed_warned')
        else:
            # 服务返回失败
            if not hasattr(self, '_service_failed_warned'):
                self.get_logger().warn(
                    f'⚠️  服务调用失败: 模型 "{self.model_name}" 未找到'
                )
                self._service_failed_warned = True
    
    def _check_and_switch_to_service(self):
        """检查话题是否可用，如果不可用则切换到服务"""
        if self.model_states_received:
            # 话题可用，取消定时器
            if hasattr(self, '_check_timer'):
                self._check_timer.cancel()
            return
        
        # 检查话题是否有发布者
        try:
            topic_info = self.get_topic_names_and_types()
            topic_names = [name for name, _ in topic_info]
            
            if '/gazebo/model_states' not in topic_names:
                # 话题不存在，切换到服务
                self.use_service = True
                self.get_logger().warn(
                    '⚠️  /gazebo/model_states 话题不存在，切换到服务模式'
                )
            else:
                # 检查是否有发布者（通过尝试获取信息）
                from rclpy.topic import get_topic_names_and_types
                try:
                    publishers = self.count_publishers('/gazebo/model_states')
                    if publishers == 0:
                        self.use_service = True
                        self.get_logger().warn(
                            '⚠️  /gazebo/model_states 话题无发布者，切换到服务模式'
                        )
                except:
                    pass
        except:
            pass
        
        # 取消检查定时器
        if hasattr(self, '_check_timer'):
            self._check_timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = GazeboGTOdomPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


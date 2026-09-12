#!/usr/bin/env python3
"""
键盘控制节点
使用键盘控制四轮四转小车在Gazebo中运动
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, UInt8
import sys
import select
import termios
import tty
import time

# 键盘映射说明
KEY_BINDINGS = """
---------------------------
键盘控制四轮四转小车
---------------------------
移动控制:
   w/W : 前进
   s/S : 后退
   a/A : 原地左转 (仅Swerve模式)
   d/D : 原地右转 (仅Swerve模式)
   q/Q : 前进左转
   e/E : 前进右转
   z/Z : 平行向左移动 (仅Swerve模式)
   c/C : 平行向右移动 (仅Swerve模式)
   
   x/X : 停止
   空格: 紧急停止

速度调节:
   i/I : 增加线速度 10%
   k/K : 减小线速度 10%
   j/J : 增加角速度 10%
   l/L : 减小角速度 10%

   u/U : 同时增加线速度和角速度 10%
   m/M : 同时减小线速度和角速度 10%

其他:
   v/V : 使能底盘 (/chassis/enable -> true)
   b/B : 失能底盘 (/chassis/enable -> false)
   n/N : 清除错误 (/evb_chassis/clear_all_faults -> 0)
   h/H : 显示帮助信息
   Ctrl+C : 退出程序

---------------------------
注意: 标记为"仅Swerve模式"的功能在Ackermann模式下无效
---------------------------
当前速度设置:
  线速度: {:.2f} m/s
  角速度: {:.2f} rad/s
---------------------------
"""


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')

        self.declare_parameter('cmd_vel_topic', '/cmd_vel_keyboard')
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        
        # 发布器
        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.enable_pub = self.create_publisher(Bool, '/chassis/enable', 10)
        self.clear_faults_pub = self.create_publisher(UInt8, '/evb_chassis/clear_all_faults', 10)
        
        # 速度参数
        self.declare_parameter('linear_speed', 0.5)
        self.declare_parameter('angular_speed', 0.5)
        self.declare_parameter('speed_increment', 0.1)
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('command_hold_timeout', 0.2)
        
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.speed_increment = self.get_parameter('speed_increment').value
        self.publish_rate = max(1.0, self.get_parameter('publish_rate').value)
        self.command_hold_timeout = max(0.02, self.get_parameter('command_hold_timeout').value)
        
        # 当前速度值
        self.current_linear = 0.0
        self.current_angular = 0.0
        self.current_linear_y = 0.0
        self.active_command = (0.0, 0.0, 0.0)
        self.last_motion_key_time = 0.0
        self.last_publish_time = 0.0
        
        # 保存终端设置
        self.settings = None
        if sys.platform != 'win32':
            self.settings = termios.tcgetattr(sys.stdin)
        
        self.get_logger().info('键盘控制节点已启动')
        self.print_help()
    
    def get_key(self):
        """获取键盘按键"""
        if sys.platform == 'win32':
            # Windows系统
            import msvcrt
            if msvcrt.kbhit():
                return msvcrt.getch().decode('utf-8')
            return None
        else:
            # Linux/Mac系统
            ready, _, _ = select.select([sys.stdin], [], [], 0)
            if ready:
                return sys.stdin.read(1)
            return None
    
    def print_help(self):
        """打印帮助信息"""
        print(KEY_BINDINGS.format(self.linear_speed, self.angular_speed))
    
    def update_speed_display(self):
        """更新速度显示"""
        print(f"\r当前速度 -> 线速度: {self.linear_speed:.2f} m/s  |  角速度: {self.angular_speed:.2f} rad/s  ", end='')
        sys.stdout.flush()
    
    def publish_twist(self, linear_x, angular_z, linear_y=0.0):
        """发布Twist消息"""
        msg = Twist()
        msg.linear.x = linear_x
        msg.linear.y = linear_y
        msg.angular.z = angular_z
        self.cmd_vel_pub.publish(msg)
        self.current_linear = linear_x
        self.current_angular = angular_z
        self.current_linear_y = linear_y

    def set_motion_command(self, linear_x, angular_z, linear_y=0.0):
        """设置当前运动命令；按键保持期间会按固定频率重复发布。"""
        self.active_command = (linear_x, angular_z, linear_y)
        self.last_motion_key_time = time.monotonic()
        self.publish_twist(linear_x, angular_z, linear_y)
        self.last_publish_time = self.last_motion_key_time

    def stop_motion(self):
        """停止并清除当前运动命令。"""
        self.active_command = (0.0, 0.0, 0.0)
        self.last_motion_key_time = 0.0
        if self.current_linear != 0.0 or self.current_angular != 0.0 or self.current_linear_y != 0.0:
            self.publish_twist(0.0, 0.0, 0.0)

    def refresh_motion_command(self):
        """无新按键时维持短时间命令，超时后自动停车。"""
        now = time.monotonic()
        linear_x, angular_z, linear_y = self.active_command
        has_motion = linear_x != 0.0 or angular_z != 0.0 or linear_y != 0.0

        if not has_motion:
            return

        if now - self.last_motion_key_time > self.command_hold_timeout:
            self.stop_motion()
            return

        publish_interval = 1.0 / self.publish_rate
        if now - self.last_publish_time >= publish_interval:
            self.publish_twist(linear_x, angular_z, linear_y)
            self.last_publish_time = now
    
    def run(self):
        """主循环"""
        try:
            if sys.platform != 'win32':
                tty.setraw(sys.stdin.fileno())

            while rclpy.ok():
                key = self.get_key()
                
                if key is None:
                    # 没有新按键时短暂保持命令，超时自动停车，避免按键重复事件造成脉冲运动。
                    self.refresh_motion_command()
                    
                    # 加一个短暂的休眠，防止CPU占用过高，并允许ROS处理其他事件
                    # 0.02秒 = 50Hz，这个频率足够平滑
                    try:
                        rclpy.spin_once(self, timeout_sec=0.02)
                    except rclpy.exceptions.ROSInterruptException:
                        break # 节点关闭时退出
                    continue # 继续等待下一个按键或下一次循环
                
                # 运动控制
                if key.lower() == 'w':
                    # 前进
                    self.set_motion_command(self.linear_speed, 0.0)
                    print("\r前进                                                      ")
                
                elif key.lower() == 's':
                    # 后退
                    self.set_motion_command(-self.linear_speed, 0.0)
                    print("\r后退                                                      ")
                
                elif key.lower() == 'a':
                    # 原地左转
                    self.set_motion_command(0.0, self.angular_speed)
                    print("\r原地左转                                                  ")
                
                elif key.lower() == 'd':
                    # 原地右转
                    self.set_motion_command(0.0, -self.angular_speed)
                    print("\r原地右转                                                  ")
                
                elif key.lower() == 'q':
                    # 前进左转
                    self.set_motion_command(self.linear_speed, self.angular_speed)
                    print("\r前进左转                                                  ")
                
                elif key.lower() == 'e':
                    # 前进右转
                    self.set_motion_command(self.linear_speed, -self.angular_speed)
                    print("\r前进右转                                                  ")
                
                elif key.lower() == 'z':
                    # 平行向左移动
                    self.set_motion_command(0.0, 0.0, self.linear_speed)
                    print("\r平行向左移动                                              ")
                
                elif key.lower() == 'c':
                    # 平行向右移动
                    self.set_motion_command(0.0, 0.0, -self.linear_speed)
                    print("\r平行向右移动                                              ")
                
                elif key.lower() == 'x' or key == ' ':
                    # 停止
                    self.stop_motion()
                    print("\r停止                                                      ")
                
                # 速度调节
                elif key.lower() == 'i':
                    # 增加线速度
                    self.linear_speed += self.speed_increment
                    self.update_speed_display()
                    self.get_logger().info(f'线速度增加到: {self.linear_speed:.2f} m/s')
                
                elif key.lower() == 'k':
                    # 减小线速度
                    self.linear_speed = max(0.0, self.linear_speed - self.speed_increment)
                    self.update_speed_display()
                    self.get_logger().info(f'线速度减小到: {self.linear_speed:.2f} m/s')
                
                elif key.lower() == 'j':
                    # 增加角速度
                    self.angular_speed += self.speed_increment
                    self.update_speed_display()
                    self.get_logger().info(f'角速度增加到: {self.angular_speed:.2f} rad/s')
                
                elif key.lower() == 'l':
                    # 减小角速度
                    self.angular_speed = max(0.0, self.angular_speed - self.speed_increment)
                    self.update_speed_display()
                    self.get_logger().info(f'角速度减小到: {self.angular_speed:.2f} rad/s')
                
                elif key.lower() == 'u':
                    # 同时增加线速度和角速度
                    self.linear_speed += self.speed_increment
                    self.angular_speed += self.speed_increment
                    self.update_speed_display()
                    self.get_logger().info(f'速度增加到: 线速度={self.linear_speed:.2f} m/s, 角速度={self.angular_speed:.2f} rad/s')
                
                elif key.lower() == 'm':
                    # 同时减小线速度和角速度
                    self.linear_speed = max(0.0, self.linear_speed - self.speed_increment)
                    self.angular_speed = max(0.0, self.angular_speed - self.speed_increment)
                    self.update_speed_display()
                    self.get_logger().info(f'速度减小到: 线速度={self.linear_speed:.2f} m/s, 角速度={self.angular_speed:.2f} rad/s')
                
                # 帮助
                elif key.lower() == 'h':
                    self.print_help()

                # 使能底盘
                elif key.lower() == 'v':
                    msg = Bool()
                    msg.data = True
                    self.enable_pub.publish(msg)
                    self.get_logger().info('发送: 使能底盘 (True)')
                    print("\r使能底盘                                                      ")

                # 失能底盘
                elif key.lower() == 'b':
                    msg = Bool()
                    msg.data = False
                    self.enable_pub.publish(msg)
                    self.get_logger().info('发送: 失能底盘 (False)')
                    print("\r失能底盘                                                      ")

                # 清除错误
                elif key.lower() == 'n':
                    msg = UInt8()
                    msg.data = 0
                    self.clear_faults_pub.publish(msg)
                    self.get_logger().info('发送: 清除错误 (0)')
                    print("\r清除错误                                                      ")
                
                # 退出
                elif key == '\x03':  # Ctrl+C
                    break
        
        except Exception as e:
            self.get_logger().error(f'错误: {e}')
        
        finally:
            # 停止小车
            self.stop_motion()
            
            # 恢复终端设置
            if self.settings is not None:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()
    
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

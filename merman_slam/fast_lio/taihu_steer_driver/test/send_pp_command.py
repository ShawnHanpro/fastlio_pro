#!/usr/bin/env python3
"""示例：发送 taihu_steer_driver PP模式位置命令"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger

class PPCommandPublisher(Node):
    def __init__(self):
        super().__init__('pp_command_publisher')
        # 与当前驱动接口保持一致
        self.pos_pub = self.create_publisher(Float64MultiArray, '/steer/position_cmd', 10)
        self.enable_client = self.create_client(Trigger, '/steer/enable')

        self.position_seq = [
            [0.0, 0.0, 0.0, 0.0],
            [1.5, 1.5, 1.5, 1.5],
            [-1.5, -1.5, -1.5, -1.5],
            [0.0, 0.0, -0.0, -0.0],
        ]
        self.seq_idx = 0
        self.timer = self.create_timer(2.0, self.timer_callback)

        self.get_logger().info('PP Command Publisher started: topic=/steer/position_cmd, period=2.0s')
        
    def send_position(self, q1, q2, q3, q4):
        """发送4轴PP目标位置（单位：弧度）"""
        msg = Float64MultiArray()
        msg.data = [float(q1), float(q2), float(q3), float(q4)]
        self.pos_pub.publish(msg)
        self.get_logger().info(f'Sent position: {msg.data}')

    def timer_callback(self):
        cmd = self.position_seq[self.seq_idx]
        self.send_position(*cmd)
        self.seq_idx = (self.seq_idx + 1) % len(self.position_seq)

    def request_enable(self):
        """调用 /steer/enable 服务使能驱动"""
        if not self.enable_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Service /steer/enable not available')
            return False

        req = Trigger.Request()
        future = self.enable_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if not future.done() or future.result() is None:
            self.get_logger().error('Enable service call timed out or failed')
            return False

        resp = future.result()
        self.get_logger().info(f'Enable response: success={resp.success}, message={resp.message}')
        return resp.success

def main(args=None):
    rclpy.init(args=args)
    node = PPCommandPublisher()

    try:
        node.request_enable()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt, exiting...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

#!/usr/bin/env python3
import os
import signal
import sys
from pathlib import Path

import rclpy
from rclpy.node import Node




try:
    from .command import RobotCommandStateMachine
    from .event.audio_done import register_audio_done_event
    from .event.station_arrival import register_station_arrival_event
    from .event.robot_status import register_robot_status_event
    from .mqtt.command_client import MQTTCommandClient, ROS_DOMAIN_ID, ROS_DOMAIN_ID_CONFIGURED, resolve_send_topic
except ImportError:
    from command import RobotCommandStateMachine
    from event.audio_done import register_audio_done_event
    from event.station_arrival import register_station_arrival_event
    from event.robot_status import register_robot_status_event
    from mqtt.command_client import MQTTCommandClient, ROS_DOMAIN_ID, ROS_DOMAIN_ID_CONFIGURED, resolve_send_topic

class RobotBridge(Node):
    def __init__(self):
        super().__init__("robot_bridge_node")
        self.get_logger().info("======启动桥接服务======")

        self.command_machine = RobotCommandStateMachine(self)
        self.mqtt_client = MQTTCommandClient(self.get_logger(), self.dispatch_action)
        self.mqtt_client.connect()
        self.event_subscriptions = [
            register_audio_done_event(self),
            register_station_arrival_event(
                self,
                self.mqtt_client.publish_send,
                resolve_send_topic,
            ),
            register_robot_status_event(
                self,
                self.mqtt_client.publish_send,
                resolve_send_topic,
            ),
        ]

    def dispatch_action(self, action, params):
        return self.command_machine.execute(action, params)

    def destroy_node(self):
        self.mqtt_client.close()
        super().destroy_node()


def parse_domain_id(value):
    try:
        domain_id = int(str(value).strip())
    except (TypeError, ValueError):
        return None
    return domain_id if domain_id >= 0 else None


def detect_local_ros_domain_id():
    if ROS_DOMAIN_ID_CONFIGURED:
        return ROS_DOMAIN_ID, f"config({ROS_DOMAIN_ID})"

    domain_id = parse_domain_id(os.getenv("ROS_DOMAIN_ID"))
    if domain_id is not None:
        return domain_id, "ROS_DOMAIN_ID"

    proc_dir = Path("/proc")
    if not proc_dir.exists():
        return ROS_DOMAIN_ID, f"config({ROS_DOMAIN_ID})"

    # 独立 Python 进程优先从本机已有 ROS 进程中继承 domain。
    for cmdline_path in proc_dir.glob("[0-9]*/cmdline"):
        try:
            cmdline = cmdline_path.read_text(errors="ignore").replace("\x00", " ")
        except OSError:
            continue

        if "ros2" not in cmdline and "_ros" not in cmdline and "rcl" not in cmdline:
            continue

        try:
            environ = cmdline_path.with_name("environ").read_bytes().split(b"\x00")
        except OSError:
            continue

        for item in environ:
            if item.startswith(b"ROS_DOMAIN_ID="):
                domain_id = parse_domain_id(item.split(b"=", 1)[1].decode(errors="ignore"))
                if domain_id is not None:
                    return domain_id, str(cmdline_path.with_name("environ"))
                break

    # 没探测到时使用固定默认值，保证节点能稳定启动。
    return ROS_DOMAIN_ID, f"config({ROS_DOMAIN_ID})"


def signal_handler(signal_num, frame):
    global node
    if node is not None:
        node.get_logger().info("接收到退出信号，正在关闭节点...")
        node.destroy_node()
    rclpy.shutdown()
    sys.exit(0)


def main(args=None):
    global node
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    domain_id, source = detect_local_ros_domain_id()
    os.environ["ROS_DOMAIN_ID"] = str(domain_id)

    rclpy.init(args=args)
    node = RobotBridge()
    node.get_logger().info(f"当前使用 ROS_DOMAIN_ID={domain_id}, 来源={source}")

    try:
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    node = None
    main()

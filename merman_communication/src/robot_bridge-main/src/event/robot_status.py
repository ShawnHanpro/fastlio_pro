import json
from typing import Any

from std_msgs.msg import String


ROS_TOPIC = "/merman/chassis/robot_status"
MQTT_EVENT_TOPIC = "robot_status"


def parse_robot_status_data(msg) -> dict[str, Any] | None:
    text = str(getattr(msg, "data", "") or "").strip()
    if not text:
        return None

    parsed = json.loads(text)

    if not isinstance(parsed, dict):
        return {
            "value": parsed
        }

    return parsed


def register_robot_status_event(node, publish_send, resolve_send_topic):
    def on_robot_status(msg):
        msg_text = _safe_message_text(msg)

        try:
            node.get_logger().info(f"监听到 ROS2 消息: topic={ROS_TOPIC}, msg={msg_text}")

            payload_data = parse_robot_status_data(msg)
            if payload_data is None:
                node.get_logger().warn(f"底盘状态消息缺少可用数据，已忽略: msg={msg_text}")
                return

            payload = publish_send(topic=MQTT_EVENT_TOPIC, data=payload_data)
            mqtt_topic = resolve_send_topic(payload.get("robotSn"))

            node.get_logger().info(
                f"已转发底盘状态到 MQTT: ros_topic={ROS_TOPIC}, "
                f"mqtt_topic={mqtt_topic}, payload={payload}"
            )

        except Exception as exc:
            node.get_logger().error(
                f"处理底盘状态消息失败，已跳过当前消息: "
                f"topic={ROS_TOPIC}, error={exc}, msg={msg_text}"
            )

    try:
        subscription = node.create_subscription(String, ROS_TOPIC, on_robot_status, 10)
        node.get_logger().info(f"已订阅底盘状态: ros2 topic={ROS_TOPIC}, msg_type={String.__name__}")
        return subscription

    except Exception as exc:
        node.get_logger().warn(f"未启用底盘状态转发能力: {exc}")
        return None


def _safe_message_text(msg) -> str:
    try:
        return str(msg)
    except Exception as exc:
        return f"<unprintable {type(msg).__name__}: {exc}>"
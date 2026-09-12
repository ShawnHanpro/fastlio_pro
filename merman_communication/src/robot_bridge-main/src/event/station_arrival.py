import json
from typing import Any

from std_msgs.msg import String

from .bool_value import parse_bool


ROS_TOPIC = "/station_arrival_detail"
MQTT_EVENT_TOPIC = "station_arrival_detail"


def parse_station_arrival_data(msg) -> dict[str, Any] | None:
    text = str(getattr(msg, "data", "") or "").strip()
    if not text:
        return None

    parsed = json.loads(text)
    if not isinstance(parsed, dict):
        return None

    station_id = parsed.get("station_id")
    if station_id is None:
        return None

    payload: dict[str, Any] = {
        "station_id": int(station_id),
        "play_audio": parse_bool(parsed.get("play_audio"), default=False),
    }

    action = parsed.get("action")
    if action is not None and str(action).strip():
        payload["action"] = str(action)

    return payload


def register_station_arrival_event(node, publish_send, resolve_send_topic):
    def on_station_arrival_detail(msg):
        msg_text = _safe_message_text(msg)
        try:
            node.get_logger().info(f"监听到 ROS2 消息: topic={ROS_TOPIC}, msg={msg_text}")
            payload_data = parse_station_arrival_data(msg)
            if payload_data is None:
                node.get_logger().warn(f"站点到达消息缺少可用数据，已忽略: msg={msg_text}")
                return

            payload = publish_send(topic=MQTT_EVENT_TOPIC, data=payload_data)
            mqtt_topic = resolve_send_topic(payload.get("robotSn"))
            node.get_logger().info(
                f"已转发站点到达到 MQTT: ros_topic={ROS_TOPIC}, "
                f"mqtt_topic={mqtt_topic}, payload={payload}"
            )
        except Exception as exc:
            node.get_logger().error(
                f"处理站点到达消息失败，已跳过当前消息: "
                f"topic={ROS_TOPIC}, error={exc}, msg={msg_text}"
            )

    try:
        subscription = node.create_subscription(String, ROS_TOPIC, on_station_arrival_detail, 10)
        node.get_logger().info(f"已订阅站点到达: ros2 topic={ROS_TOPIC}, msg_type={String.__name__}")
        return subscription
    except Exception as exc:
        node.get_logger().warn(f"未启用站点到达转发能力: {exc}")
        return None


def _safe_message_text(msg) -> str:
    try:
        return str(msg)
    except Exception as exc:
        return f"<unprintable {type(msg).__name__}: {exc}>"

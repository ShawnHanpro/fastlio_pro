import time
from typing import Any


def build_send_payload(topic: str, data: Any, robot_sn: str, timestamp: int | None = None):
    return {
        "robotSn": robot_sn,
        "topic": topic,
        "data": data,
        "timestamp": int(time.time() * 1000) if timestamp is None else int(timestamp),
    }

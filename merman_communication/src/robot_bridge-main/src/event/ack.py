import time
from typing import Any

from .bool_value import parse_bool


ACK_FLAG_FIELDS = ("needAck", "need_ack", "ack", "requireAck", "require_ack")


def message_requires_ack(payload: dict[str, Any]) -> bool:
    for field in ACK_FLAG_FIELDS:
        if field in payload:
            return parse_bool(payload.get(field), default=False)

    command_type = str(payload.get("commandType") or "").strip().upper()
    return command_type == "CONTROL"


def build_ack_payload(uid: str, action: str, success: bool, message: str, timestamp: int | None = None):
    return {
        "uid": str(uid or "").strip(),
        "success": bool(success),
        "data": {
            "action": action,
            "message": str(message),
        },
        "timestamp": int(time.time() * 1000) if timestamp is None else int(timestamp),
    }

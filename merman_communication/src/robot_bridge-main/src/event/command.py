import json
import time
from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class BridgeCommand:
    uid: str
    robot_sn: str
    action: str
    params: dict[str, Any]
    should_ack: bool


def build_command_payload(action: str, uid: str, robot_sn: str, params: dict[str, Any] | None = None):
    return {
        "uid": uid,
        "robotSn": robot_sn,
        "commandType": "CONTROL",
        "action": action,
        "params": params or {},
        "timestamp": int(time.time() * 1000),
    }


def parse_command_payload(payload: dict[str, Any], should_ack: bool) -> BridgeCommand:
    params = payload.get("params") or {}
    if isinstance(params, str):
        try:
            params = json.loads(params)
        except Exception:
            pass
    if params is None:
        params = {}
    if not isinstance(params, dict):
        params = {"value": params}

    return BridgeCommand(
        uid=str(payload.get("uid") or "").strip(),
        robot_sn=str(payload.get("robotSn") or "").strip(),
        action=str(payload.get("action") or "").strip().lower(),
        params=params,
        should_ack=should_ack,
    )

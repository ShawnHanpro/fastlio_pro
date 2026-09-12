import json
from pathlib import Path
import threading
import time
from typing import Any
from concurrent.futures import ThreadPoolExecutor, Future

try:
    from ..event.ack import build_ack_payload, message_requires_ack as _message_requires_ack
    from ..event.command import build_command_payload as _build_command_payload, parse_command_payload
    from ..event.send import build_send_payload as _build_send_payload
    from .client import ManagedMqttClient, MqttEndpoint
except ImportError:
    from event.ack import build_ack_payload, message_requires_ack as _message_requires_ack
    from event.command import build_command_payload as _build_command_payload, parse_command_payload
    from event.send import build_send_payload as _build_send_payload
    from mqtt.client import ManagedMqttClient, MqttEndpoint

_CONF_PATH = Path(__file__).resolve().parent.parent / "conf.yaml"


def _normalize_prefix(value: str, *, leading_slash: bool = False) -> str:
    prefix = str(value or "").strip()
    if not prefix:
        return ""
    if leading_slash and not prefix.startswith("/"):
        prefix = f"/{prefix}"
    if not prefix.endswith("/"):
        prefix = f"{prefix}/"
    return prefix


def _read_optional_int(data: dict[str, Any], key: str, conf_path: Path) -> int | None:
    value = data.get(key)
    if value is None or str(value).strip() == "":
        return None
    try:
        return int(value)
    except (TypeError, ValueError) as exc:
        raise RuntimeError(f"配置 {key} 必须为整数: {conf_path}") from exc


def _load_settings(conf_path: Path = _CONF_PATH) -> dict[str, Any]:
    if not conf_path.exists():
        raise RuntimeError(f"robot_bridge 缺少配置文件: {conf_path}")

    try:
        import yaml  # type: ignore
    except ImportError as exc:
        raise RuntimeError("读取 robot_bridge/conf.yaml 需要安装 pyyaml: pip install pyyaml") from exc

    with conf_path.open("r", encoding="utf-8") as fp:
        data = yaml.safe_load(fp) or {}

    if not isinstance(data, dict):
        raise RuntimeError(f"配置文件根节点必须为对象: {conf_path}")

    if not isinstance(data, dict):
        raise RuntimeError(f"配置文件根节点必须为对象: {conf_path}")

    robot_sn = str(data.get("robot_sn") or "").strip()
    if not robot_sn:
        raise RuntimeError(f"配置缺少 robot_sn: {conf_path}")

    configured_ros_domain_id = _read_optional_int(data, "ros_domain_id", conf_path)
    if configured_ros_domain_id is not None and configured_ros_domain_id < 0:
        raise RuntimeError(f"配置 ros_domain_id 必须大于等于 0: {conf_path}")

    ros_domain_id = 99 if configured_ros_domain_id is None else configured_ros_domain_id

    mqtt_raw = data.get("mqtt") or {}
    if not isinstance(mqtt_raw, dict):
        raise RuntimeError(f"配置 mqtt 节点必须为对象: {conf_path}")

    broker = str(mqtt_raw.get("broker") or "10.89.33.104").strip()
    if not broker:
        raise RuntimeError(f"配置 mqtt.broker 不能为空: {conf_path}")

    port = int(mqtt_raw.get("port") or 3003)
    username = mqtt_raw.get("username")
    password = mqtt_raw.get("password")
    username = None if username is None else str(username)
    password = None if password is None else str(password)

    topic_prefix = _normalize_prefix(str(mqtt_raw.get("topic_prefix") or "robot/loacl/"))
    topic_cloud_prefix = _normalize_prefix(str(mqtt_raw.get("topic_cloud_prefix") or "robot/info/"))

    topic_local_cmd = (
        str(mqtt_raw.get("topic_local_cmd") or mqtt_raw.get("topic_cmd") or "").strip()
        or f"{topic_prefix}ctl/{robot_sn}"
    )
    topic_cloud_cmd = str(mqtt_raw.get("topic_cloud_cmd") or "").strip() or f"robot/ctl/{robot_sn}"
    topic_ack = str(mqtt_raw.get("topic_ack") or "").strip() or f"{topic_cloud_prefix}{robot_sn}"
    topic_send = str(mqtt_raw.get("topic_send") or "").strip() or f"{topic_prefix}send/{robot_sn}"
    client_id = str(mqtt_raw.get("client_id") or "").strip() or f"robot_bridge_{robot_sn}_{int(time.time())}"

    return {
        "robot_sn": robot_sn,
        "ros_domain_id": ros_domain_id,
        "ros_domain_id_configured": configured_ros_domain_id is not None,
        "mqtt_broker": broker,
        "mqtt_port": port,
        "mqtt_username": username,
        "mqtt_password": password,
        "mqtt_topic_prefix": topic_prefix,
        "mqtt_topic_local_cmd": topic_local_cmd,
        "mqtt_topic_cloud_cmd": topic_cloud_cmd,
        "mqtt_topic_ack": topic_ack,
        "mqtt_topic_send": topic_send,
        "mqtt_client_id": client_id,
    }


_SETTINGS = _load_settings()
ROBOT_SN = _SETTINGS["robot_sn"]
ROS_DOMAIN_ID = _SETTINGS["ros_domain_id"]
ROS_DOMAIN_ID_CONFIGURED = _SETTINGS["ros_domain_id_configured"]
MQTT_BROKER = _SETTINGS["mqtt_broker"]
MQTT_PORT = _SETTINGS["mqtt_port"]
MQTT_CLIENT_ID = _SETTINGS["mqtt_client_id"]
MQTT_USERNAME = _SETTINGS["mqtt_username"]
MQTT_PASSWORD = _SETTINGS["mqtt_password"]
MQTT_TOPIC_PREFIX = _SETTINGS["mqtt_topic_prefix"]
MQTT_TOPIC_LOCAL_CMD = _SETTINGS["mqtt_topic_local_cmd"]
MQTT_TOPIC_CLOUD_CMD = _SETTINGS["mqtt_topic_cloud_cmd"]
MQTT_TOPIC_ACK = _SETTINGS["mqtt_topic_ack"]
MQTT_TOPIC_SEND = _SETTINGS["mqtt_topic_send"]


def mqtt_endpoint() -> MqttEndpoint:
    return MqttEndpoint(
        broker=MQTT_BROKER,
        port=MQTT_PORT,
        username=MQTT_USERNAME,
        password=MQTT_PASSWORD,
    )


def _assert_robot_sn(robot_sn: str | None) -> str:
    value = ROBOT_SN if robot_sn is None else str(robot_sn).strip()
    if value != ROBOT_SN:
        raise ValueError(f"robot_sn 不允许在代码中覆盖，必须使用 conf.yaml: expected={ROBOT_SN}, actual={value}")
    return ROBOT_SN


def build_command_payload(
    action: str,
    uid: str,
    robot_sn: str | None = None,
    params: dict[str, Any] | None = None,
):
    return _build_command_payload(action=action, uid=uid, robot_sn=_assert_robot_sn(robot_sn), params=params)


def resolve_cmd_topic(robot_sn: str | None = None):
    _assert_robot_sn(robot_sn)
    return MQTT_TOPIC_LOCAL_CMD


def resolve_cloud_cmd_topic(robot_sn: str | None = None):
    _assert_robot_sn(robot_sn)
    return MQTT_TOPIC_CLOUD_CMD


def resolve_send_topic(robot_sn: str | None = None):
    _assert_robot_sn(robot_sn)
    return MQTT_TOPIC_SEND


def build_send_payload(topic: str, data: Any, robot_sn: str | None = None, timestamp: int | None = None):
    return _build_send_payload(topic=topic, data=data, robot_sn=_assert_robot_sn(robot_sn), timestamp=timestamp)


def publish_command_payload(payload: dict[str, Any], robot_sn: str | None = None):
    topic = resolve_cmd_topic(robot_sn)
    client_id = f"test_{payload.get('action', 'command')}_{int(time.time())}"
    client = ManagedMqttClient(name="command_publish", endpoint=mqtt_endpoint(), client_id=client_id)
    client.start()
    try:
        client.publish(topic, json.dumps(payload, ensure_ascii=False), qos=1)
    finally:
        client.stop()
    return topic


def publish_send_payload(payload: dict[str, Any], robot_sn: str | None = None, qos: int = 1, retain: bool = False):
    resolved_sn = _assert_robot_sn(robot_sn)
    topic = resolve_send_topic(resolved_sn)
    client_id = f"send_{resolved_sn}_{int(time.time())}"
    client = ManagedMqttClient(name="send_publish", endpoint=mqtt_endpoint(), client_id=client_id, default_qos=qos)
    client.start()
    try:
        client.publish(topic, json.dumps(payload, ensure_ascii=False), qos=qos, retain=retain)
    finally:
        client.stop()
    return topic


class MQTTCommandClient:
    def __init__(self, logger, on_command):
        self.logger = logger
        self.on_command = on_command
        self.client = ManagedMqttClient(
            name="bridge_command",
            endpoint=mqtt_endpoint(),
            client_id=MQTT_CLIENT_ID,
            logger=logger,
            default_qos=1,
        )
        self._executor = ThreadPoolExecutor(max_workers=4)
        self._active_commands: dict[str, Future] = {}
        self._active_lock = threading.Lock()
        self._move_action_lock = threading.Lock()

    def connect(self):
        self.client.subscribe(MQTT_TOPIC_LOCAL_CMD, self._on_message, qos=1)
        if MQTT_TOPIC_CLOUD_CMD != MQTT_TOPIC_LOCAL_CMD:
            self.client.subscribe(MQTT_TOPIC_CLOUD_CMD, self._on_message, qos=1)
        self.client.start()
        self.logger.info(
            "已启动 MQTT 控制订阅:\n"
            f"  broker={MQTT_BROKER}:{MQTT_PORT}\n"
            f"  local={MQTT_TOPIC_LOCAL_CMD}\n"
            f"  cloud={MQTT_TOPIC_CLOUD_CMD}"
        )

    def close(self):
        self._executor.shutdown(wait=True)
        self.client.stop()

    def publish_ack(self, uid, action, success, message):
        uid = str(uid or "").strip()
        if not uid:
            self.logger.warn(f"跳过回执：缺少 uid, action={action}, success={success}, message={message}")
            return

        payload = build_ack_payload(uid=uid, action=action, success=success, message=message)
        ok = self.client.publish(MQTT_TOPIC_ACK, json.dumps(payload, ensure_ascii=False), qos=1, fire_and_forget=True)
        if ok:
            self.logger.info(f"已发布执行回执: topic={MQTT_TOPIC_ACK}, payload={payload}")
        else:
            self.logger.warn(f"发布执行回执失败: topic={MQTT_TOPIC_ACK}, payload={payload}")

    def publish_send(self, topic: str, data: Any, robot_sn: str | None = None, timestamp: int | None = None, qos: int = 1):
        resolved_sn = _assert_robot_sn(robot_sn)
        payload = build_send_payload(topic=topic, data=data, robot_sn=resolved_sn, timestamp=timestamp)
        mqtt_topic = resolve_send_topic(resolved_sn)
        ok = self.client.publish(mqtt_topic, json.dumps(payload, ensure_ascii=False), qos=qos)
        if ok:
            self.logger.info(f"已发布发送消息: topic={mqtt_topic}, payload={payload}")
        else:
            self.logger.warn(f"发布发送消息失败: topic={mqtt_topic}, payload={payload}")
        return payload

    def message_requires_ack(self, payload: dict[str, Any]) -> bool:
        return _message_requires_ack(payload)

    def maybe_publish_ack(self, should_ack: bool, uid: str, action: str, success: bool, message: str) -> None:
        if not should_ack:
            return
        self.publish_ack(uid, action, success, message)

    _MOVE_ACTIONS = {"forward", "back_off", "turn_left", "turn_right", "stop_move"}

    def _on_message(self, topic: str, payload_bytes: bytes):
        try:
            payload = json.loads(payload_bytes.decode("utf-8"))
        except Exception as exc:
            self.logger.error(f"MQTT 消息不是有效 JSON: {exc}")
            return

        if not isinstance(payload, dict):
            self.logger.warn(f"MQTT 消息格式错误，需为对象: {payload}")
            return

        should_ack = self.message_requires_ack(payload)
        command = parse_command_payload(payload, should_ack=should_ack)

        if command.should_ack and not command.uid:
            self.logger.warn(f"消息要求回执但缺少 uid，已忽略: payload={payload}")
            return

        if command.robot_sn and command.robot_sn != ROBOT_SN:
            self.logger.warn(
                f"robotSn 不匹配，忽略消息: expected={ROBOT_SN}, actual={command.robot_sn}, payload={payload}"
            )
            self.maybe_publish_ack(
                command.should_ack,
                command.uid,
                command.action,
                False,
                f"robotSn mismatch: expected={ROBOT_SN}, actual={command.robot_sn}",
            )
            return

        if not command.action:
            self.logger.warn(f"缺少 action，忽略消息: {payload}")
            self.maybe_publish_ack(command.should_ack, command.uid, "", False, "missing action")
            return

        source = "cloud" if topic == MQTT_TOPIC_CLOUD_CMD else "local"
        self.logger.info(
            f"收到 MQTT 指令: source={source}, topic={topic}, payload={payload} "
            f"action={command.action}, params={command.params}"
        )

        future = self._executor.submit(
            self._execute_command,
            command.uid,
            command.action,
            command.params,
            command.should_ack,
        )
        with self._active_lock:
            self._active_commands[command.uid] = future
        future.add_done_callback(lambda f, uid=command.uid: self._on_command_done(uid))

    def _execute_command(self, uid: str, action: str, params: dict, should_ack: bool):
        try:
            if action in self._MOVE_ACTIONS:
                with self._move_action_lock:
                    result = self.on_command(action, params) or {}
            else:
                result = self.on_command(action, params) or {}
            success = bool(result.get("success", True))
            message = result.get("message", "ok")
            self.maybe_publish_ack(should_ack, uid, action, success, message)
        except Exception as exc:
            self.logger.error(f"执行回调失败: action={action}, error={exc}")
            self.maybe_publish_ack(should_ack, uid, action, False, str(exc))

    def _on_command_done(self, uid: str):
        with self._active_lock:
            self._active_commands.pop(uid, None)

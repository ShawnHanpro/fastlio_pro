import threading
from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class MqttEndpoint:
    broker: str
    port: int
    username: str | None = None
    password: str | None = None


class ManagedMqttClient:
    """统一管理 paho MQTT 连接、订阅、发布和停止。"""

    def __init__(
        self,
        *,
        name: str,
        endpoint: MqttEndpoint,
        client_id: str,
        logger=None,
        default_qos: int = 1,
        reconnect_min_delay: int = 1,
        reconnect_max_delay: int = 10,
        publish_connect_wait: float = 3.0,
    ):
        self.name = name
        self.endpoint = endpoint
        self.client_id = client_id
        self.logger = logger
        self.default_qos = int(default_qos or 1)
        self.reconnect_min_delay = int(reconnect_min_delay or 1)
        self.reconnect_max_delay = int(reconnect_max_delay or 10)
        self.publish_connect_wait = max(0.0, float(publish_connect_wait or 0.0))
        self._client = None
        self._subscriptions: dict[str, tuple[int, Any]] = {}
        self._lock = threading.RLock()
        self._connected_event = threading.Event()
        self._connected = False
        self._stopping = False

    @property
    def is_started(self) -> bool:
        return self._client is not None

    @property
    def is_connected(self) -> bool:
        with self._lock:
            return self._connected

    def start(self) -> None:
        with self._lock:
            if self._client is not None:
                return
            try:
                import paho.mqtt.client as mqtt  # type: ignore
            except ImportError as exc:
                raise RuntimeError("MQTT 需要 paho-mqtt: pip install paho-mqtt") from exc

            self._stopping = False
            self._connected = False
            self._connected_event.clear()

            client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION1, client_id=self.client_id)
            if self.endpoint.username and self.endpoint.password:
                client.username_pw_set(self.endpoint.username, self.endpoint.password)
            client.reconnect_delay_set(min_delay=self.reconnect_min_delay, max_delay=self.reconnect_max_delay)
            client.on_connect = self._on_connect
            client.on_disconnect = self._on_disconnect
            for topic, (_qos, handler) in self._subscriptions.items():
                client.message_callback_add(topic, self._wrap_handler(handler))
            client.connect_async(self.endpoint.broker, self.endpoint.port, 60)
            client.loop_start()
            self._client = client
        self._log("info", f"MQTT 客户端已启动: name={self.name}, broker={self.endpoint.broker}:{self.endpoint.port}")

    def subscribe(self, topic: str, handler, *, qos: int | None = None) -> None:
        topic = str(topic or "").strip()
        if not topic:
            raise ValueError("MQTT 订阅 topic 不能为空")
        sub_qos = int(self.default_qos if qos is None else qos)
        with self._lock:
            self._subscriptions[topic] = (sub_qos, handler)
            client = self._client
            if client is not None:
                client.message_callback_add(topic, self._wrap_handler(handler))
                client.subscribe(topic, qos=sub_qos)
        self._log("info", f"MQTT 已订阅: name={self.name}, topic={topic}, qos={sub_qos}")

    def publish(
        self,
        topic: str,
        payload: str,
        *,
        qos: int | None = None,
        retain: bool = False,
        timeout: float = 3.0,
        fire_and_forget: bool = False,
    ) -> bool:
        with self._lock:
            client = self._client
        if client is None:
            self._log("warn", f"MQTT 客户端未启动，跳过发布: name={self.name}, topic={topic}")
            return False
        if not self.wait_connected(self.publish_connect_wait):
            self._log("warn", f"MQTT 未连接，跳过发布: name={self.name}, topic={topic}")
            return False

        info = client.publish(topic, payload, qos=int(self.default_qos if qos is None else qos), retain=retain)
        if fire_and_forget:
            return True
        info.wait_for_publish(timeout=timeout)
        if info.is_published():
            return True
        self._log("warn", f"MQTT 发布超时: name={self.name}, topic={topic}")
        return False

    def stop(self) -> None:
        with self._lock:
            client = self._client
            self._client = None
            self._connected = False
            self._stopping = True
            self._connected_event.clear()
        if client is None:
            return
        try:
            client.loop_stop()
            client.disconnect()
        except Exception:
            pass
        self._log("info", f"MQTT 客户端已停止: name={self.name}")

    def wait_connected(self, timeout: float | None = None) -> bool:
        return self._connected_event.wait(timeout=max(0.0, float(timeout or 0.0)))

    def _on_connect(self, client, userdata, flags, rc) -> None:
        if rc != 0:
            with self._lock:
                self._connected = False
                self._connected_event.clear()
            self._log("error", f"MQTT 连接失败: name={self.name}, rc={rc}")
            return

        with self._lock:
            self._connected = True
            self._connected_event.set()
            subscriptions = list(self._subscriptions.items())

        for topic, (qos, _handler) in subscriptions:
            client.subscribe(topic, qos=qos)

        topics = ",".join(topic for topic, _item in subscriptions) if subscriptions else "无"
        self._log(
            "info",
            f"MQTT 已连接: name={self.name}, broker={self.endpoint.broker}:{self.endpoint.port}, topics={topics}",
        )

    def _on_disconnect(self, client, userdata, rc) -> None:
        with self._lock:
            self._connected = False
            self._connected_event.clear()
            stopping = self._stopping
        if stopping or rc == 0:
            self._log("info", f"MQTT 已断开: name={self.name}, rc={rc}")
            return
        self._log("warn", f"MQTT 异常断开，将自动重连: name={self.name}, rc={rc}")

    @staticmethod
    def _wrap_handler(handler):
        def wrapped(_client, _userdata, msg) -> None:
            handler(msg.topic or "", msg.payload or b"")

        return wrapped

    def _log(self, level: str, message: str) -> None:
        from datetime import datetime
        ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        print(f"[{ts}] [{level.upper()}] [{self.name}] {message}")

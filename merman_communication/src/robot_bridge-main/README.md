# robot_bridge 消息格式约定

本文档约定桥接程序的 MQTT 消息格式。消息分为两个方向：

- 桥接程序对外发出的事件消息
- 其他程序发给桥接程序的控制消息

两个方向使用不同字段，避免把事件和控制指令混用。

## 1. 桥接程序对外发出的事件消息

用于桥接程序把 ROS2 事件转发到 MQTT，例如到站事件
`/station_arrival_detail`。

MQTT Topic：

```text
robot/loacl/send/{robotSn}
```

如果 `conf.yaml` 中配置了 `mqtt.topic_send`，以配置值为准；否则按
`mqtt.topic_prefix + send/{robotSn}` 自动生成。

消息格式统一为：

```json
{
  "robotSn": "sn-sewpg-8s7kP2g9xR5d1m6",
  "topic": "station_arrival_detail",
  "data": {
    "station_id": 1,
    "play_audio": true,
    "action": "introduction_gesture"
  },
  "timestamp": 1779430837872
}
```

字段说明：

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `robotSn` | string | 机器人 SN |
| `topic` | string | 业务事件名，不是 MQTT Topic |
| `data` | object | 业务事件数据 |
| `timestamp` | number | 毫秒时间戳 |

规则：

- 桥接程序对外发出的事件消息使用 `topic + data`。
- 不在顶层放 `action`。
- `data.action` 只表示事件附带动作，例如到站后需要配合的讲解动作。
- `station_arrival_detail` 是事件名，不是控制 action。

### 到站事件示例

ROS2 输入：

```bash
ROS_DOMAIN_ID=99 ros2 topic pub --once -w 0 /station_arrival_detail std_msgs/msg/String "{data: '{\"station_id\":2,\"play_audio\":true,\"action\":\"introduction_gesture\"}'}"
```

桥接发出的 MQTT 消息：

```json
{
  "robotSn": "sn-sewpg-8s7kP2g9xR5d1m6",
  "topic": "station_arrival_detail",
  "data": {
    "station_id": 1,
    "play_audio": true,
    "action": "introduction_gesture"
  },
  "timestamp": 1779430837872
}
```

## 2. 其他程序发给桥接程序的控制消息

用于云端、语音程序或其他程序控制机器人，例如跳点、底盘移动、音频控制、
发布任务等。

MQTT Topic 分为两类：

```text
robot/ctl/{robotSn}
robot/loacl/ctl/{robotSn}
```

其中 `robot/ctl/{robotSn}` 用于云端控制，`robot/loacl/ctl/{robotSn}` 用于本地语音控制。

如果 `conf.yaml` 中配置了 `mqtt.topic_cloud_cmd` 或 `mqtt.topic_local_cmd`，以配置值为准。
否则云端控制默认使用 `robot/ctl/{robotSn}`，本地语音控制默认按
`mqtt.topic_prefix + ctl/{robotSn}` 自动生成。

消息格式统一为：

```json
{
  "uid": "cmd_001",
  "robotSn": "sn-sewpg-8s7kP2g9xR5d1m6",
  "commandType": "CONTROL",
  "action": "jump_to",
  "params": {
    "pointId": 13
  },
  "timestamp": 1779430837872
}
```

字段说明：

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `uid` | string | 指令唯一 ID，用于回执追踪 |
| `robotSn` | string | 机器人 SN |
| `commandType` | string | 固定使用 `CONTROL` |
| `action` | string | 要执行的动作 |
| `params` | object | 动作参数 |
| `needAck` / `need_ack` / `ack` / `requireAck` / `require_ack` | boolean/string | 可选，是否需要回执 |
| `timestamp` | number | 毫秒时间戳 |

规则：

- 其他程序发给桥接程序的控制消息使用 `action + params`。
- 如果消息带 `robotSn`，必须与 `conf.yaml` 中的 `robot_sn` 一致，否则桥接忽略并回执失败。
- `commandType=CONTROL` 默认需要回执；如需关闭或为非 CONTROL 消息开启回执，可显式设置回执开关字段。
- 需要回执的消息必须携带 `uid`，回执会原样带回该 `uid`。
- 云端控制发到 `robot/ctl/{robotSn}`。
- 本地语音控制发到 `robot/loacl/ctl/{robotSn}`。
- 如果要让桥接程序执行 `introduction_gesture`，应放在顶层 `action`。
- 如果 `introduction_gesture` 只是到站事件携带的信息，应放在事件消息的
  `data.action` 中。

### 跳点控制示例

```json
{
  "uid": "jump_to_001",
  "robotSn": "sn-sewpg-8s7kP2g9xR5d1m6",
  "commandType": "CONTROL",
  "action": "jump_to",
  "params": {
    "pointId": 13
  },
  "timestamp": 1779430837872
}
```

### 音频控制示例

```json
{
  "uid": "audio_pause_001",
  "robotSn": "sn-sewpg-8s7kP2g9xR5d1m6",
  "commandType": "CONTROL",
  "action": "tour_audio_pause",
  "params": {},
  "timestamp": 1779430837872
}
```

## 3. 控制回执消息

桥接程序执行控制消息后，会发布回执。

MQTT Topic：

```text
robot/info/{robotSn}
```

如果 `conf.yaml` 中配置了 `mqtt.topic_ack`，以配置值为准；否则按
`mqtt.topic_cloud_prefix + {robotSn}` 自动生成。

消息格式：

```json
{
  "uid": "jump_to_001",
  "success": true,
  "data": {
    "action": "jump_to",
    "message": "jump_to executed"
  },
  "timestamp": 1779430837999
}
```

字段说明：

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `uid` | string | 对应控制消息的 `uid` |
| `success` | boolean | 是否执行成功 |
| `data` | object | 回执业务数据 |
| `data.action` | string | 对应控制消息的 `action` |
| `data.message` | string | 执行结果说明 |
| `timestamp` | number | 毫秒时间戳 |

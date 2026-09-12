# robot_bridge Vibe Coding 指南

本文供代码助手和参与开发的工程师快速理解、修改与验证 `robot_bridge`。开始编码前先阅读本文；消息协议的完整示例以 `README.md` 为准。

## 项目功能

`robot_bridge` 是 ROS2 Humble 与 MQTT 之间的双向桥接服务，主要承担两类工作：

1. 接收 MQTT 控制消息，执行对应的 ROS2 publisher、timer 或 service 操作，并按需发送执行回执。
2. 订阅 ROS2 业务事件，将事件转换成统一 MQTT 消息后上报。

两条数据流必须保持清晰分离：

```text
MQTT action + params  -> command -> ROS2 控制
ROS2 topic            -> event   -> MQTT topic + data
```

事件不是命令。以 `/station_arrival_detail` 为例，`station_arrival_detail` 是事件名；事件中的 `data.action` 是附带业务信息，不等于 MQTT 控制消息顶层的 `action`。

## 程序入口与装配

`src/robot_bridge.py` 是组合根，只负责：

- 创建和销毁 ROS2 Node。
- 创建 `RobotCommandStateMachine`，由状态机装配 command 模块。
- 创建、连接和关闭 `MQTTCommandClient`。
- 注册 event 模块，并持有 ROS2 subscription。
- 将 MQTT 收到的 `action` 和 `params` 分派给 command 状态机。
- 解析和设置 `ROS_DOMAIN_ID`，管理进程信号与 ROS2 生命周期。

不要把具体 topic 的消息构造、参数校验、publisher/service 创建或事件解析重新堆回 `robot_bridge.py`。

## 目录职责

```text
src/
├── robot_bridge.py       # 节点与各组件的总体装配
├── command/              # MQTT -> ROS2 控制业务
│   ├── state_machine.py  # command 注册、setup 和 action 分派
│   └── *.py              # 一个文件对应一类控制业务
├── event/                # ROS2 -> MQTT 事件及消息模型
│   ├── station_arrival.py
│   ├── audio_done.py
│   ├── command.py        # MQTT 控制消息模型与解析
│   ├── send.py           # MQTT 上报消息构造
│   └── ack.py            # 控制回执判断与构造
├── mqtt/
│   ├── client.py         # 通用 MQTT 客户端生命周期
│   └── command_client.py # 配置、控制订阅、并发执行和回执
└── conf.yaml             # robot_sn、ROS domain 和 MQTT 配置
```

## 当前 command 功能

| Action | ROS2 行为 | 实现文件 |
| --- | --- | --- |
| `jump_to` | 发布 `/tour_cmd/jump_to` | `command/jump_to.py` |
| `forward`、`back_off`、`turn_left`、`turn_right`、`stop_move` | 持续发布 `/cmd_vel_web` | `command/move.py` |
| `return_guest_area` | 调用 `/robot/go_welcome` | `command/return_guest_area.py` |
| `return_charging_position` | 调用 `/robot/go_charge` | `command/return_charging_position.py` |
| `publish_mission` | 发布 `/mission_topic` | `command/publish_mission.py` |
| `audio_done` | 发布 `/audio_done` | `command/audio_done.py` |
| `photograph` | 当前返回未接入 ROS | `command/photograph.py` |
| `voice_reply` | 当前返回由 voice 项目处理 | `command/voice_reply.py` |

command 模块可提供：

- `ACTION` 或 `ACTIONS`：声明支持的控制动作。
- `setup(context)`：创建本业务需要的 ROS2 publisher、timer 或 service client。
- `execute(context, params[, action])`：校验参数并执行业务，返回 `{"success": bool, "message": str}`。

新增 command 后，必须在 `command/state_machine.py` 的 `COMMAND_MODULES` 中注册。资源初始化放在对应模块的 `setup()`，不要放进 `RobotBridge`。

## 当前 event 功能

| ROS2 Topic | 行为 | 实现文件 |
| --- | --- | --- |
| `/station_arrival_detail` | 解析到站信息并上报 MQTT `station_arrival_detail` 事件 | `event/station_arrival.py` |
| `/audio_done` | 监听并记录音频结束消息 | `event/audio_done.py` |

event 模块应同时拥有该事件的 topic 常量、消息类型、subscription 注册、数据解析和回调处理。新增 event 后，在 `RobotBridge.event_subscriptions` 中注册并持有返回的 subscription。

## MQTT 消息边界

控制消息使用：

```json
{
  "uid": "cmd_001",
  "robotSn": "robot-sn",
  "commandType": "CONTROL",
  "action": "jump_to",
  "params": {"pointId": 13},
  "timestamp": 1779430837872
}
```

事件上报使用：

```json
{
  "robotSn": "robot-sn",
  "topic": "station_arrival_detail",
  "data": {"station_id": 2, "play_audio": true},
  "timestamp": 1779430837872
}
```

开发时遵守以下约束：

- 控制使用顶层 `action + params`，事件使用顶层 `topic + data`。
- `commandType=CONTROL` 默认需要回执；需要回执的消息必须携带 `uid`。
- 消息中的 `robotSn` 如存在，必须与 `conf.yaml` 一致。
- 不要硬编码 MQTT broker、robot SN 或最终 MQTT topic；统一读取 `conf.yaml` 并使用 `mqtt/command_client.py` 的解析结果。
- 保持现有兼容字段和 ACK 行为，除非需求明确要求变更协议。

## Vibe Coding 工作方式

修改功能时优先做小而完整的垂直切片：

1. 先判断需求属于 `command`、`event` 还是 MQTT 基础设施。
2. 在具体业务模块中完成资源 setup、参数解析、执行或回调。
3. 只在组合根或状态机中增加一处注册，不把业务细节带入装配层。
4. 沿完整方向验证一次：MQTT -> command -> ROS2 -> ACK，或 ROS2 -> event -> MQTT。
5. 同步更新 `README.md` 中对外可见的消息协议和示例。

尽量保持模块可独立理解。topic/service 名称应定义在使用它的业务模块附近；异常信息应包含 action/topic 和关键参数；事件回调应捕获单条坏消息，避免导致节点退出。

## 配置与运行

配置文件为 `src/conf.yaml`。其中 MQTT 凭据属于敏感信息，不要复制到日志、测试样例或新增文档中，也不要在无明确授权时改动生产连接信息。

ROS2 Humble 环境中可从 `src` 目录运行：

```bash
source /opt/ros/humble/setup.bash
cd src
python3 -m robot_bridge
```

也可以构建容器：

```bash
docker build -t robot-bridge ./src
```

`publish_mission` 依赖运行环境提供 `evbhuman_interfaces.msg.MissionArray` 及其 waypoint 消息类型；缺失时节点仍可启动，但该 action 会不可用并输出警告。

## 修改后的最低验证

提交前至少执行：

```bash
python3 -m compileall -q src
```

如果环境具有 ROS2 和 MQTT，再按改动范围进行 topic/service 联调。到站事件可参考 `README.md` 的 `/station_arrival_detail` 发布示例。验证时重点检查：

- 节点启动没有导入或资源初始化错误。
- MQTT 控制 action 路由到正确 command。
- ROS2 事件只上报一次且 payload 方向正确。
- CONTROL 指令成功和失败场景都产生预期 ACK。
- 关闭节点时 MQTT executor 和连接正常退出。

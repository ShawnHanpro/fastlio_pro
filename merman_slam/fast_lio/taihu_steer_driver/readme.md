# taihu_steer_driver

单 EtherCAT 主站（master0）控制 4 个钛虎转向电机，控制模式为 PP（Profile Position，`mode_of_operation = 1`）。

## 启动

```bash
ros2 launch taihu_steer_driver taihu_steer_driver.launch.py
```

## Services

- `/steer/enable`：`std_srvs/srv/Trigger`
- `/steer/disable`：`std_srvs/srv/Trigger`
- `/steer/reset_fault`：`std_srvs/srv/Trigger`

示例：

```bash
ros2 service call /steer/enable std_srvs/srv/Trigger {}
ros2 service call /steer/disable std_srvs/srv/Trigger {}
ros2 service call /steer/reset_fault std_srvs/srv/Trigger {}
```

## Command topic

- `/steer/position_cmd`
  - 类型：`std_msgs/msg/Float64MultiArray`
  - 单位：rad
  - 长度必须为 4

示例：

```bash
ros2 topic pub --once /steer/position_cmd std_msgs/msg/Float64MultiArray "{data: [0.0, 0.2, -0.2, 0.1]}"
```

## Feedback topics

### `/steer/joint_states`
类型：`sensor_msgs/msg/JointState`

- `position`：实际位置，单位 rad
- `velocity`：实际速度，单位 rad/s
- `effort`：实际转矩，单位 N·m（由 `0x6077` 按最大转矩参数换算）

### `/steer/driver_state`
类型：`taihu_steer_driver/msg/DriverState`

消息定义：

- `header`：时间戳等标准头信息
- `joint_names[]`：关节名称数组
- `enabled[]`：每轴使能状态
- `position[]`：每轴实际位置（rad）
- `cmd_position[]`：每轴当前命令位置（rad）
- `velocity[]`：每轴实际速度（rad/s）
- `torque[]`：每轴实际转矩（N·m）
- `status_word[]`：每轴状态字（CiA 402）
- `error_code[]`：每轴错误码
- `mode_display[]`：每轴模式显示值（CiA 402）

查看示例：

```bash
ros2 topic echo /steer/driver_state
```

## 参数

- `motor_offset`
- `motor_dir`
- `encoder_resolution`
- `motor_max_torque_nm`
- `auto_enable`
- EtherCAT 实时参数：`cpu_affinity`、`priority`、`cycle_time_ns` 等

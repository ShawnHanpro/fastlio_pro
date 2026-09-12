# zlac8015d_four_wheel_driver_cpp

ROS 2 C++ package for controlling two ZLAC8015D CANopen motor controllers on one CAN bus.

## Set up Can0
- ip link set can0 down
- ip link set can0 up type can bitrate 500000

## Set Can ID
只连接需要更改id的驱动器，运行：
- cansend can0 601#2B0A200002000000
- cansend can0 601#2B10200001000000
更改id为2
断电重启

## Wheel mapping
- Controller ID=1: right side wheels
  - left motor -> right rear wheelipp
  - right motor -> right front wheel
- Controller ID=2: left side wheels
  - left motor -> left front wheel
  - right motor -> left rear wheel

## Commands
- Service `/wheel_control_can/set_wheel_speeds`: set 4 wheel speeds in rpm and get ack
- Service `/wheel_control_can/enable`: configure both controllers into velocity mode and enable them
- Service `/wheel_control_can/disable`: stop and disable both controllers
- Service `/wheel_control_can/clear_fault`: clear controller faults

## State topic
- Topic `/wheel_control_can/state`: publishes commanded speeds, feedback speeds, per-motor state strings, per-controller state strings, and raw fault codes.

## Notes
- Startup does **not** auto-enable or auto-clear faults.
- Speed feedback is converted from 0.1 rpm to rpm.
- Runtime speed commands use RPDO by default. Startup configuration, enable, clear-fault, and low-rate diagnostics still use SDO.
- With `sync_control=true`, RPDO1 maps `0x60FF:03` (combined target speed: low 16 bits left motor, high 16 bits right motor).


## High-rate topic command

This package also subscribes to `/wheel_control_can/wheel_rpm_cmd` (`std_msgs/msg/Float32MultiArray`).
Data order: `[right_front_rpm, right_rear_rpm, left_front_rpm, left_rear_rpm]`.

Example at 200 Hz:

```bash
ros2 topic pub /wheel_control_can/wheel_rpm_cmd std_msgs/msg/Float32MultiArray "{data: [100.0, 100.0, 100.0, 100.0]}" -r 200
```

Wheel direction can be configured from launch parameters:

- `right_wheel_sign` (default `-1.0`)
- `left_wheel_sign` (default `1.0`)

Communication parameters:

- `use_pdo_commands` (default `true`): send runtime speed commands through RPDO1 instead of SDO.
- `configure_pdo_mapping` (default `true`): configure RPDO1 mapping on enable.
- `sdo_timeout_ms` (default `200`): timeout for startup/configuration SDO transactions.
- `state_sdo_timeout_ms` (default `50`): timeout for low-rate state polling SDO reads.

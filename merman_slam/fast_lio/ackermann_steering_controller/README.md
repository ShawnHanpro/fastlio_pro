# ackermann_steering_controller

Standalone ROS 2 node that wraps `evb_steering_controllers_library`, computes native four-wheel independent steering (4WIS) commands, and talks directly to the Taihu steering driver and ZLAC wheel driver.

## Node

Executable:

```bash
ros2 launch ackermann_steering_controller ackermann_standalone.launch.py
```

Subscribed topics:

- `/cmd_vel` (`geometry_msgs/msg/Twist`)

Published topics:

- `/steer/position_cmd` (`std_msgs/msg/Float64MultiArray`)
- `/wheel_control_can/wheel_rpm_cmd` (`std_msgs/msg/Float32MultiArray`)
- `/odom_wheel` (`nav_msgs/msg/Odometry`)

Subscribed feedback topics:

- `/steer/joint_states` (`sensor_msgs/msg/JointState`)
- `/wheel_control_can/state` (`zlac8015d_four_wheel_driver_cpp/msg/FourWheelState`)

Internal kinematics order is `FL, FR, RL, RR`.

Driver output orders:

- Taihu steering `/steer/position_cmd`: `RR, RL, FL, FR` (unit `rad`)
- ZLAC wheel `/wheel_control_can/wheel_rpm_cmd`: `RF, RR, LF, LR` (unit `rpm`)

## Parameters

The launch file loads `config/ackermann_kinematics.yaml`.

- `wheelbase` [m]
- `track_width` [m]
- `wheel_radius` [m]
- `wheel.max_accel` [rad/s^2]
- `wheel.max_speed` [rad/s]
- `steering.max_angle` [rad]
- `base_frame_id`
- `odom_frame_id`
- `enable_odom_tf`
- `steer_joint_names_rr_rl_fl_fr`

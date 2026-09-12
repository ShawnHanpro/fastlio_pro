# g1_nav2

G1 精简 Nav2 + 四轮四转伪差速控制包。

安装位置：

```text
/home/niic/slam_nav/merman_slam/fast_lio/g1_nav2
```

## 不使用 URDF

本包不启动：

```text
robot_state_publisher
```

也不读取：

```text
URDF
merman_description
```

Nav2 的机器人几何只使用：

```yaml
footprint: "[[0.4, 0.25], [0.4, -0.25], [-0.4, -0.25], [-0.4, 0.25]]"
```

因此 RViz 中要看导航机器人体积轮廓，请显示：

```text
/local_costmap/published_footprint
/global_costmap/published_footprint
```

## 控制结构

```text
teleop_twist_keyboard
        |
        | /cmd_vel
        v
pseudo_diff_4wis_controller
        ^
        | /cmd_vel_nav
        |
       Nav2
```

输出：

```text
/steer/position_cmd
/wheel_control_can/wheel_rpm_cmd
```

优先级：

```text
/cmd_vel 最近 0.5 秒有消息
    -> teleop 优先

否则
    -> /cmd_vel_nav

两路都超时
    -> 驱动轮速度归零
```

## 定位和避障输入

```text
/Odometry_loc
/nav2_scan
/map
map -> odom
odom -> base_link
```

## 编译

```bash
cd /home/niic/slam_nav/merman_slam/fast_lio

source /opt/ros/humble/setup.bash
source /home/niic/slam_nav/merman_ros2_bridge/install/setup.bash

rm -rf build/g1_nav2 install/g1_nav2

colcon build --packages-select g1_nav2

source install/setup.bash
```

## 启动

```bash
ros2 launch g1_nav2 g1_nav2.launch.py
```

## teleop

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

默认 `/cmd_vel` 即可，不需要 remap。

注意：

```text
require_steering_feedback: true
```

默认开启。若收不到 Taihu 的：

```text
/steer/joint_states
```

控制器会安全地将驱动轮 RPM 强制为 0。

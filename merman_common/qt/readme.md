# merman_chassis_panel QT 面板

底盘控制 QT 界面（Qt Widgets + rclcpp），二进制自动拷贝到 `../bin/merman_chassis_panel`。

## 编译

```bash
cd ~/ros_ws/slam_nav/merman_common/qt
cmake -B build -S .        # 删除 build 后需先执行这句（重新配置）
cmake --build build        # 之后每次改代码只需这一句
```

编译成功后二进制会自动拷贝到 `../bin/merman_chassis_panel`。

## 启动

```bash
cd ~/ros_ws/slam_nav/merman_common
./bin/merman_chassis_panel
```

- 不设环境变量时 DOMAIN_ID 默认 98；也可用 `ROS_DOMAIN_ID=98 ./bin/merman_chassis_panel` 指定
- 已有实例在跑时先 `pkill -f bin/merman_chassis_panel` 再启动

## 功能

- DOMAIN_ID 切换（第一行），右侧常驻显示当前模式
- 模式切换：下拉选 建图/定位 → 确认模式，调用 `/system_mode`（std_srvs/SetBool，true=定位）
- 保存地图：`/map_save`（std_srvs/Trigger），结果在底部状态栏
- 方向遥控：`/cmd_vel`（按下持续发布，松开停）
- 录点：`/baselink2map`（nav_msgs/Odometry）位姿写入 `../waypoints/waypoints.yaml`，
  导览点(t)=task、途经点(v)=via，清空点位=删除 yaml（下次录点自动重建）

## 本地联调

无真实服务时可跑 mock 服务端观察 QT 发出的请求：

```bash
python3 mock_services.py                  # 模拟 /system_mode /map_save 并打印收到的请求
ros2 topic pub -r 10 /baselink2map nav_msgs/msg/Odometry "{pose: {pose: {position: {x: 1.5, y: 2.5}, orientation: {w: 1.0}}}}"
ros2 topic echo /cmd_vel
```

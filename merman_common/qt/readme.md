# merman_chassis_panel QT 面板

底盘控制 + 地图显示 QT 界面（Qt Widgets + rclcpp），二进制自动拷贝到 `../bin/merman_chassis_panel`。

## 编译

```bash
cd ~/ros_ws/slam_nav/merman_common/qt
cmake -B build -S .        # build 目录被清空后需先执行这句（重新配置）
cmake --build build        # 之后每次改代码只需这一句
```

编译成功后二进制会自动拷贝到 `../bin/merman_chassis_panel`。
build 目录里只保留最终二进制，中间产物（CMakeCache/CMakeFiles/*.o 等）不入库，
删过一次后重新编译前记得先跑第一句。

## 启动

```bash
cd ~/ros_ws/slam_nav/merman_common
ROS_DOMAIN_ID=9 FASTDDS_BUILTIN_TRANSPORTS=UDPv4 ./bin/merman_chassis_panel
```

- `ROS_DOMAIN_ID` 必须与机器人一致，不设时默认 98
- 建议带 `FASTDDS_BUILTIN_TRANSPORTS=UDPv4`，与机器人 bringup 脚本保持一致
- 已有实例在跑时先 `kill $(pgrep merman_chassis_panel)` 再启动

## 功能

- DOMAIN_ID 切换（第一行）：**后台线程**执行节点销毁/重建（坏网络上 DDS 销毁可能耗时数秒），
  期间按钮置灰、其它操作提示稍候，GUI 不会卡；右侧常驻显示当前模式
- 模式切换：下拉选 建图/定位 → 确认模式，调用 `/system_mode`（std_srvs/SetBool，true=定位）
  - **全程不阻塞 GUI**：不 wait_for_service，直接异步发请求 + 定时器轮询
  - 响应丢失/服务未拉起时 **5 秒明确报超时**，界面不会无声挂死
  - 重复点击在途请求期间会被拒绝
- 保存地图：`/map_save`（std_srvs/Trigger），仅建图模式可用，超时 10 秒兜底
- 方向遥控：`/cmd_vel`（按下持续发布，松开停）——**不依赖任何服务**，服务全挂也能用
- 点位管理：`/baselink2map`（nav_msgs/Odometry）位姿写入 `../waypoints/waypoints.yaml`
  - 录点：导览点(t)=task、途经点(v)=via，追加到文件末尾；清空全部点位=删除 yaml
  - 编辑行：输入 id + 选类型 → 确认修改（用当前位姿覆盖该点 x/y/theta/type）、
    插入（当前位姿插到 id 位置，之后点位 id 依次 +1）、
    删除（删掉该点，之后点位 id 依次 -1）
  - 只改 QT 本地文件（现场流程：本地录完 scp 到机器人目录），
    机器人侧 save_waypoints.py 无需任何改动；同样不依赖服务
- 模式显示不依赖服务确认：机器人开机即定位模式 → 默认显示“定位(默认)”；
  FAST-LIO 的 `/map_2d` 只在建图模式发布，收到时间戳连续前进的新帧时
  自动显示“建图(自动检测)”并放开保存地图按钮（服务没响应也能反映真实模式）
- 地图显示（左侧）：
  - 建图模式订阅 FAST-LIO `/map_2d`（地图变化时每秒一帧，逐秒增量显示）
  - 定位模式显示 map_server `/map`（已保存的完整地图）
  - 无数据时黑屏；两路按 header 时间戳取新
  - 叠加机器人位姿箭头（红色）、`/nav2_scan` 实时激光（绿色，rviz 风格，
    imu_link → base_link → 地图系变换，数据断流 1.5 秒后自动消隐）
    和 waypoints.yaml 点位（导览点=蓝圆、途经点=橙方块）
  - 滚轮缩放、左键拖拽平移、双击恢复自适应全图；左下角显示数据源和地图参数
  - **QoS 注意**：`/map` 必须用 RELIABLE+TRANSIENT_LOCAL 订阅（map_server 只发一次
    锁存帧，Fast DDS 不给 best_effort 订阅者投递锁存帧，用 best_effort 会收不到）；
    `/map_2d` 发布端是 BEST_EFFORT，订阅只能用 BEST_EFFORT，两路 QoS 不能共用

## 现场网络要点（网线直连 / 双 WiFi 场景）

Fast DDS 的 UDPv4 传输：发现走组播 239.255.0.1，数据走单播、按对方通告的所有网卡 IP + 本机路由表发送。
“发现成功 ≠ 数据可达”——组播在直连网线上是通的，但单播可能被路由到错误网卡后全丢，
典型症状就是：DOMAIN 同步正常（纯本地操作）、点确认模式后永远无响应。

- 网线两端配 **独立网段** 的静态 IP（如 192.168.137.1/2），避开两侧 WiFi 和 VPN 的网段
- 接好线后先验证路由：`ip route get <机器人网线IP>`，输出必须是 `dev enp0s31f6`（有线口）
- 笔记本同时连 WiFi + 网线时，若 WiFi 网段与网线网段相同，Linux 会把去往机器人的包
  从 WiFi 发出去，数据全丢——这是双 WiFi + 网线场景卡死的常见根因

## 本地联调

无真实服务时可跑 mock 服务端观察 QT 发出的请求：

```bash
python3 mock_services.py                  # 模拟 /system_mode /map_save 并打印收到的请求
ros2 topic pub -r 10 /baselink2map nav_msgs/msg/Odometry "{pose: {pose: {position: {x: 1.5, y: 2.5}, orientation: {w: 1.0}}}}"
ros2 topic echo /cmd_vel
```

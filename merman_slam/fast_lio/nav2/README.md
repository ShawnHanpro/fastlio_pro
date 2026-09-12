# EVBHuman Navigation2 Package

这是用于 EVBHuman 机器人在 Gazebo 仿真环境中进行导航的 ROS2 Nav2 功能包。

## 功能特性

- 基于已建地图的自主导航
- AMCL 定位
- 动态路径规划
- 障碍物避障
- 支持 Gazebo 仿真环境

## 文件结构

```
nav2/
├── launch/
│   └── navigation2_gazebo.launch.py  # 主导航启动文件
├── map/
│   ├── rtabmap_lidar.yaml            # 地图配置文件
│   └── rtabmap_lidar.pgm             # 地图图像
├── param/
│   └── evbhuman_navigation2.yaml     # Nav2 参数配置
├── CMakeLists.txt
├── package.xml
└── README.md
```

## 依赖项

确保已安装以下 ROS2 包：
- nav2_bringup
- nav2_bt_navigator
- nav2_controller
- nav2_planner
- nav2_amcl
- nav2_map_server
- rviz2

## 使用方法

### 1. 编译功能包

在 motion_control 工作空间根目录下：

```bash
cd /home/test/workspace/motion_control
colcon build --packages-select nav2
source install/setup.bash
```

### 2. 启动 Gazebo 仿真

首先需要启动你的机器人 Gazebo 仿真环境（包含机器人模型和传感器）：

```bash
# 示例：启动机器人仿真（根据你的实际启动文件调整）
ros2 launch evbhuman_description gazebo.launch.py
```

### 3. 启动导航

在新终端中，source 工作空间并启动导航：

```bash
cd /home/test/workspace/motion_control
source install/setup.bash
ros2 launch nav2 navigation2_gazebo.launch.py
```

### 4. 启动 RViz 可视化

在另一个终端中启动 RViz（使用已添加 Nav2 组件的配置）：

```bash
cd /home/test/workspace/motion_control
source install/setup.bash
ros2 run rviz2 rviz2 -d src/evbhuman_description/rviz/view.rviz
```

### 5. 在 RViz 中设置初始位姿和目标点

导航启动后：

1. **设置初始位姿（2D Pose Estimate）**：
   - 在 RViz 工具栏点击 "2D Pose Estimate"
   - 在地图上点击并拖动鼠标设置机器人的初始位置和方向
   - 这会帮助 AMCL 进行定位

2. **发送导航目标（2D Goal Pose）**：
   - 在 RViz 工具栏点击 "2D Goal Pose"
   - 在地图上点击并拖动鼠标设置目标位置和方向
   - 机器人会自动规划路径并导航到目标点

### 6. 启动参数

可用的启动参数：

- `use_sim_time`: 是否使用仿真时间（默认: true）
- `map`: 地图文件路径（默认: rtabmap_lidar.yaml）
- `params_file`: 参数文件路径（默认: evbhuman_navigation2.yaml）
- `autostart`: 是否自动启动导航节点（默认: true）

示例：使用自定义参数启动

```bash
ros2 launch nav2 navigation2_gazebo.launch.py \
    map:=/path/to/your/map.yaml \
    autostart:=true
```

## 配置调整

### 机器人尺寸调整

如果你的机器人尺寸与默认设置不同，需要修改 `param/evbhuman_navigation2.yaml` 中的 `footprint` 参数：

```yaml
footprint: "[[0.4, 0.25], [0.4, -0.25], [-0.4, -0.25], [-0.4, 0.25]]"
```

格式为机器人轮廓的各个顶点坐标 [x, y]，单位为米。

### 速度参数调整

根据你的机器人性能，可以调整最大速度参数：

```yaml
controller_server:
  ros__parameters:
    FollowPath:
      max_vel_x: 0.26        # 最大线速度 (m/s)
      max_vel_theta: 1.0     # 最大角速度 (rad/s)
```

### 激光雷达话题

确保激光雷达话题 `/scan` 正确发布，如果话题名称不同，需要在参数文件中修改：

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      voxel_layer:
        scan:
          topic: /scan  # 修改为你的激光雷达话题
```

## RViz 显示组件

已在 `evbhuman_description/rviz/view.rviz` 中添加了以下 Nav2 组件：

- **Map** - 显示已建立的地图
- **LaserScan** - 激光雷达扫描数据（红色）
- **Global Costmap** - 全局代价地图
- **Local Costmap** - 局部代价地图
- **Global Plan** - 全局路径规划（绿色）
- **Local Plan** - 局部路径规划（黄色）
- **Robot Footprint** - 机器人轮廓（绿色）
- **Goal Point** - 目标点显示（紫色）
- **VoxelGrid** - 体素网格（障碍物检测）
- **Navigation 2 面板** - Nav2 控制面板

## 故障排除

### 问题 1: 机器人不移动

- 检查是否正确设置了初始位姿（2D Pose Estimate）
- 确认 `/scan` 话题有数据：`ros2 topic echo /scan`
- 确认 `/odom` 话题有数据：`ros2 topic echo /odom`

### 问题 2: 路径规划失败

- 确认地图正确加载
- 检查目标点是否在可行区域内
- 调整 `planner_server` 中的 `tolerance` 参数

### 问题 3: TF 变换错误

- 确保机器人的 TF 树正确发布
- 检查 `base_link`, `odom`, `map` 等坐标系的变换关系
- 运行 `ros2 run tf2_tools view_frames` 查看 TF 树

## 参考资料

- [Nav2 官方文档](https://navigation.ros.org/)
- [Nav2 参数配置指南](https://navigation.ros.org/configuration/index.html)

## 许可证

Apache-2.0

